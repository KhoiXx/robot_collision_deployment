import copy
import sys
from enum import Enum

import numpy as np
import rospy
import tf
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, Float32MultiArray

from settings import *
from collections import deque


class RobotStatus(Enum):
    IDLE = 0
    RUNNING = 1
    STUCK = 2


class RobotEnv:
    def __init__(self, index: int):
        # rospy.init_node(f"robot_env_{index}", anonymous=True)
        self.goal_size = 0.15
        self.distance = 0.0
        self.avg_speed = 0.0
        self.speed_count = 0

        # Robot index for unique topic names
        self.index = index

        # Subscribe to IMU, Lidar, and Odometry topics
        self.odom_sub = rospy.Subscriber(f"/robot_{self.index}/odometry/filtered", Odometry, self.odom_callback)
        self.amcl = rospy.Subscriber(f"/robot_{self.index}/amcl_pose", PoseWithCovarianceStamped, self.amcl_callback)

        self.cmd_vel = rospy.Publisher(f"/robot_{self.index}/cmd_vel", Twist, queue_size=10)
        self.cmd_pose = rospy.Publisher(f"/robot_{self.index}/cmd_pose", Twist, queue_size=10)
        # Publish crash topic
        self.crash_pub = rospy.Publisher(f"/robot_{self.index}/is_crashed", Bool, queue_size=10)
        self.lidar_sub = rospy.Subscriber(f"/robot_{self.index}/scan", LaserScan, self.lidar_callback)

        self.rate = rospy.Rate(10)  # 10 Hz

        self.goal_point = [0.0, 0.0]
        self.scan = np.array([])
        self.is_crash = False

        self.state = None
        self.speed = None
        self.speed_GT = None
        self.state_GT = None

        # TODO: dung amcl de xac dinh vi tri ground truth voi lidar
        self.amcl_prev_time = None
        self.amcl_prev_position = None
        self.amcl_prev_yaw = None

        # Track AMCL update rate
        self.amcl_update_times = deque(maxlen=100)
        self.amcl_update_count = 0

    def set_new_goal(self, goal: list):
        self.goal_point = goal
        [x, y] = self.get_local_goal()
        self.pre_distance = np.sqrt(x ** 2 + y ** 2)
        self.distance = copy.deepcopy(self.pre_distance)

    def lidar_callback(self, data: LaserScan):
        """
        scan_msg: LaserScan
        """
        self.scan = np.asarray(data.ranges)[:NUM_OBS]
        crash_distance = ROBOT_RADIUS + SAFE_DISTANCE
        valid_scan = self.scan[np.isfinite(self.scan)]
        is_crash = np.any(valid_scan <= crash_distance)
        self.crash_pub.publish(is_crash)
        self.is_crash = is_crash

    def get_laser_observation(self):
        """
        Process laser scan to match training format (454 observations)
        FIXED: Now returns 454 observations to match training
        """
        scan = copy.deepcopy(self.scan)
        scan[np.isnan(scan)] = 6.0
        scan[np.isinf(scan)] = 6.0
        scan = np.where(scan > 6.0, 6.0, scan)

        raw_beam_num = len(scan)
        sparse_beam_num = NUM_OBS  # Now 454 to match training

        # Downsample laser scan to sparse_beam_num points
        step = float(raw_beam_num) / sparse_beam_num
        sparse_scan_left = []
        index = 0.0
        for x in range(int(sparse_beam_num / 2)):
            sparse_scan_left.append(scan[int(index)])
            index += step
        sparse_scan_right = []
        index = raw_beam_num - 1.0
        for x in range(int(sparse_beam_num / 2)):
            sparse_scan_right.append(scan[int(index)])
            index -= step
        scan_sparse = np.concatenate((sparse_scan_left, sparse_scan_right[::-1]), axis=0)

        # Normalize to [-0.5, 0.5] range (matching training)
        return scan_sparse / 6.0 - 0.5

    def odom_callback(self, odometry: Odometry):
        pos = odometry.pose.pose.position
        quat = odometry.pose.pose.orientation
        euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        yaw = normalize_angle(euler[2])
        # Use UKF-filtered odometry (combines encoder + IMU)
        self.state = [pos.x, pos.y, yaw]

        vx = odometry.twist.twist.linear.x
        wz = odometry.twist.twist.angular.z
        self.speed = [vx, wz]
    
    def amcl_callback(self, amcl_pose: PoseWithCovarianceStamped):
        pos = amcl_pose.pose.pose.position
        quat = amcl_pose.pose.pose.orientation
        euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        yaw = normalize_angle(euler[2])
        self.state_GT = [pos.x, pos.y, yaw]

        # Track AMCL update rate
        current_time = rospy.Time.now().to_sec()
        self.amcl_update_times.append(current_time)
        self.amcl_update_count += 1

        # Log AMCL rate every 100 updates
        if self.amcl_update_count % 100 == 0 and len(self.amcl_update_times) > 10:
            intervals = [self.amcl_update_times[i] - self.amcl_update_times[i-1]
                        for i in range(1, len(self.amcl_update_times))]
            avg_interval = np.mean(intervals)
            amcl_rate = 1.0 / avg_interval if avg_interval > 0 else 0
            rospy.loginfo(f"[AMCL] Rate: {amcl_rate:.1f} Hz | Interval: {avg_interval*1000:.1f}ms")

        # Calculate speed
        current_position = np.array([pos.x, pos.y])

        current_yaw = euler[2]
        if self.amcl_prev_time is not None:
            delta_t = current_time - self.amcl_prev_time
            if delta_t > 0:
                # FIXED: Correct direction (current - previous)
                delta_position = current_position - self.amcl_prev_position
                linear_vel = np.linalg.norm(delta_position) / delta_t
                delta_yaw = current_yaw - self.amcl_prev_yaw
                angular_vel = normalize_angle(delta_yaw) / delta_t
                self.speed_GT = [linear_vel, angular_vel]
        else:
            self.speed_GT = [0.0, 0.0]
        self.amcl_prev_time = current_time
        self.amcl_prev_position = current_position
        self.amcl_prev_yaw = current_yaw

    def get_local_goal(self):
        # Use AMCL if available, fallback to UKF odom if AMCL stale
        if self.state_GT is not None:
            [x, y, theta] = self.state_GT
        else:
            # Fallback to UKF odometry if AMCL not available
            [x, y, theta] = self.state if self.state is not None else [0, 0, 0]

        [goal_x, goal_y] = self.goal_point
        local_x = (goal_x - x) * np.cos(theta) + (goal_y - y) * np.sin(theta)
        local_y = -(goal_x - x) * np.sin(theta) + (goal_y - y) * np.cos(theta)
        return [local_x, local_y]

    def get_reward_and_terminate(self, t):
        """
        Reward function - SYNCED with training stage_world2.py
        Same structure as training for consistent deployment behavior
        """
        terminate = False
        # Get current robot state
        laser_scan_raw = self.get_laser_observation()  # Normalized [-0.5, 0.5]
        laser_scan = (laser_scan_raw + 0.5) * 6.0  # Denormalize to [0, 6] meters
        [x, y, theta] = self.state_GT
        [v, w] = self.speed

        self.pre_distance = copy.deepcopy(self.distance)
        self.distance = np.sqrt((self.goal_point[0] - x) ** 2 + (self.goal_point[1] - y) ** 2)

        # Get front obstacle distance - SAME as training
        n = len(laser_scan)
        front_distances = laser_scan[n//3:2*n//3]  # Front 1/3 of scan
        front_distances = front_distances[front_distances < 6.0]  # Filter valid readings
        min_obstacle_dist = np.mean(np.sort(front_distances)[:5]) if len(front_distances) > 0 else 6.0

        # ========================================
        # TERMINAL REWARDS - SAME as training
        # ========================================
        if self.distance < self.goal_size:
            return 30.0, True, 'Reach Goal'
        if self.is_crash:
            return -25.0, True, 'Crash'
        if t >= 700:  # Match training timeout
            return -10.0, True, 'Timeout'

        # ========================================
        # STEP REWARDS - SYNCED with training
        # ========================================
        reward = 0.0
        progress = self.pre_distance - self.distance

        # 1. Progress reward - training proven value
        reward += 2.0 * progress

        # 2. Safety reward - training 2-zone system
        if min_obstacle_dist < 0.35:
            # Very close - strong speed penalty
            safe_speed = 0.2
            if v > safe_speed:
                reward -= 0.3 * (v - safe_speed)
        elif min_obstacle_dist < 0.6:
            # Close - moderate speed limit
            safe_speed = 0.4
            if v > safe_speed:
                reward -= 0.1 * (v - safe_speed)

        # 3. Rotation penalty - training proven value
        if np.abs(w) > 0.8:
            reward -= 0.06 * np.abs(w)

        # 4. Heading bonus - ALWAYS ACTIVE (training proven)
        local_goal = self.get_local_goal()
        if min_obstacle_dist > 0.6:
            goal_angle = np.arctan2(local_goal[1], local_goal[0])
            heading_error = np.abs(goal_angle) / np.pi
            if heading_error < 0.3:  # Only reward when reasonably aligned
                reward += 0.1 * (1.0 - heading_error)

        # 5. Velocity smoothing - ALWAYS ACTIVE (training proven)
        if hasattr(self, 'prev_linear_vel'):
            vel_change = abs(v - self.prev_linear_vel)
            if vel_change > 0.1:  # Threshold for sudden change
                reward -= 0.05 * vel_change
        self.prev_linear_vel = v

        return reward, False, None

    def control_vel(self, action):
        move_cmd = Twist()
        move_cmd.linear.x = action[0]
        move_cmd.linear.y = 0.0
        move_cmd.linear.z = 0.0
        move_cmd.angular.x = 0.0
        move_cmd.angular.y = 0.0
        move_cmd.angular.z = action[1]
        self.cmd_vel.publish(move_cmd)
        self.avg_speed += action[0]
        self.speed_count += 1

    def control_pose(self, pose):
        pose_cmd = Pose()
        assert len(pose) == 3
        pose_cmd.position.x = pose[0]
        pose_cmd.position.y = pose[1]
        pose_cmd.position.z = 0

        qtn = tf.transformations.quaternion_from_euler(0, 0, pose[2], "rxyz")
        pose_cmd.orientation.x = qtn[0]
        pose_cmd.orientation.y = qtn[1]
        pose_cmd.orientation.z = qtn[2]
        pose_cmd.orientation.w = qtn[3]
        self.cmd_pose.publish(pose_cmd)

    def stop(self):
        self.control_vel([0.0, 0.0])

def normalize_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


#     def compute_control(self):
#         pass

#     def publish_control(self):
#         # Publish motor commands to ESP
#         motor_cmd = Float32MultiArray()
#         motor_cmd.data = [self.linear_velocity, self.angular_velocity]
#         self.motor_pub.publish(motor_cmd)

#         # Publish velocity for RViz visualization
#         vel_msg = Twist()
#         vel_msg.linear.x = self.linear_velocity
#         vel_msg.angular.z = self.angular_velocity
#         self.vel_pub.publish(vel_msg)
