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
        self.state = [round(pos.x, 4), round(pos.y, 4), yaw]

        self.state = [odometry.pose.pose.position.x, odometry.pose.pose.position.y, euler[2]]

        vx = odometry.twist.twist.linear.x
        wz = odometry.twist.twist.angular.z
        self.speed = [round(vx, 4), round(wz, 4)]
    
    def amcl_callback(self, amcl_pose: PoseWithCovarianceStamped):
        pos = amcl_pose.pose.pose.position
        quat = amcl_pose.pose.pose.orientation
        euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        yaw = normalize_angle(euler[2])
        self.state_GT = [pos.x, pos.y, yaw]

        # Calculate speed
        current_time = rospy.Time.now().to_sec()
        current_position = np.array([pos.x, pos.y])

        current_yaw = euler[2]
        if self.amcl_prev_time is not None:
            delta_t = current_time - self.amcl_prev_time
            if delta_t > 0:
                delta_position = self.amcl_prev_position - current_position
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
        [x, y, theta] = self.state_GT
        [goal_x, goal_y] = self.goal_point
        local_x = (goal_x - x) * np.cos(theta) + (goal_y - y) * np.sin(theta)
        local_y = -(goal_x - x) * np.sin(theta) + (goal_y - y) * np.cos(theta)
        return [local_x, local_y]

    def get_reward_and_terminate(self, t):
        terminate = False
        laser_scan = self.get_laser_observation()
        [x, y, theta] = self.state_GT
        [v, w] = self.speed_GT
        self.pre_distance = copy.deepcopy(self.distance)
        self.distance = np.sqrt((self.goal_point[0] - x) ** 2 + (self.goal_point[1] - y) ** 2)
        reward_g = (self.pre_distance - self.distance) * 2.5
        reward_c = 0
        reward_w = 0
        result = 0
        reward_p = 0
        reward_theta = 0
        is_crash = self.is_crash

        # Phần thưởng căn chỉnh hướng
        if self.distance < 0.7:
            angle_to_goal = np.arctan2(self.goal_point[1] - y, self.goal_point[0] - x)
            angle_diff = normalize_angle(angle_to_goal - theta)
            reward_theta = -0.3 * np.abs(angle_diff)
        else:
            if np.abs(w) > 1.45:
                reward_w = -0.1 * np.abs(w)

        min_distance_to_obstacle = np.min(laser_scan)
        if min_distance_to_obstacle < 0:
            reward_p = -0.25
        elif min_distance_to_obstacle < 0.25:
            reward_p = (min_distance_to_obstacle - 0.25) / 2

        if self.distance < self.goal_size:
            terminate = True
            reward_g = 40
            result = "Reach Goal"

        if is_crash == 1:
            # TODO: add a recover mode
            terminate = True
            reward_c = -15.0
            result = "Crashed"

        if t > 10000:
            terminate = True
            result = "Time out"
        reward = reward_g + reward_c + reward_w + reward_p + reward_theta
        return reward, terminate, result

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
