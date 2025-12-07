#!/usr/bin/env python3
"""
Debug goal transformation issue
Check if robot can calculate local goal correctly
"""

import rospy
import tf
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

class GoalTransformDebugger:
    def __init__(self):
        rospy.init_node('goal_transform_debugger', anonymous=True)

        self.tf_listener = tf.TransformListener()

        # Current robot pose
        self.robot_pose_map = None  # In map frame
        self.robot_pose_odom = None  # In odom frame

        # Subscribe to odometry
        rospy.Subscriber("/robot_0/odometry/filtered", Odometry, self.odom_callback)

        # Subscribe to goal
        rospy.Subscriber("/robot_0/move_base_simple/goal", PoseStamped, self.goal_callback)

        rospy.loginfo("="*70)
        rospy.loginfo("GOAL TRANSFORM DEBUGGER")
        rospy.loginfo("="*70)
        rospy.loginfo("Waiting for odometry and goals...")

    def odom_callback(self, msg):
        """Track robot pose in odom frame"""
        self.robot_pose_odom = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'yaw': self.get_yaw(msg.pose.pose.orientation)
        }

        # Try to get robot pose in map frame via TF
        try:
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/robot_0/odom', rospy.Time(0))

            # Robot in map = TF(map->odom) + robot in odom
            map_x = trans[0] + self.robot_pose_odom['x']
            map_y = trans[1] + self.robot_pose_odom['y']
            map_yaw = self.get_yaw_from_quat(rot) + self.robot_pose_odom['yaw']

            self.robot_pose_map = {
                'x': map_x,
                'y': map_y,
                'yaw': map_yaw
            }

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            self.robot_pose_map = None

    def goal_callback(self, msg):
        """Debug goal transformation"""
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        goal_yaw = self.get_yaw(msg.pose.orientation)
        goal_frame = msg.header.frame_id

        print("\n" + "="*70)
        print("NEW GOAL RECEIVED")
        print("="*70)
        print(f"Goal position: ({goal_x:.3f}, {goal_y:.3f})")
        print(f"Goal yaw: {np.degrees(goal_yaw):.1f}°")
        print(f"Goal frame: {goal_frame}")
        print("-"*70)

        # Check robot pose in odom
        if self.robot_pose_odom is not None:
            print("Robot pose (odom frame):")
            print(f"  Position: ({self.robot_pose_odom['x']:.3f}, {self.robot_pose_odom['y']:.3f})")
            print(f"  Yaw: {np.degrees(self.robot_pose_odom['yaw']):.1f}°")
        else:
            print("⚠️  Robot pose (odom) NOT AVAILABLE!")

        print("-"*70)

        # Check robot pose in map
        if self.robot_pose_map is not None:
            print("Robot pose (map frame) - via TF:")
            print(f"  Position: ({self.robot_pose_map['x']:.3f}, {self.robot_pose_map['y']:.3f})")
            print(f"  Yaw: {np.degrees(self.robot_pose_map['yaw']):.1f}°")
        else:
            print("⚠️  Robot pose (map) NOT AVAILABLE!")
            print("⚠️  TF map->odom NOT FOUND!")
            print("⚠️  Robot CANNOT calculate position on map!")
            print("")
            print("💡 SOLUTION: Set initial pose in RViz using '2D Pose Estimate'")

        print("-"*70)

        # Calculate local goal if possible
        if goal_frame == 'map':
            if self.robot_pose_map is not None:
                # Goal in map, robot in map → calculate local goal
                dx = goal_x - self.robot_pose_map['x']
                dy = goal_y - self.robot_pose_map['y']

                # Transform to robot frame
                robot_yaw = self.robot_pose_map['yaw']
                local_x = dx * np.cos(robot_yaw) + dy * np.sin(robot_yaw)
                local_y = -dx * np.sin(robot_yaw) + dy * np.cos(robot_yaw)

                distance = np.sqrt(local_x**2 + local_y**2)
                angle_to_goal = np.degrees(np.arctan2(local_y, local_x))

                print("Local goal (in robot frame):")
                print(f"  Forward: {local_x:.3f}m")
                print(f"  Left: {local_y:.3f}m")
                print(f"  Distance: {distance:.3f}m")
                print(f"  Angle: {angle_to_goal:.1f}°")

                if abs(angle_to_goal) < 45:
                    direction = "FORWARD"
                elif abs(angle_to_goal) > 135:
                    direction = "BACKWARD ⚠️"
                elif angle_to_goal > 0:
                    direction = "LEFT"
                else:
                    direction = "RIGHT"

                print(f"  → Robot should go: {direction}")

                if local_x < 0:
                    print("")
                    print("⚠️⚠️⚠️ WARNING: Goal is BEHIND robot! ⚠️⚠️⚠️")
                    print("This means:")
                    print("  1. Initial pose was set WRONG, OR")
                    print("  2. Robot orientation is 180° off")

            else:
                print("❌ CANNOT calculate local goal!")
                print("❌ Robot doesn't know its position on map (TF missing)")
                print("❌ Robot will NOT navigate correctly!")

        elif goal_frame == 'robot_0/odom':
            # Goal in odom frame - direct calculation
            if self.robot_pose_odom is not None:
                dx = goal_x - self.robot_pose_odom['x']
                dy = goal_y - self.robot_pose_odom['y']

                robot_yaw = self.robot_pose_odom['yaw']
                local_x = dx * np.cos(robot_yaw) + dy * np.sin(robot_yaw)
                local_y = -dx * np.sin(robot_yaw) + dy * np.cos(robot_yaw)

                distance = np.sqrt(local_x**2 + local_y**2)
                angle_to_goal = np.degrees(np.arctan2(local_y, local_x))

                print("Local goal (in robot frame):")
                print(f"  Forward: {local_x:.3f}m")
                print(f"  Left: {local_y:.3f}m")
                print(f"  Distance: {distance:.3f}m")
                print(f"  Angle: {angle_to_goal:.1f}°")

        print("="*70)

    def get_yaw(self, orientation):
        """Get yaw from quaternion"""
        quat = [orientation.x, orientation.y, orientation.z, orientation.w]
        euler = tf.transformations.euler_from_quaternion(quat)
        return euler[2]

    def get_yaw_from_quat(self, quat):
        """Get yaw from quaternion tuple"""
        euler = tf.transformations.euler_from_quaternion(quat)
        return euler[2]

if __name__ == '__main__':
    try:
        debugger = GoalTransformDebugger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
