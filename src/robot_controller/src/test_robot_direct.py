#!/usr/bin/env python3
"""
SIMPLE TEST SCRIPT - Không cần service, không cần Jetson
Chỉ test giao tiếp ESP32 + điều khiển
"""

import rospy
import sys
import os

# Add src to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src/robot_controller/src'))

from robot_control import RobotControl

if __name__ == "__main__":
    try:
        # Init node trước
        rospy.init_node("robot_test_node")

        # Robot index = 0 (single robot test)
        index = 0

        # Port - thay đổi nếu cần
        port = rospy.get_param('~port', '/dev/ttyUSB0')
        baudrate = 115200

        rospy.loginfo(f"Starting robot test on port {port}")

        # Tạo robot control
        robot_control_node = RobotControl(index, port, baudrate=baudrate)

        rospy.loginfo("Robot controller ready! Use teleop_twist_keyboard to control.")
        rospy.loginfo("Subscribing to: /robot_0/cmd_vel")
        rospy.loginfo("Publishing to: /robot_0/odom")

        # Spin
        rospy.spin()

    except KeyboardInterrupt:
        rospy.loginfo("Shutting down robot test node")
    except Exception as e:
        rospy.logerr(f"Error: {e}")
        import traceback
        traceback.print_exc()

