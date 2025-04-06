#!/home/khoint/thesis/deployment/src/.venv/bin/python3
import numpy as np
import rospy 
from sensor_msgs.msg import LaserScan


def lidar_callback(msg: LaserScan):
    scan = msg.ranges
    print(len(scan))
    crash_distance = 0.26
    is_crash = np.any(np.array(scan) <= crash_distance)
    print(f"Is crashed: {is_crash}")

if __name__=="__main__":
        

    rospy.loginfo("Init test node")

    rospy.init_node("Lidar listener")
    lidar_sub = rospy.Subscriber(f"/robot_0/scan", LaserScan, lidar_callback)
    rospy.spin()

