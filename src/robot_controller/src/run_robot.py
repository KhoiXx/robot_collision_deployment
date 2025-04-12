#!/home/khoint/thesis/deployment/src/.venv/bin/python3
from robot_control import RobotControl
import rospy
import sys
from std_srvs.srv import Trigger 

def get_index():
    try:
        counter_service = rospy.ServiceProxy("/get_robot_counter", Trigger)
        response = counter_service()
        if response.success:
            index = int(response.message)
            return index - 1 if index else index
        else:
            rospy.logerr("Cannot get the index from 'get_robot_counter'")
            sys.exit()
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == "__main__":
    try:
        rospy.wait_for_service('/get_robot_counter')
        index = get_index()
        rospy.init_node(f"robot_{index}_execute")

        robot_control_node = RobotControl(index, "/dev/esp", baudrate=115200)
        rospy.loginfo(f"robot_{index}_execute node is ready")
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo(f"Shutting node 'robot_{index}_execute")
        rospy.on_shutdown()