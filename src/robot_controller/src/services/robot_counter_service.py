#!/home/khoint/thesis/deployment/src/.venv/bin/python3

import rospy
from std_srvs.srv import Trigger, TriggerResponse


class RobotCounterService:
    def __init__(self):
        self.robot_count = 0
        rospy.init_node('robot_counter_service')

        try:
            rospy.wait_for_service('/register_robot', timeout=1)
        except rospy.ROSException:
            rospy.Service('register_robot', Trigger, self.handle_request)
        try:
            rospy.wait_for_service('/get_robot_counter', timeout=1)
        except rospy.ROSException:
            rospy.Service('get_robot_counter', Trigger, self.handle_get_counter)
        rospy.loginfo("robot_counter_service is ready")
    
    def handle_request(self, request):
        current_index = self.robot_count
        self.robot_count += 1
        return TriggerResponse(success=True, message=str(current_index))
    
    def handle_get_counter(self, request):
        return TriggerResponse(success=True, message=str(self.robot_count))
    
if __name__ == "__main__":
    rospy.init_node('robot_counter_service')
    RobotCounterService()
    rospy.spin()
