from typing import Any

import rospy
from std_msgs.msg import Bool, Float32

from custom_msgs.msg import RobotState
from model.utils import convert_statemsg2state
from settings import NUM_ROBOTS


class GatherNode:
    def __init__(self, current_id: int):
        self.subscriber_states = [None] * NUM_ROBOTS
        self.subscriber_next_states = [None] * NUM_ROBOTS
        self.subscriber_rewards = [None] * NUM_ROBOTS
        self.subscriber_terminal = [None] * NUM_ROBOTS

        self.current_id = current_id
        for i in range(NUM_ROBOTS):
            if i == current_id:
                continue
            rospy.Subscriber(f"/robot_{i}/state", RobotState, lambda msg: self.state_callback(msg, i))
            rospy.Subscriber(f"/robot_{i}/next_state", RobotState, lambda msg: self.next_state_callback(msg, i))
            rospy.Subscriber(f"/robot_{i}/reward", Float32, lambda msg: self.reward_callback(msg, i))
            rospy.Subscriber(f"/robot_{i}/terminal", Bool, lambda msg: self.terminal_callback(msg, i))

    def state_callback(self, state: RobotState, robot_id: int):
        self.subscriber_states[robot_id] = convert_statemsg2state(state)

    def next_state_callback(self, next_state: RobotState, robot_id: int):
        self.subscriber_next_states[robot_id] = convert_statemsg2state(next_state)

    def reward_callback(self, reward: Float32, robot_id: int):
        self.subscriber_rewards[robot_id] =  reward

    def terminal_callback(self, terminal: Bool, robot_id: int):
        self.subscriber_terminal[robot_id] =  terminal

    def set_cur_robot_state(self, state: list):
        self.subscriber_states[self.current_id] = state
    
    def set_cur_robot_next_state(self, next_state: list):
        self.subscriber_next_states[self.current_id] = next_state

    def set_cur_robot_reward(self, reward: float):
        self.subscriber_rewards[self.current_id] = reward

    def set_cur_robot_terminal(self, terminal: bool):
        self.subscriber_terminal[self.current_id] = terminal
    
    def __getattribute__(self, name: str) -> Any:
        retries = 0
        while retries < 10:
            attr_data = super().__getattribute__(name)
            if type(attr_data) == list and None in attr_data:
                rospy.sleep(0.1)
                retries += 1
                continue
            break
        return attr_data

    def __setattr__(self, name, value):
        super().__setattr__(name, value)