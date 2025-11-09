#!/usr/bin/env python3
import sys
import time
from collections import deque
from pathlib import Path

import message_filters
import numpy as np
import rospy
import torch
import torch.nn as nn
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float32
from std_srvs.srv import Trigger
from torch.optim import Adam

# RobotState msg:
# float32[] obs_prev
# float32[] obs_cur
# float32[] obs_next
# float32[] goal
# float32[] speed
from custom_msgs.msg import RobotState
from gather_node import GatherNode
from model.net import ActorCriticNetwork  # FIXED: Use ActorCriticNetwork instead of CNNPolicy
from model.ppo import (generate_action, generate_train_data, ppo_update,
                       transform_buffer)
from robot_env import RobotEnv, RobotStatus
from settings import *


class RunModel:
    def __init__(self, robot: RobotEnv):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.robot = robot
        self.gather_node = GatherNode(robot.index)
        self.robot_status = RobotStatus.IDLE
        self.__init_topic()
        self.__load_policy()

        self.terminal = True

    def __init_topic(self):
        index = self.robot.index
        rospy.Subscriber(f"/robot_{index}/move_base_simple/goal", PoseStamped, self.goal_callback)
        # publisher
        self.state_publish = rospy.Publisher(f"/robot_{index}/state", RobotState, queue_size=10)
        self.next_state_publish = rospy.Publisher(f"/robot_{index}/next_state", RobotState, queue_size=10)
        self.reward_publish = rospy.Publisher(f"/robot_{index}/reward", Float32, queue_size=10)
        self.terminal_publish = rospy.Publisher(f"/robot_{index}/terminal", Bool, queue_size=10)

    def __load_policy(self):
        # FIXED: Use ActorCriticNetwork matching training architecture
        self.policy = ActorCriticNetwork(frames=LASER_HIST, action_space=ACT_SIZE, hidden_size=128)
        self.policy.to(self.device)

        # Use separate optimizers like in training for better performance
        self.optimizers = [
            Adam(self.policy.enc_critic_params(), lr=5e-4),  # Critic optimizer
            Adam(self.policy.actor_params(), lr=1e-4)        # Actor optimizer
        ]
        # Keep single optimizer for compatibility
        self.optimizer = Adam(self.policy.parameters(), lr=LEARNING_RATE)

        rospy.loginfo("Loading the ActorCriticNetwork policy")
        if not os.path.exists(POLICY_PATH):
            rospy.logerr("No policy found")
            exit()

        # IMPORTANT: Update this to your actual trained model file!
        # Copy your trained model: policy/stage2/cnn_gru_attention_5700.pth -> deployment/policy/
        file = POLICY_PATH / Path("cnn_gru_attention_5700.pth")
        if os.path.exists(file):
            print("####################################")
            print("####### Loading Trained Model ######")
            print("####################################")
            state_dict = torch.load(file, map_location=self.device)
            self.policy.load_state_dict(state_dict)
            rospy.loginfo(f"Successfully loaded model from {file}")
        else:
            rospy.logerr(f"Policy file not found: {file}")
            rospy.logerr("Please copy your trained model to the policy directory!")
            exit()

    def run(self):
        buff = []
        global_update = 0
        self.robot_status = RobotStatus.RUNNING

        self.terminal = False
        ep_reward = 0
        step = 1
        obs = self.robot.get_laser_observation()
        obs_stack = deque([obs, obs, obs])
        goal = np.asarray(self.robot.get_local_goal())
        speed = np.asarray(self.robot.speed)
        state = [obs_stack, goal, speed]

        self.publish_robot_state(state, self.state_publish)
        self.gather_node.set_cur_robot_state(state)
        rospy.sleep(0.002)

        rospy.loginfo("########### Start running ###########")
        while not self.terminal and not rospy.is_shutdown():
            start_time = time.time()
            state_list = self.gather_node.subscriber_states
            if None in state_list:
                rospy.logerr("State were not correctly gathered")
            print(f"Length state list: {len(state_list)}")
            v, a, logprob, scaled_action = generate_action(
                state_list=state_list, policy=self.policy, action_bound=ACTION_BOUND
            )
            real_action = scaled_action[self.robot.index]
            self.robot.control_vel(real_action)
            # FIXME: not align with real world
            rospy.sleep(0.005)
            r, self.terminal, result = self.robot.get_reward_and_terminate(step)
            print(f"Reward: {r}, step: {step}")

            self.reward_publish.publish(r)
            self.gather_node.set_cur_robot_reward(r)
            self.terminal_publish.publish(self.terminal)
            self.gather_node.set_cur_robot_terminal(self.terminal)
            rospy.sleep(0.002)

            ep_reward += r

            state_next = self.robot.get_laser_observation()
            left = obs_stack.popleft()
            obs_stack.append(state_next)
            goal_next = np.asarray(self.robot.get_local_goal())
            speed_next = np.asarray(self.robot.speed)
            state_next = [obs_stack, goal_next, speed_next]

            if step % NUM_STEPS == 0:
                self.publish_robot_state(state_next, self.next_state_publish)
                self.gather_node.set_cur_robot_next_state(state_next)
                rospy.sleep(0.002)

                next_state_list = self.gather_node.subscriber_next_states
                last_v, _, _, _ = generate_action(
                    state_list=next_state_list, policy=self.policy, action_bound=ACTION_BOUND
                )
            r_list = self.gather_node.subscriber_rewards
            terminal_list = self.gather_node.subscriber_terminal

            # Update policy
            buff.append((state_list, a, r_list, terminal_list, logprob, v))
            if len(buff) > NUM_STEPS - 1:
                rospy.loginfo("Transform buff")
                s_batch, goal_batch, speed_batch, a_batch, r_batch, d_batch, l_batch, v_batch = transform_buffer(
                    buff=buff
                )
                t_batch, advs_batch = generate_train_data(
                    rewards=r_batch, gamma=GAMMA, values=v_batch, last_value=last_v, dones=d_batch, lam=LAMBDA
                )
                memory = (s_batch, goal_batch, speed_batch, a_batch, l_batch, t_batch, v_batch, r_batch, advs_batch)
                ppo_update(
                    policy=self.policy,
                    optimizer=self.optimizer,
                    batch_size=BATCH_SIZE,
                    memory=memory,
                    epoch=EPOCH,
                    coeff_entropy=COEFF_ENTROPY,
                    clip_value=CLIP_VALUE,
                    num_step=NUM_STEPS,
                    num_env=NUM_ROBOTS,
                    frames=LASER_HIST,
                    obs_size=NUM_OBS,
                    act_size=ACT_SIZE,
                )
                buff = []
                global_update += 1
            step += 1
            # logger_env.info(f"{id+1},{r:.5f},{env.move_distance:.5f},{real_action[0]:.4f},{real_action[1]:.4f},{self.robot.distance},{terminal},{result},{(time.time() - start_time):.5f}")
        self.robot_status = RobotStatus.IDLE

        if global_update != 0 and global_update % 20 == 0:
            torch.save(self.policy.state_dict(), "/policy/stage1_fix.pth")

    def goal_callback(self, goal_pose: PoseStamped):
        rospy.loginfo("########### Goal callback ###########")
        self.terminal = True

        retries = 0
        while retries < 10:
            if self.robot_status != RobotStatus.IDLE:
                rospy.sleep(0.5)
                continue
            x = goal_pose.pose.position.x
            y = goal_pose.pose.position.y
            self.robot.set_new_goal([x, y])
            rospy.loginfo(f"Setting new goal: x: {x}, y: {y}")
            rospy.sleep(0.01)
            self.run()
        rospy.logerr("Robot is not ready, please wait and set new goal")

    def publish_robot_state(self, state: list, topic: rospy.Publisher):
        msg = RobotState()
        obs: deque = state[0]
        msg.obs_prev = obs[0].tolist()
        msg.obs_cur = obs[1].tolist()
        msg.obs_next = obs[2].tolist()
        msg.goal = state[1].tolist()
        msg.speed = state[2].tolist()
        topic.publish(msg)


def get_index():
    try:
        resgister_service = rospy.ServiceProxy("/register_robot", Trigger)
        response = resgister_service()
        if response.success:
            return int(response.message)
        else:
            rospy.logerr("Cannot get the index from 'robot_counter_service'")
            sys.exit()
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")


if __name__ == "__main__":
    try:
        rospy.wait_for_service("/register_robot")
        rospy.wait_for_service("/get_robot_counter")

        rospy.loginfo("Init node model")
        index = get_index()

        rospy.init_node(f'robot_{index}_model')

        robot_env = RobotEnv(index)
        model = RunModel(robot_env)
        rospy.loginfo(f"robot_{index}_model node is ready")
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo(f"Shutting down node 'robot_{index}_model")
        rospy.on_shutdown()
