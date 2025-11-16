#!/usr/bin/env python3
"""
SAFE VERSION of run_model.py
Adds critical safety checks and proper timing

Author: Fixed for safety by Claude
Date: 2025-11-16
"""
import sys
import time
from collections import deque
from pathlib import Path

import numpy as np
import rospy
import torch
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from model.net import ActorCriticNetwork
from robot_env import RobotEnv, RobotStatus
from settings import *


class RunModelSafe:
    def __init__(self, robot: RobotEnv):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.robot = robot
        self.robot_status = RobotStatus.IDLE
        self.__init_topic()
        self.__load_policy()

        self.terminal = True
        self.safety_stop = False

        # Safety parameters
        self.MIN_OBSTACLE_DISTANCE = 0.25  # meters
        self.MAX_LINEAR_VEL = 0.7  # m/s (MATCH TRAINING - crucial for model performance!)
        self.MAX_ANGULAR_VEL = 1.0  # rad/s (MATCH TRAINING)
        self.CONTROL_RATE = 10  # Hz (match training frequency)
        self.MAX_STEPS_PER_GOAL = 500  # Timeout after 50 seconds at 10Hz

    def __init_topic(self):
        index = self.robot.index
        rospy.Subscriber(f"/robot_{index}/move_base_simple/goal", PoseStamped, self.goal_callback)

        # Emergency stop subscriber
        rospy.Subscriber(f"/robot_{index}/emergency_stop", Bool, self.emergency_stop_callback)

    def __load_policy(self):
        self.policy = ActorCriticNetwork(frames=LASER_HIST, action_space=ACT_SIZE, hidden_size=128)
        self.policy.to(self.device)
        self.policy.eval()  # Set to evaluation mode

        rospy.loginfo("Loading the ActorCriticNetwork policy")
        if not os.path.exists(POLICY_PATH):
            rospy.logerr("No policy found")
            exit()

        file = POLICY_PATH / Path("cnn_modern_best_71pct.pth")
        if os.path.exists(file):
            rospy.loginfo("="*50)
            rospy.loginfo("Loading Trained Model (SAFE MODE)")
            rospy.loginfo("="*50)
            state_dict = torch.load(file, map_location=self.device)
            self.policy.load_state_dict(state_dict)
            rospy.loginfo(f"Model loaded: {file}")
        else:
            rospy.logerr(f"Policy file not found: {file}")
            exit()

    def emergency_stop_callback(self, msg):
        """Emergency stop handler"""
        if msg.data:
            rospy.logwarn("EMERGENCY STOP ACTIVATED!")
            self.safety_stop = True
            self.robot.stop()

    def _wait_for_sensors(self, timeout=5.0):
        """Wait for sensor data to be available"""
        rospy.loginfo("Waiting for sensor data...")
        start_time = rospy.Time.now()

        while (rospy.Time.now() - start_time).to_sec() < timeout:
            if (self.robot.state_GT is not None and
                self.robot.speed_GT is not None and
                len(self.robot.scan) > 0):
                rospy.loginfo("✅ All sensors ready!")
                return True
            rospy.sleep(0.1)

        rospy.logerr("❌ Sensor timeout! Check:")
        rospy.logerr(f"  - AMCL pose: {self.robot.state_GT is not None}")
        rospy.logerr(f"  - Speed: {self.robot.speed_GT is not None}")
        rospy.logerr(f"  - LiDAR: {len(self.robot.scan) > 0}")
        return False

    def _prepare_state_tensor(self, state):
        """Convert state to model input tensors"""
        obs_stack, goal, speed = state
        obs = torch.FloatTensor(np.array(obs_stack)).unsqueeze(0).to(self.device)
        goal_tensor = torch.FloatTensor(goal).unsqueeze(0).to(self.device)
        speed_tensor = torch.FloatTensor(speed).unsqueeze(0).to(self.device)
        return {'obs': obs, 'goal': goal_tensor, 'speed': speed_tensor}

    def _scale_action(self, action):
        """Scale and limit action to safe bounds

        SAFETY: Apply stricter limits than training
        """
        # Scale from network output [0,1] and [-1,1]
        v = action[0] * ACTION_BOUND[1][0]  # [0, 0.7]
        w = action[1] * ACTION_BOUND[1][1]  # [-1, 1]

        # Apply safety limits (more conservative than training)
        v = np.clip(v, 0.0, self.MAX_LINEAR_VEL)
        w = np.clip(w, -self.MAX_ANGULAR_VEL, self.MAX_ANGULAR_VEL)

        return np.array([v, w])

    def _safety_check(self, action):
        """Check if action is safe to execute

        Returns: (is_safe, modified_action)
        """
        laser_scan = self.robot.get_laser_observation()

        # Convert normalized scan back to real distances
        scan_real = (laser_scan + 0.5) * 6.0
        min_dist = np.min(scan_real[np.isfinite(scan_real)])

        v, w = action

        # CRITICAL: Stop if obstacle too close
        if min_dist < self.MIN_OBSTACLE_DISTANCE:
            rospy.logwarn(f"⚠️  OBSTACLE TOO CLOSE: {min_dist:.2f}m - STOPPING!")
            return False, np.array([0.0, 0.0])

        # Slow down if obstacle nearby
        if min_dist < 0.5 and v > 0.2:
            v_safe = 0.2 * (min_dist - self.MIN_OBSTACLE_DISTANCE) / (0.5 - self.MIN_OBSTACLE_DISTANCE)
            v_safe = max(0.0, v_safe)
            rospy.logwarn(f"⚠️  SLOWING DOWN: obstacle at {min_dist:.2f}m, reducing v: {v:.2f} → {v_safe:.2f}")
            return True, np.array([v_safe, w])

        return True, action

    def run(self):
        """Main control loop with safety checks"""
        # Wait for sensors
        if not self._wait_for_sensors():
            rospy.logerr("Cannot start - sensors not ready!")
            return

        self.robot_status = RobotStatus.RUNNING
        self.terminal = False
        self.safety_stop = False

        ep_reward = 0
        step = 0

        # Initialize state
        obs = self.robot.get_laser_observation()
        obs_stack = deque([obs, obs, obs])
        goal = np.asarray(self.robot.get_local_goal())

        # SAFETY: Use speed_GT (from AMCL) instead of speed (from odom)
        speed = np.asarray(self.robot.speed_GT) if self.robot.speed_GT is not None else np.array([0.0, 0.0])
        state = [obs_stack, goal, speed]

        rospy.loginfo("="*60)
        rospy.loginfo("START NAVIGATION (SAFE MODE)")
        rospy.loginfo(f"Goal: {self.robot.goal_point}")
        rospy.loginfo(f"Initial distance: {self.robot.distance:.2f}m")
        rospy.loginfo(f"Control rate: {self.CONTROL_RATE} Hz")
        rospy.loginfo(f"Max linear vel: {self.MAX_LINEAR_VEL} m/s")
        rospy.loginfo(f"Max angular vel: {self.MAX_ANGULAR_VEL} rad/s")
        rospy.loginfo("="*60)

        rate = rospy.Rate(self.CONTROL_RATE)

        while not self.terminal and not rospy.is_shutdown() and not self.safety_stop:
            loop_start = time.time()

            # SAFETY: Check step limit
            if step >= self.MAX_STEPS_PER_GOAL:
                rospy.logwarn("⏱️  TIMEOUT: Exceeded max steps")
                self.robot.stop()
                break

            # Model inference
            state_tensor = self._prepare_state_tensor(state)
            with torch.no_grad():
                v, action, logprob, mean = self.policy(
                    state_tensor['obs'],
                    state_tensor['goal'],
                    state_tensor['speed']
                )

            # Scale action
            real_action = self._scale_action(mean[0].cpu().numpy())

            # SAFETY CHECK
            is_safe, safe_action = self._safety_check(real_action)

            # Execute action
            if is_safe:
                self.robot.control_vel(safe_action)
                if step % 10 == 0:  # Log every 1 second
                    rospy.loginfo(f"Step {step}: v={safe_action[0]:.3f} m/s, w={safe_action[1]:.3f} rad/s, dist={self.robot.distance:.2f}m")
            else:
                self.robot.stop()
                rospy.logwarn("Action blocked by safety check")

            # Get reward and check termination
            r, self.terminal, result = self.robot.get_reward_and_terminate(step)
            ep_reward += r

            if self.terminal:
                rospy.loginfo(f"🏁 TERMINATED: {result}")
                break

            # Update state for next iteration
            state_next = self.robot.get_laser_observation()
            obs_stack.popleft()
            obs_stack.append(state_next)
            goal_next = np.asarray(self.robot.get_local_goal())
            speed_next = np.asarray(self.robot.speed_GT) if self.robot.speed_GT is not None else np.array([0.0, 0.0])
            state = [obs_stack, goal_next, speed_next]

            step += 1

            # Maintain control rate
            rate.sleep()

            loop_time = time.time() - loop_start
            if loop_time > 0.15:  # Warn if loop too slow
                rospy.logwarn(f"⚠️  Slow loop: {loop_time*1000:.1f}ms (target: {1000/self.CONTROL_RATE:.1f}ms)")

        # Stop robot
        self.robot.stop()
        self.robot_status = RobotStatus.IDLE

        rospy.loginfo("="*60)
        rospy.loginfo("NAVIGATION COMPLETE")
        rospy.loginfo(f"Result: {result}")
        rospy.loginfo(f"Total reward: {ep_reward:.2f}")
        rospy.loginfo(f"Steps: {step}")
        rospy.loginfo(f"Final distance to goal: {self.robot.distance:.2f}m")
        rospy.loginfo("="*60)

    def goal_callback(self, goal_pose: PoseStamped):
        """Handle new goal"""
        rospy.loginfo("="*60)
        rospy.loginfo("NEW GOAL RECEIVED")
        rospy.loginfo("="*60)

        # Stop current navigation
        self.terminal = True
        rospy.sleep(0.5)  # Wait for current run to finish

        # Check if robot is ready
        retries = 0
        while retries < 10:
            if self.robot_status != RobotStatus.IDLE:
                rospy.sleep(0.5)
                retries += 1
                continue

            # Set new goal
            x = goal_pose.pose.position.x
            y = goal_pose.pose.position.y
            self.robot.set_new_goal([x, y])
            rospy.loginfo(f"Goal set: ({x:.2f}, {y:.2f})")

            # Start navigation
            rospy.sleep(0.1)
            self.run()
            return

        rospy.logerr("Robot is not ready, please wait and try again")


def get_index():
    try:
        register_service = rospy.ServiceProxy("/register_robot", Trigger)
        response = register_service()
        if response.success:
            return int(response.message)
        else:
            rospy.logerr("Cannot get index from robot_counter_service")
            sys.exit()
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        sys.exit()


if __name__ == "__main__":
    try:
        rospy.wait_for_service("/register_robot", timeout=5.0)
        rospy.wait_for_service("/get_robot_counter", timeout=5.0)

        rospy.loginfo("Initializing model node...")
        index = get_index()

        rospy.init_node(f'robot_{index}_model_safe')

        robot_env = RobotEnv(index)
        model = RunModelSafe(robot_env)

        rospy.loginfo("="*60)
        rospy.loginfo(f"robot_{index}_model_safe node READY")
        rospy.loginfo("Waiting for goal on /robot_0/move_base_simple/goal")
        rospy.loginfo("Emergency stop: rostopic pub /robot_0/emergency_stop std_msgs/Bool true")
        rospy.loginfo("="*60)

        rospy.spin()

    except KeyboardInterrupt:
        rospy.loginfo("Shutting down safely...")
        # Stop robot before exit
        try:
            robot_env.stop()
        except:
            pass
    except Exception as e:
        rospy.logerr(f"Fatal error: {e}")
        import traceback
        traceback.print_exc()
