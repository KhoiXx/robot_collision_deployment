#!/usr/bin/env python3
"""
SAFE VERSION of run_model.py
Adds critical safety checks and proper timing

Author: Fixed for safety by Claude
Date: 2025-11-16
"""
from numpy._typing._generic_alias import NDArray
from numpy._typing._generic_alias import NDArray
from numpy import floating
import sys
import time
from collections import deque
from pathlib import Path
from typing import Any

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
        self.MIN_OBSTACLE_DISTANCE = 0.18  # meters
        self.MAX_LINEAR_VEL = 0.3  # m/s - DEPLOYMENT LIMIT (training was 0.7, reduced for safety)
        self.MAX_ANGULAR_VEL = 1.0  # rad/s (MATCH TRAINING)
        self.CONTROL_RATE = 20  # Hz (match training frequency)
        self.MAX_STEPS_PER_GOAL = 500  # Timeout - increased because robot is slower (80s at 10Hz)

        # Stuck detection and recovery
        self.STUCK_TIME_THRESHOLD = 2.0  # seconds - if no progress for this long, consider stuck
        self.STUCK_DISTANCE_THRESHOLD = 0.05  # meters - minimum distance to consider as progress
        self.REVERSE_SPEED = -0.08  # m/s - reverse speed for recovery
        self.REVERSE_DURATION = 2  # seconds - how long to reverse
        self.MAX_STUCK_RETRIES = 3  # Maximum stuck recovery attempts before giving up

        # Stuck detection state
        self.stuck_start_time = None
        self.stuck_position = None
        self.stuck_count = 0

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

        file = POLICY_PATH / Path("cnn_modern_latest.pth")
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
                rospy.loginfo("All sensors ready!")
                return True
            rospy.sleep(0.1)

        rospy.logerr("Sensor timeout! Check:")
        if self.robot.use_pure_ukf:
            rospy.logerr(f"  - Pose (Pure UKF): {self.robot.state_GT is not None}")
        else:
            rospy.logerr(f"  - Pose (AMCL/Cartographer): {self.robot.state_GT is not None}")
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
        """Check if action is safe to execute - FRONT 120° ONLY

        Returns: (is_safe, modified_action, should_reverse)
        """
        laser_scan = self.robot.get_laser_observation()

        # Convert normalized scan back to real distances
        scan_real = (laser_scan + 0.5) * 6.0

        # Check ONLY front 120° (±60° from center)
        # Laser has 454 beams covering 360°, so front 120° ≈ 151 beams
        n = len(scan_real)
        front_120_start = n // 2 - n // 6  # -60° (1/6 of 360° = 60°)
        front_120_end = n // 2 + n // 6    # +60°
        front_scan = scan_real[front_120_start:front_120_end]

        # Get minimum distance in front 120° only
        front_scan_valid = front_scan[np.isfinite(front_scan)]
        if len(front_scan_valid) == 0:
            return True, action, False  # No valid readings, assume safe

        min_front_dist = np.min(front_scan_valid)

        v, w = action

        # CRITICAL: If obstacle too close IN FRONT (only when moving forward)
        if v > 0 and min_front_dist < self.MIN_OBSTACLE_DISTANCE:
            rospy.logwarn(f"FRONT OBSTACLE TOO CLOSE: {min_front_dist:.2f}m - SHOULD REVERSE!")
            return False, np.array([0.0, 0.0]), True  # Signal to reverse

        # No slow down - let model handle it
        return True, action, False

    def _check_stuck(self, commanded_velocity):
        """Detect if robot is stuck (not making progress)

        Args:
            commanded_velocity: The velocity command sent to robot [v, w]

        Returns: True if stuck, False otherwise
        """
        if self.robot.state_GT is None:
            return False

        # Only check stuck if robot is commanded to move forward
        v_cmd = commanded_velocity[0]
        if v_cmd <= 0.05:  # Robot commanded to stop or move very slowly
            # Reset stuck detection - robot is intentionally not moving
            self.stuck_position = None
            self.stuck_start_time = None
            return False

        current_position = np.array(self.robot.state_GT[:2])  # [x, y]
        current_time = rospy.Time.now().to_sec()

        # Initialize stuck detection on first call
        if self.stuck_position is None:
            self.stuck_position = current_position
            self.stuck_start_time = current_time
            return False

        # Calculate distance moved since stuck detection started
        distance_moved = np.linalg.norm(current_position - self.stuck_position)
        time_elapsed = current_time - self.stuck_start_time

        # Check if robot made progress
        if distance_moved > self.STUCK_DISTANCE_THRESHOLD:
            # Robot is making progress - reset stuck detection
            self.stuck_position = current_position
            self.stuck_start_time = current_time
            return False

        # Check if stuck for too long
        if time_elapsed > self.STUCK_TIME_THRESHOLD:
            rospy.logwarn(f"STUCK DETECTED: Moved only {distance_moved:.3f}m in {time_elapsed:.1f}s with v_cmd={v_cmd:.2f}")
            return True

        return False

    def _reverse_safely(self):
        """Reverse robot safely to recover from stuck situation

        Returns: True if recovery successful, False otherwise
        """
        rospy.logwarn(f"RECOVERY ATTEMPT {self.stuck_count + 1}/{self.MAX_STUCK_RETRIES}: Reversing...")

        # Check if there's space behind
        laser_scan = self.robot.get_laser_observation()
        scan_real = (laser_scan + 0.5) * 6.0

        # Check back 120° (±60° from rear)
        n = len(scan_real)
        back_120_start = n // 6  # Start at +60° (since 0° is rear in this setup)
        back_120_end = n - n // 6  # End at -60°

        # Actually, laser 0° is front, so back is around n/2
        # Back 120° would be around indices [n/2-n/6 : n/2+n/6] from the opposite side
        # Let's take the rear third of the scan
        back_distances = np.concatenate([scan_real[:n//6], scan_real[-n//6:]])
        back_distances_valid = back_distances[np.isfinite(back_distances)]

        if len(back_distances_valid) > 0:
            min_back_dist = np.min(back_distances_valid)
            if min_back_dist < self.MIN_OBSTACLE_DISTANCE:
                rospy.logerr("Cannot reverse - obstacle behind!")
                return False

        # Reverse for specified duration
        reverse_action = np.array([self.REVERSE_SPEED, 0.0])
        reverse_steps = int(self.REVERSE_DURATION * self.CONTROL_RATE)

        rospy.loginfo(f"Reversing at {self.REVERSE_SPEED:.2f} m/s for {self.REVERSE_DURATION}s...")
        rate = rospy.Rate(self.CONTROL_RATE)

        for i in range(reverse_steps):
            self.robot.control_vel(reverse_action)
            rate.sleep()

            # Safety check while reversing
            if self.safety_stop or rospy.is_shutdown():
                self.robot.stop()
                return False

        # Stop after reversing
        self.robot.stop()
        rospy.sleep(0.5)  # Brief pause

        # Reset stuck detection
        if self.robot.state_GT is not None:
            self.stuck_position = np.array(self.robot.state_GT[:2])
            self.stuck_start_time = rospy.Time.now().to_sec()

        rospy.loginfo("Recovery complete - resuming navigation")
        return True

    def run(self):
        """Main control loop with safety checks"""
        # Wait for sensors
        if not self._wait_for_sensors():
            rospy.logerr("Cannot start - sensors not ready!")
            return

        self.robot_status = RobotStatus.RUNNING
        self.terminal = False
        self.safety_stop = False

        # Reset stuck detection for new goal
        self.stuck_position = None
        self.stuck_start_time = None
        self.stuck_count = 0

        ep_reward = 0
        step = 0

        # Initialize state
        obs = self.robot.get_laser_observation()
        obs_stack = deque([obs, obs, obs])
        if obs_stack is None:
            obs_stack = deque([obs] * LASER_HIST)
        else:
            _ = obs_stack.popleft()
            obs_stack.append(obs)
        goal = np.asarray(self.robot.get_local_goal())

        # Speed: Use speed_GT (from AMCL/Cartographer or UKF depending on mode)
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

        # Track actual control rate
        rate_tracker = deque(maxlen=100)
        last_step_time = time.time()

        while not self.terminal and not rospy.is_shutdown() and not self.safety_stop:
            loop_start = time.time()

            # SAFETY: Check step limit
            if step >= self.MAX_STEPS_PER_GOAL:
                rospy.logwarn("TIMEOUT: Exceeded max steps")
                self.robot.stop()
                break

            # Model inference
            state_tensor = self._prepare_state_tensor(state)
            with torch.no_grad():
                # Return order: action, v, logprob, mean (matches training)
                _, _, _, mean = self.policy(
                    state_tensor['obs'],
                    state_tensor['goal'],
                    state_tensor['speed']
                )

            # Scale action
            real_action = self._scale_action(mean[0].cpu().numpy())

            # SAFETY CHECK
            is_safe, safe_action, should_reverse = self._safety_check(real_action)

            # Execute action or reverse if needed
            if is_safe:
                self.robot.control_vel(safe_action)
                if step % 10 == 0:  # Log every 1 second
                    rospy.loginfo(f"Step {step}: v={safe_action[0]:.3f} m/s, w={safe_action[1]:.3f} rad/s, dist={self.robot.distance:.2f}m, state: {self.robot.state_GT}")
            elif should_reverse:
                # Obstacle too close - try to reverse
                rospy.logwarn("Obstacle too close - attempting safe reverse")
                if self._reverse_safely():
                    rospy.loginfo("Reverse successful - resuming")
                else:
                    rospy.logerr("Cannot reverse safely - stopping")
                    self.robot.stop()
            else:
                self.robot.stop()
                rospy.logwarn("Action blocked by safety check")

            # STUCK DETECTION (only check if robot is commanded to move)
            if self._check_stuck(safe_action):
                if self.stuck_count >= self.MAX_STUCK_RETRIES:
                    rospy.logerr(f"STUCK: Exceeded max recovery attempts ({self.MAX_STUCK_RETRIES})")
                    self.robot.stop()
                    self.terminal = True
                    result = 'Stuck - Recovery Failed'
                    break

                # Attempt recovery
                if self._reverse_safely():
                    self.stuck_count += 1
                    # Continue with current goal after recovery
                else:
                    rospy.logerr("Recovery failed - terminating")
                    self.robot.stop()
                    self.terminal = True
                    result = 'Stuck - Cannot Recover'
                    break

            # Get reward and check termination
            r, self.terminal, result = self.robot.get_reward_and_terminate(step)
            ep_reward += r

            if self.terminal:
                rospy.loginfo(f"TERMINATED: {result}")
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

            # Track actual loop timing
            loop_time = time.time() - loop_start
            step_interval = loop_time
            rate_tracker.append(step_interval)

            # Log timing every 50 steps (~5s at 10Hz)
            if step % 50 == 0 and len(rate_tracker) > 10:
                avg_interval = np.mean(rate_tracker)
                actual_rate = 1.0 / avg_interval if avg_interval > 0 else 0
                min_interval = np.min(rate_tracker)
                max_interval = np.max(rate_tracker)
                rospy.loginfo(f"[RATE] Actual: {actual_rate:.1f} Hz (target: {self.CONTROL_RATE} Hz) | "
                             f"Interval: {avg_interval*1000:.1f}ms (min: {min_interval*1000:.0f}ms, max: {max_interval*1000:.0f}ms)")

            if loop_time > 0.15:  # Warn if loop too slow
                rospy.logwarn(f"Slow loop: {loop_time*1000:.1f}ms (target: {1000/self.CONTROL_RATE:.1f}ms)")

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
        """Handle new goal - NON-BLOCKING version"""
        x = goal_pose.pose.position.x
        y = goal_pose.pose.position.y

        rospy.loginfo("="*60)
        rospy.loginfo(f"NEW GOAL RECEIVED: ({x:.2f}, {y:.2f})")
        rospy.loginfo("="*60)

        # Set new goal
        self.robot.set_new_goal([x, y])

        # Stop current navigation if running
        if self.robot_status == RobotStatus.RUNNING:
            rospy.loginfo("Stopping current navigation...")
            self.terminal = True

        # Start new navigation in thread to avoid blocking
        import threading
        def start_navigation():
            # Wait for current run() to finish if needed
            while self.robot_status == RobotStatus.RUNNING:
                rospy.sleep(0.1)

            # Now start new navigation
            rospy.loginfo(f"Starting navigation to ({x:.2f}, {y:.2f})")
            self.run()

        nav_thread = threading.Thread(target=start_navigation)
        nav_thread.daemon = True
        nav_thread.start()


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
        index = 0

        rospy.init_node(f'robot_{index}_model_safe')

        # Check localization mode
        use_pure_ukf = rospy.get_param('~use_pure_ukf', False)
        use_cartographer = rospy.get_param('~use_cartographer', False)

        if use_pure_ukf:
            rospy.loginfo("="*60)
            rospy.loginfo("PURE UKF MODE: 100% UKF odometry (no AMCL/Cartographer)")
            rospy.loginfo("="*60)
        elif use_cartographer:
            rospy.loginfo("="*60)
            rospy.loginfo("CARTOGRAPHER MODE: Cartographer for ground truth pose")
            rospy.loginfo("="*60)
        else:
            rospy.loginfo("="*60)
            rospy.loginfo("AMCL MODE: AMCL for ground truth pose")
            rospy.loginfo("="*60)

        robot_env = RobotEnv(index, use_pure_ukf=use_pure_ukf, use_cartographer=use_cartographer)
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
