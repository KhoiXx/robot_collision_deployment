#!/usr/bin/env python3
"""
Server-side inference node with online learning for Stage 2 PPO model.

This node runs on the SERVER (not robot), handles:
1. Subscribes to robot sensor data (/scan, /odom, /goal) via ROS network
2. Runs model inference on server GPU/CPU
3. Publishes velocity commands to robot (/cmd_vel)
4. Collects experiences for online learning
5. Periodically updates model weights (PPO)

Author: Claude Code
Date: 2025-11-02
"""

import os
import sys
import rospy
import torch
import numpy as np
from collections import deque
from scipy.interpolate import interp1d
from typing import Optional, Tuple

# ROS messages
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Bool

# Add model path
sys.path.append(os.path.dirname(__file__))
from model.net_stage2 import ActorCriticNetwork
from model.ppo_modern import ppo_update_stage2_modern
from model.replay_buffer import ExperienceBuffer
from torch.optim import Adam


class ServerInferenceNode:
    """
    Server-side inference node with online learning
    Runs Stage 2 PPO model remotely and updates via collected experiences
    """

    def __init__(self):
        rospy.init_node('server_inference_node', anonymous=False)

        # ================== Configuration ==================
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.laser_hist = 3          # Frame stacking
        self.num_obs = 454            # Model expects 454 beams
        self.raw_laser_beams = 226    # Real LiDAR has 226 beams
        self.action_bounds = [[0, -2], [1.1, 2]]  # [lin_min, ang_min], [lin_max, ang_max]

        # Online learning hyperparameters (match training)
        self.gamma = 0.99
        self.lam = 0.95
        self.batch_size = 768
        self.epoch = 5
        self.coeff_entropy = 7e-4
        self.clip_value = 0.1
        self.update_frequency = 128   # Update model every N steps
        self.save_frequency = 20      # Save model every N updates

        # ================== Model Setup ==================
        rospy.loginfo("="*70)
        rospy.loginfo("🚀 Server Inference Node - Stage 2 PPO")
        rospy.loginfo("="*70)

        self.policy = ActorCriticNetwork(frames=self.laser_hist, action_space=2, hidden_size=128)
        self.policy.to(self.device)
        self.load_model()

        # Separate optimizers for online learning (match training config)
        self.critic_opt = Adam(self.policy.enc_critic_params(), lr=5e-4)
        self.actor_opt = Adam(self.policy.actor_params(), lr=1.5e-4)
        self.optimizers = [self.critic_opt, self.actor_opt]

        rospy.loginfo(f"✅ Device: {self.device}")
        rospy.loginfo(f"✅ Model: ActorCriticNetwork (Stage 2)")
        rospy.loginfo(f"✅ Optimizers: Critic LR=5e-4, Actor LR=1.5e-4")

        # ================== Experience Buffer ==================
        self.buffer = ExperienceBuffer(
            capacity=self.update_frequency,  # Collect exactly enough for one update
            num_envs=1,
            obs_dim=self.num_obs,
            gamma=self.gamma,
            lam=self.lam
        )
        rospy.loginfo(f"✅ Experience buffer: capacity={self.update_frequency}")

        # ================== State Tracking ==================
        self.scan_raw = np.zeros(self.raw_laser_beams)
        self.obs_stack = deque(maxlen=3)
        self.position = [0.0, 0.0, 0.0]  # [x, y, theta]
        self.velocity = [0.0, 0.0]        # [linear, angular]
        self.goal = [5.0, 0.0]            # Default goal
        self.goal_received = False

        # Experience tracking
        self.last_state = None
        self.last_action = None
        self.last_logprob = None
        self.last_value = None

        # Episode tracking
        self.step_count = 0
        self.global_update = 0
        self.episode_count = 0
        self.episode_reward = 0.0
        self.episode_steps = 0

        # Performance tracking
        self.success_count = 0
        self.collision_count = 0
        self.timeout_count = 0

        # ================== ROS Topics ==================
        # Subscribers (from robot)
        rospy.Subscriber('/robot_0/scan', LaserScan, self.scan_callback, queue_size=1)
        rospy.Subscriber('/robot_0/odom', Odometry, self.odom_callback, queue_size=1)
        rospy.Subscriber('/robot_0/move_base_simple/goal', PoseStamped, self.goal_callback)

        # Publishers (to robot)
        self.cmd_vel_pub = rospy.Publisher('/robot_0/cmd_vel', Twist, queue_size=10)

        # Monitoring publishers
        self.reward_pub = rospy.Publisher('/server/reward', Float32, queue_size=10)
        self.terminal_pub = rospy.Publisher('/server/terminal', Bool, queue_size=10)

        # ================== Control Loop Timer ==================
        # Run at 50Hz (20ms) - balanced between responsiveness and computation
        self.control_rate = 50  # Hz
        self.control_timer = rospy.Timer(
            rospy.Duration(1.0 / self.control_rate),
            self.control_loop
        )

        # Create log directory
        self.log_dir = os.path.join(os.path.dirname(__file__), 'logs', 'server_inference')
        os.makedirs(self.log_dir, exist_ok=True)

        rospy.loginfo("="*70)
        rospy.loginfo("✅ Server Inference Node Ready!")
        rospy.loginfo(f"   Control rate: {self.control_rate} Hz")
        rospy.loginfo(f"   Online learning: Enabled")
        rospy.loginfo(f"   Update frequency: Every {self.update_frequency} steps")
        rospy.loginfo("="*70)
        rospy.loginfo("Waiting for sensor data and goal...")

    def load_model(self):
        """Load trained Stage 2 model from snapshot"""
        model_path = os.path.join(
            os.path.dirname(__file__),
            'policy/stage2/cnn_modern_100_best_70pct.pth'
        )

        if not os.path.exists(model_path):
            rospy.logerr(f"❌ Model not found: {model_path}")
            rospy.logerr("Please copy the trained model first!")
            rospy.logerr("Run: cp /path/to/cnn_modern_100_best_70pct.pth policy/stage2/")
            sys.exit(1)

        try:
            state_dict = torch.load(model_path, map_location=self.device)
            self.policy.load_state_dict(state_dict)
            self.policy.eval()  # Start in eval mode

            rospy.loginfo(f"✅ Model loaded: {model_path}")
            rospy.loginfo(f"   Parameters: {sum(p.numel() for p in self.policy.parameters()):,}")

        except Exception as e:
            rospy.logerr(f"❌ Failed to load model: {e}")
            sys.exit(1)

    # ==================== ROS Callbacks ====================

    def scan_callback(self, msg: LaserScan):
        """Process LiDAR scan and upsample from 226 → 454 beams"""
        # Convert to numpy array
        scan = np.array(msg.ranges)

        # Handle inf/nan values
        scan[np.isnan(scan)] = 10.0
        scan[np.isinf(scan)] = 10.0

        # Clip to reasonable range [0, 10] meters
        scan = np.clip(scan, 0.0, 10.0)

        # Store raw scan
        self.scan_raw = scan

    def odom_callback(self, msg: Odometry):
        """Process odometry data"""
        # Extract position
        self.position[0] = msg.pose.pose.position.x
        self.position[1] = msg.pose.pose.position.y

        # Extract orientation (quaternion to yaw)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.position[2] = np.arctan2(siny_cosp, cosy_cosp)

        # Extract velocity
        self.velocity[0] = msg.twist.twist.linear.x
        self.velocity[1] = msg.twist.twist.angular.z

    def goal_callback(self, msg: PoseStamped):
        """Set new navigation goal"""
        self.goal[0] = msg.pose.position.x
        self.goal[1] = msg.pose.position.y
        self.goal_received = True

        rospy.loginfo("="*70)
        rospy.loginfo(f"🎯 New goal set: ({self.goal[0]:.2f}, {self.goal[1]:.2f})")
        rospy.loginfo("="*70)

        # Reset episode tracking
        self.episode_reward = 0.0
        self.episode_steps = 0

    # ==================== Helper Functions ====================

    def upsample_laser_scan(self, scan_226: np.ndarray) -> np.ndarray:
        """
        Upsample 226 beams to 454 beams using linear interpolation
        This matches the training data format
        """
        if len(scan_226) != self.raw_laser_beams:
            rospy.logwarn(f"Expected {self.raw_laser_beams} beams, got {len(scan_226)}")
            # Pad or truncate
            if len(scan_226) < self.raw_laser_beams:
                scan_226 = np.pad(scan_226, (0, self.raw_laser_beams - len(scan_226)),
                                  mode='constant', constant_values=10.0)
            else:
                scan_226 = scan_226[:self.raw_laser_beams]

        x_old = np.linspace(0, 1, self.raw_laser_beams)
        x_new = np.linspace(0, 1, self.num_obs)
        interpolator = interp1d(x_old, scan_226, kind='linear')
        scan_454 = interpolator(x_new)

        return scan_454

    def get_local_goal(self) -> np.ndarray:
        """
        Transform global goal to robot's local frame
        Returns [distance, angle] to goal
        """
        # Global goal
        gx, gy = self.goal

        # Robot position
        rx, ry, theta = self.position

        # Vector to goal
        dx = gx - rx
        dy = gy - ry

        # Rotate to robot's frame
        local_x = dx * np.cos(-theta) - dy * np.sin(-theta)
        local_y = dx * np.sin(-theta) + dy * np.cos(-theta)

        # Distance and angle to goal
        distance = np.sqrt(local_x**2 + local_y**2)
        angle = np.arctan2(local_y, local_x)

        return np.array([distance, angle])

    def scale_action(self, action: np.ndarray) -> np.ndarray:
        """
        Scale action from [-1, 1] to physical bounds
        action[0]: linear velocity → [0, 1.1] m/s
        action[1]: angular velocity → [-2, 2] rad/s
        """
        linear = ((action[0] + 1) / 2 *
                  (self.action_bounds[1][0] - self.action_bounds[0][0]) +
                  self.action_bounds[0][0])
        angular = ((action[1] + 1) / 2 *
                   (self.action_bounds[1][1] - self.action_bounds[0][1]) +
                   self.action_bounds[0][1])
        return np.array([linear, angular])

    def calculate_reward(self, state: dict) -> Tuple[float, bool, str]:
        """
        Calculate reward (MUST match training reward function!)

        Returns:
            reward: Scalar reward
            done: Episode termination flag
            result: Termination reason string
        """
        goal = state['goal']
        distance = goal[0]

        # Check goal reached (0.5m threshold)
        if distance < 0.5:
            return 100.0, True, "Reach Goal"

        # Check collision (0.25m threshold from LiDAR)
        min_scan = np.min(self.scan_raw)
        if min_scan < 0.25:
            return -100.0, True, "Collision"

        # Check timeout (500 steps max)
        if self.episode_steps > 500:
            return -30.0, True, "Timeout"

        # Step reward components (match ppo_stage2_enhanced.py)
        # Progress reward
        progress_reward = 0.8 * (-distance / 10.0)  # Negative distance scaled

        # Time penalty
        time_penalty = -0.01

        # Rotation penalty
        action = state.get('action', np.array([0.0, 0.0]))
        rotation_penalty = -0.08 * abs(action[1])

        reward = progress_reward + time_penalty + rotation_penalty

        return reward, False, "Running"

    # ==================== Main Control Loop ====================

    def control_loop(self, event):
        """
        Main control loop - runs at configured rate (default 50Hz)
        Handles inference, experience collection, and online learning
        """
        # Skip if no scan data yet
        if len(self.scan_raw) == 0:
            return

        # Skip if no goal received yet
        if not self.goal_received:
            return

        # ============ Prepare Observation ============
        scan_upsampled = self.upsample_laser_scan(self.scan_raw)

        # Normalize observation (divide by max range, center around 0)
        # This matches training preprocessing
        obs = scan_upsampled / 6.0 - 0.5

        # Initialize observation stack if empty
        if len(self.obs_stack) == 0:
            self.obs_stack = deque([obs, obs, obs], maxlen=3)
        else:
            self.obs_stack.append(obs)

        # ============ Prepare State ============
        laser_stack = np.array(self.obs_stack)  # [3, 454]
        goal = self.get_local_goal()             # [2]: distance, angle
        speed = np.array(self.velocity)          # [2]: linear, angular

        # Convert to tensors
        laser_tensor = torch.FloatTensor(laser_stack).unsqueeze(0).to(self.device)
        goal_tensor = torch.FloatTensor(goal).unsqueeze(0).to(self.device)
        speed_tensor = torch.FloatTensor(speed).unsqueeze(0).to(self.device)

        # ============ Model Inference ============
        with torch.no_grad():
            action, value, logprob, mean = self.policy(
                laser_tensor, goal_tensor, speed_tensor
            )

        # Use mean for deployment (deterministic policy)
        action_np = mean[0].cpu().numpy()
        value_np = value[0].cpu().numpy().item()
        logprob_np = logprob[0].cpu().numpy().item()

        # Scale action to physical bounds
        scaled_action = self.scale_action(action_np)

        # ============ Publish Command ============
        cmd = Twist()
        cmd.linear.x = float(scaled_action[0])
        cmd.angular.z = float(scaled_action[1])
        self.cmd_vel_pub.publish(cmd)

        # ============ Experience Collection ============
        current_state = {
            'obs': laser_stack,
            'goal': goal,
            'speed': speed,
            'action': action_np
        }

        # Calculate reward and check termination
        reward, done, result = self.calculate_reward(current_state)
        self.episode_reward += reward
        self.episode_steps += 1

        # Publish monitoring data
        self.reward_pub.publish(Float32(reward))
        self.terminal_pub.publish(Bool(done))

        # Store experience if we have a previous state
        if self.last_state is not None:
            self.buffer.add(
                obs=self.last_state['obs'],
                goal=self.last_state['goal'],
                speed=self.last_state['speed'],
                action=self.last_action,
                reward=reward,
                done=done,
                logprob=self.last_logprob,
                value=self.last_value
            )

        # Update counters
        self.step_count += 1

        # ============ Online Learning Update ============
        if self.buffer.is_ready(self.update_frequency):
            self.update_policy(value_np)  # Pass current value for bootstrapping

        # ============ Episode Termination ============
        if done:
            self.handle_episode_end(result)

        # Update last state and action
        self.last_state = current_state
        self.last_action = action_np
        self.last_logprob = logprob_np
        self.last_value = value_np

    def update_policy(self, last_value: float):
        """
        Perform online learning update using PPO
        """
        rospy.loginfo("="*70)
        rospy.loginfo(f"🔄 Policy Update #{self.global_update + 1}")
        rospy.loginfo(f"   Buffer size: {len(self.buffer)}")

        # Get training batch with GAE
        batch, advantages, returns = self.buffer.get_training_batch(last_value)

        # Prepare memory tuple for PPO update (match Stage 2 format)
        obs_batch = batch['observations']    # [N, 3, 454]
        goal_batch = batch['goals']           # [N, 2]
        speed_batch = batch['speeds']         # [N, 2]
        action_batch = batch['actions']       # [N, 2]
        logprob_batch = batch['logprobs']     # [N]
        value_batch = batch['values']         # [N]
        reward_batch = batch['rewards']       # [N]

        # Reshape for PPO (expects [num_env * num_step, ...])
        N = len(obs_batch)
        memory = (
            obs_batch.reshape(N, self.laser_hist, self.num_obs),
            goal_batch.reshape(N, 2),
            speed_batch.reshape(N, 2),
            action_batch.reshape(N, 2),
            logprob_batch.reshape(N, 1),
            returns.reshape(N, 1),
            value_batch.reshape(N, 1),
            reward_batch.reshape(N, 1),
            advantages.reshape(N, 1)
        )

        # Switch to training mode
        self.policy.train()

        try:
            # Perform PPO update
            training_metrics = ppo_update_stage2_modern(
                policy=self.policy,
                optimizer=self.optimizers,
                batch_size=min(self.batch_size, N),  # Adjust if buffer smaller
                memory=memory,
                filter_index=[],  # No filtering for single robot
                epoch=self.epoch,
                coeff_entropy=self.coeff_entropy,
                clip_value=self.clip_value,
                num_step=N,
                num_env=1,  # Single robot
                frames=self.laser_hist,
                obs_size=self.num_obs,
                act_size=2,
                target_kl=0.025,
                value_loss_coeff=3.5,
                value_clip=True,
                use_huber_loss=False,
                max_grad_norm=0.5
            )

            # Log training metrics
            if training_metrics:
                rospy.loginfo(f"   Policy loss: {training_metrics['policy_loss']:.4f}")
                rospy.loginfo(f"   Value loss: {training_metrics['value_loss']:.4f}")
                rospy.loginfo(f"   Entropy: {training_metrics['entropy']:.4f}")
                rospy.loginfo(f"   KL divergence: {training_metrics['kl_divergence']:.4f}")
                rospy.loginfo(f"   Explained variance: {training_metrics['explained_variance']:.3f}")

        except Exception as e:
            rospy.logerr(f"❌ PPO update failed: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())

        # Switch back to eval mode
        self.policy.eval()

        # Clear buffer
        self.buffer.reset()

        # Save model periodically
        self.global_update += 1
        if self.global_update % self.save_frequency == 0:
            self.save_model()

        rospy.loginfo("✅ Policy update complete!")
        rospy.loginfo("="*70)

    def save_model(self):
        """Save current model weights"""
        save_dir = os.path.join(os.path.dirname(__file__), 'policy/stage2')
        os.makedirs(save_dir, exist_ok=True)

        save_path = os.path.join(save_dir, f'online_update_{self.global_update}.pth')
        latest_path = os.path.join(save_dir, 'online_latest.pth')

        torch.save(self.policy.state_dict(), save_path)
        torch.save(self.policy.state_dict(), latest_path)

        rospy.loginfo(f"💾 Model saved: {save_path}")

        # Also log performance stats
        if self.episode_count > 0:
            success_rate = self.success_count / self.episode_count
            collision_rate = self.collision_count / self.episode_count

            rospy.loginfo(f"📊 Performance (last {self.episode_count} episodes):")
            rospy.loginfo(f"   Success rate: {success_rate:.1%}")
            rospy.loginfo(f"   Collision rate: {collision_rate:.1%}")

    def handle_episode_end(self, result: str):
        """Handle episode termination"""
        self.episode_count += 1

        # Update statistics
        if result == "Reach Goal":
            self.success_count += 1
        elif result == "Collision":
            self.collision_count += 1
        elif result == "Timeout":
            self.timeout_count += 1

        rospy.loginfo("="*70)
        rospy.loginfo(f"📊 Episode {self.episode_count} Complete")
        rospy.loginfo(f"   Result: {result}")
        rospy.loginfo(f"   Steps: {self.episode_steps}")
        rospy.loginfo(f"   Reward: {self.episode_reward:.2f}")
        rospy.loginfo("="*70)

        # Reset episode tracking
        self.episode_reward = 0.0
        self.episode_steps = 0

        # Stop robot
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)


def main():
    """Main entry point"""
    try:
        node = ServerInferenceNode()
        rospy.loginfo("Server inference node spinning...")
        rospy.spin()

    except KeyboardInterrupt:
        rospy.loginfo("Shutting down server inference node")
    except Exception as e:
        rospy.logerr(f"Fatal error: {e}")
        import traceback
        rospy.logerr(traceback.format_exc())


if __name__ == '__main__':
    main()
