#!/usr/bin/env python3
"""
Integration Test Script for RL Model Deployment
Tests each component independently before full integration

Usage:
    roscore &
    python3 test_model_integration.py

Author: Generated for Nguyen Tan Khoi's thesis
Date: 2025-11-16
"""

import os
import sys
import rospy
import torch
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from model.net import ActorCriticNetwork
from settings import *


class ModelIntegrationTest:
    def __init__(self):
        rospy.init_node('model_integration_test')

        self.scan_received = False
        self.odom_received = False
        self.amcl_received = False
        self.model_loaded = False

        print("\n" + "="*60)
        print("  RL MODEL INTEGRATION TEST")
        print("="*60)

        # Test 1: Model Loading
        self.test_model_loading()

        # Test 2: Sensor Data
        print("\n[INFO] Waiting for sensor data (5 seconds)...")
        rospy.Subscriber('/robot_0/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/robot_0/odometry/filtered', Odometry, self.odom_callback)
        rospy.Subscriber('/robot_0/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)

        rospy.sleep(5.0)

        # Test 3: Inference
        if self.scan_received and self.odom_received:
            self.test_inference()
        else:
            print("\n[WARNING] Sensor data not received, skipping inference test")

        # Final Report
        self.report_results()

    def test_model_loading(self):
        """Test 1: Load model from file"""
        print("\n=== TEST 1: MODEL LOADING ===")
        try:
            self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
            print(f"[INFO] Using device: {self.device}")

            self.policy = ActorCriticNetwork(frames=LASER_HIST, action_space=ACT_SIZE)
            self.policy.to(self.device)
            print(f"[INFO] Network architecture created")

            model_path = POLICY_PATH / "cnn_modern_best_71pct.pth"
            print(f"[INFO] Loading model from: {model_path}")

            if os.path.exists(model_path):
                state_dict = torch.load(model_path, map_location=self.device)
                self.policy.load_state_dict(state_dict)
                self.model_loaded = True
                print(f"✅ Model loaded successfully!")
                print(f"   - File: {model_path}")
                print(f"   - Size: {os.path.getsize(model_path) / 1024 / 1024:.2f} MB")
            else:
                print(f"❌ Model file not found: {model_path}")
                print(f"   Please run: cp <training_path>/cnn_modern_best_71pct.pth {model_path}")
        except Exception as e:
            print(f"❌ Model loading failed: {e}")
            import traceback
            traceback.print_exc()

    def scan_callback(self, msg):
        """Test 2a: LiDAR data reception"""
        if not self.scan_received:
            print("\n=== TEST 2a: LIDAR DATA ===")
            print(f"✅ Received laser scan")
            print(f"   - Points: {len(msg.ranges)}")
            print(f"   - Range: [{msg.range_min:.2f}, {msg.range_max:.2f}] m")
            print(f"   - Angle: [{msg.angle_min:.2f}, {msg.angle_max:.2f}] rad")
            print(f"   - Increment: {msg.angle_increment:.4f} rad")

            # Check data quality
            valid_scans = [r for r in msg.ranges if not np.isnan(r) and not np.isinf(r)]
            print(f"   - Valid scans: {len(valid_scans)}/{len(msg.ranges)} ({100*len(valid_scans)/len(msg.ranges):.1f}%)")

            self.scan_received = True
            self.last_scan = msg

    def odom_callback(self, msg):
        """Test 2b: Odometry data reception"""
        if not self.odom_received:
            print("\n=== TEST 2b: ODOMETRY DATA (UKF FILTERED) ===")
            print(f"✅ Received odometry from UKF fusion")
            print(f"   - Position: ({msg.pose.pose.position.x:.3f}, {msg.pose.pose.position.y:.3f}) m")
            print(f"   - Orientation (quat): [{msg.pose.pose.orientation.x:.3f}, {msg.pose.pose.orientation.y:.3f}, "
                  f"{msg.pose.pose.orientation.z:.3f}, {msg.pose.pose.orientation.w:.3f}]")
            print(f"   - Linear vel: {msg.twist.twist.linear.x:.3f} m/s")
            print(f"   - Angular vel: {msg.twist.twist.angular.z:.3f} rad/s")

            self.odom_received = True
            self.last_odom = msg

    def amcl_callback(self, msg):
        """Test 2c: AMCL pose reception (optional)"""
        if not self.amcl_received:
            print("\n=== TEST 2c: AMCL LOCALIZATION ===")
            print(f"✅ Received AMCL pose estimate")
            print(f"   - Position: ({msg.pose.pose.position.x:.3f}, {msg.pose.pose.position.y:.3f}) m")

            # Check covariance
            cov = msg.pose.covariance
            print(f"   - Covariance XX: {cov[0]:.4f}")
            print(f"   - Covariance YY: {cov[7]:.4f}")
            print(f"   - Covariance YawYaw: {cov[35]:.4f}")

            if cov[0] < 0.1 and cov[7] < 0.1:
                print(f"   - Localization quality: GOOD (low uncertainty)")
            elif cov[0] < 0.5 and cov[7] < 0.5:
                print(f"   - Localization quality: MEDIUM")
            else:
                print(f"   - Localization quality: POOR (high uncertainty)")
                print(f"   - Tip: Set initial pose in RViz using '2D Pose Estimate'")

            self.amcl_received = True

    def test_inference(self):
        """Test 3: Model inference"""
        print("\n=== TEST 3: MODEL INFERENCE ===")
        if not self.model_loaded:
            print("❌ Cannot test inference - model not loaded")
            return

        try:
            # Create dummy observation matching training format
            print("[INFO] Creating dummy observation...")
            obs = torch.randn(1, LASER_HIST, NUM_OBS).to(self.device)  # (batch=1, frames=3, beams=454)
            goal = torch.randn(1, 2).to(self.device)  # (batch=1, goal_dim=2)
            speed = torch.randn(1, 2).to(self.device)  # (batch=1, speed_dim=2)

            print(f"[INFO] Input shapes:")
            print(f"   - Observation: {obs.shape} (batch, frames, beams)")
            print(f"   - Goal: {goal.shape} (batch, [x, y])")
            print(f"   - Speed: {speed.shape} (batch, [v, w])")

            # Run inference
            print("[INFO] Running inference...")
            with torch.no_grad():
                v, action, logprob, mean = self.policy(obs, goal, speed)

            print(f"✅ Inference successful!")
            print(f"   - Value estimate: {v.item():.4f}")
            print(f"   - Action (raw):")
            print(f"     - Linear (0-1): {mean[0,0].item():.4f}")
            print(f"     - Angular (-1,1): {mean[0,1].item():.4f}")

            # Scale to real bounds
            v_scaled = mean[0,0].item() * ACTION_BOUND[1][0]
            w_scaled = mean[0,1].item() * ACTION_BOUND[1][1]
            print(f"   - Action (scaled to bounds):")
            print(f"     - Linear: {v_scaled:.4f} m/s (max {ACTION_BOUND[1][0]})")
            print(f"     - Angular: {w_scaled:.4f} rad/s (max {ACTION_BOUND[1][1]})")

            # Test with real sensor data if available
            if self.scan_received and self.odom_received:
                print("\n[INFO] Testing with real sensor data...")
                self.test_real_sensor_inference()

        except Exception as e:
            print(f"❌ Inference failed: {e}")
            import traceback
            traceback.print_exc()

    def test_real_sensor_inference(self):
        """Test inference with real sensor data"""
        try:
            # Process laser scan like robot_env.py does
            scan = np.asarray(self.last_scan.ranges)[:NUM_OBS]
            scan[np.isnan(scan)] = 6.0
            scan[np.isinf(scan)] = 6.0
            scan = np.where(scan > 6.0, 6.0, scan)

            # Normalize to [-0.5, 0.5]
            scan_normalized = scan / 6.0 - 0.5

            # Stack 3 frames (using same scan 3 times for this test)
            obs_stack = np.stack([scan_normalized, scan_normalized, scan_normalized])

            # Dummy goal and speed
            goal = np.array([1.0, 0.0])  # 1 meter ahead
            speed = np.array([0.0, 0.0])  # Starting from rest

            # Convert to tensors
            obs_tensor = torch.FloatTensor(obs_stack).unsqueeze(0).to(self.device)
            goal_tensor = torch.FloatTensor(goal).unsqueeze(0).to(self.device)
            speed_tensor = torch.FloatTensor(speed).unsqueeze(0).to(self.device)

            # Inference
            with torch.no_grad():
                v, action, logprob, mean = self.policy(obs_tensor, goal_tensor, speed_tensor)

            # Scale actions
            v_scaled = mean[0,0].item() * ACTION_BOUND[1][0]
            w_scaled = mean[0,1].item() * ACTION_BOUND[1][1]

            print(f"✅ Real sensor inference successful!")
            print(f"   - Processed {len(scan)} laser beams")
            print(f"   - Min distance: {np.min(scan):.2f} m")
            print(f"   - Commanded action:")
            print(f"     - Linear: {v_scaled:.4f} m/s")
            print(f"     - Angular: {w_scaled:.4f} rad/s")

            # Safety check
            min_dist = np.min(scan)
            if min_dist < 0.3 and v_scaled > 0.1:
                print(f"   ⚠️  WARNING: Obstacle at {min_dist:.2f}m but commanding forward motion!")
            elif min_dist < 0.3:
                print(f"   ✅ Safety: Obstacle detected, low/no forward velocity")

        except Exception as e:
            print(f"❌ Real sensor inference failed: {e}")
            import traceback
            traceback.print_exc()

    def report_results(self):
        """Final report"""
        print("\n" + "="*60)
        print("  INTEGRATION TEST RESULTS")
        print("="*60)

        tests = [
            ("Model Loading", self.model_loaded),
            ("LiDAR Data", self.scan_received),
            ("Odometry Data", self.odom_received),
            ("AMCL Localization", self.amcl_received),
        ]

        for test_name, passed in tests:
            status = "✅ PASS" if passed else "❌ FAIL"
            print(f"{test_name:25s}: {status}")

        print("="*60)

        # Overall status
        critical_tests = [self.model_loaded, self.scan_received, self.odom_received]
        if all(critical_tests):
            print("\n🎉 All critical tests passed! Ready for deployment.")
            print("\nNext steps:")
            print("1. roslaunch robot_controller robot_navigation.launch")
            print("2. Set initial pose in RViz (if using AMCL)")
            print("3. Set goal: rostopic pub /robot_0/move_base_simple/goal ...")
        else:
            print("\n⚠️  Some critical tests failed. Fix issues before deployment.")
            if not self.model_loaded:
                print("- Model loading failed: Check model file path")
            if not self.scan_received:
                print("- LiDAR data not received: Check sensor connection and topics")
            if not self.odom_received:
                print("- Odometry not received: Check robot_bringup.launch")

        print("")


if __name__ == '__main__':
    try:
        print("\n[INFO] Starting integration test...")
        print("[INFO] Make sure roscore is running!")
        print("[INFO] For full test, also run: roslaunch robot_controller robot_bringup.launch\n")

        tester = ModelIntegrationTest()

        # Keep node alive briefly to receive all callbacks
        rospy.sleep(1.0)

    except rospy.ROSInterruptException:
        print("\n[INFO] Test interrupted by user")
    except KeyboardInterrupt:
        print("\n[INFO] Test interrupted by user")
    except Exception as e:
        print(f"\n[ERROR] Test failed with exception: {e}")
        import traceback
        traceback.print_exc()
