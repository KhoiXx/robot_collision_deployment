#!/usr/bin/env python3
"""
Encoder Performance Test
========================
Tests 3 targets:
1. Smoothness: Latency cmd_vel → encoder response
2. Frequency: Actual Hz and packet loss detection
3. Data Quality: Encoder accuracy and consistency

Usage:
    rosrun robot_controller test_encoder_performance.py
"""

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from collections import deque
from datetime import datetime


class EncoderPerformanceTest:
    def __init__(self):
        rospy.init_node("encoder_performance_test")

        # Configuration
        self.robot_namespace = rospy.get_param("~robot_namespace", "/robot_0")
        self.window_size = 100  # samples for statistics

        # Subscribers
        rospy.Subscriber(f"{self.robot_namespace}/cmd_vel", Twist, self.cmd_vel_callback)
        rospy.Subscriber(f"{self.robot_namespace}/odom", Odometry, self.odom_callback)

        # === TARGET 1: SMOOTHNESS - Latency Tracking ===
        self.last_cmd_time = None
        self.last_cmd_linear = 0.0
        self.last_cmd_angular = 0.0
        self.latencies = deque(maxlen=self.window_size)

        # === TARGET 2: FREQUENCY - Packet Loss Detection ===
        self.odom_timestamps = deque(maxlen=self.window_size)
        self.last_odom_time = None
        self.expected_dt = 1.0 / 30.0  # 30Hz expected
        self.packet_loss_count = 0
        self.total_packets = 0

        # === TARGET 3: DATA QUALITY - Encoder Accuracy ===
        self.velocity_errors = deque(maxlen=self.window_size)
        self.timestamp_jitters = deque(maxlen=self.window_size)

        # Statistics Timer
        rospy.Timer(rospy.Duration(2.0), self.print_statistics)

        rospy.loginfo("=" * 80)
        rospy.loginfo("ENCODER PERFORMANCE TEST STARTED")
        rospy.loginfo("=" * 80)
        rospy.loginfo(f"Namespace: {self.robot_namespace}")
        rospy.loginfo(f"Window size: {self.window_size} samples")
        rospy.loginfo(f"Expected frequency: {1.0/self.expected_dt:.1f} Hz")
        rospy.loginfo("=" * 80)

    def cmd_vel_callback(self, msg: Twist):
        """Track cmd_vel commands to measure response latency"""
        self.last_cmd_time = rospy.Time.now()
        self.last_cmd_linear = msg.linear.x
        self.last_cmd_angular = msg.angular.z

    def odom_callback(self, msg: Odometry):
        """
        Analyze odometry for:
        1. Latency (time from cmd_vel to odom update)
        2. Frequency (actual Hz and packet loss)
        3. Accuracy (encoder velocity vs commanded)
        """
        current_time = rospy.Time.now()
        self.total_packets += 1

        # === TARGET 1: LATENCY ===
        if self.last_cmd_time is not None:
            latency = (current_time - self.last_cmd_time).to_sec() * 1000  # ms
            self.latencies.append(latency)

        # === TARGET 2: FREQUENCY & PACKET LOSS ===
        if self.last_odom_time is not None:
            dt = (current_time - self.last_odom_time).to_sec()
            self.odom_timestamps.append(dt)

            # Detect packet loss (gap > 1.5x expected period)
            if dt > self.expected_dt * 1.5:
                self.packet_loss_count += 1
                rospy.logwarn(f"Packet loss detected! Gap: {dt*1000:.1f}ms (expected: {self.expected_dt*1000:.1f}ms)")

            # Timestamp jitter
            jitter = abs(dt - self.expected_dt) * 1000  # ms
            self.timestamp_jitters.append(jitter)

        self.last_odom_time = current_time

        # === TARGET 3: ENCODER ACCURACY ===
        measured_linear = msg.twist.twist.linear.x
        measured_angular = msg.twist.twist.angular.z

        # Error vs commanded velocity
        linear_error = abs(measured_linear - self.last_cmd_linear)
        angular_error = abs(measured_angular - self.last_cmd_angular)
        combined_error = linear_error + angular_error
        self.velocity_errors.append(combined_error)

    def print_statistics(self, event):
        """Print comprehensive statistics every 2 seconds"""
        if len(self.odom_timestamps) < 10:
            rospy.loginfo("Collecting data...")
            return

        rospy.loginfo("\n" + "=" * 80)
        rospy.loginfo(f"ENCODER PERFORMANCE STATISTICS @ {datetime.now().strftime('%H:%M:%S')}")
        rospy.loginfo("=" * 80)

        # === TARGET 1: SMOOTHNESS (LATENCY) ===
        if len(self.latencies) > 0:
            latency_arr = np.array(self.latencies)
            rospy.loginfo("TARGET 1: SMOOTHNESS - cmd_vel → odom Latency")
            rospy.loginfo(f"  Min:     {np.min(latency_arr):.2f} ms")
            rospy.loginfo(f"  Max:     {np.max(latency_arr):.2f} ms")
            rospy.loginfo(f"  Mean:    {np.mean(latency_arr):.2f} ms")
            rospy.loginfo(f"  Std Dev: {np.std(latency_arr):.2f} ms")

            # Pass/Fail criteria
            avg_latency = np.mean(latency_arr)
            if avg_latency < 50:
                rospy.loginfo(f"  Status:  ✓ PASS (< 50ms)")
            elif avg_latency < 100:
                rospy.logwarn(f"  Status:  ⚠ WARNING (50-100ms)")
            else:
                rospy.logerr(f"  Status:  ✗ FAIL (> 100ms)")

        rospy.loginfo("-" * 80)

        # === TARGET 2: FREQUENCY & PACKET LOSS ===
        if len(self.odom_timestamps) > 0:
            dt_arr = np.array(self.odom_timestamps)
            actual_freq = 1.0 / np.mean(dt_arr)
            packet_loss_rate = (self.packet_loss_count / self.total_packets) * 100

            rospy.loginfo("TARGET 2: FREQUENCY & PACKET LOSS")
            rospy.loginfo(f"  Expected:     {1.0/self.expected_dt:.1f} Hz")
            rospy.loginfo(f"  Actual:       {actual_freq:.2f} Hz")
            rospy.loginfo(f"  Period Mean:  {np.mean(dt_arr)*1000:.2f} ms")
            rospy.loginfo(f"  Period Std:   {np.std(dt_arr)*1000:.2f} ms")
            rospy.loginfo(f"  Packets:      {self.total_packets}")
            rospy.loginfo(f"  Packet Loss:  {self.packet_loss_count} ({packet_loss_rate:.2f}%)")

            # Pass/Fail criteria
            freq_deviation = abs(actual_freq - (1.0/self.expected_dt))
            if packet_loss_rate < 1.0 and freq_deviation < 2.0:
                rospy.loginfo(f"  Status:       ✓ PASS")
            elif packet_loss_rate < 5.0:
                rospy.logwarn(f"  Status:       ⚠ WARNING")
            else:
                rospy.logerr(f"  Status:       ✗ FAIL")

        rospy.loginfo("-" * 80)

        # === TARGET 3: DATA QUALITY ===
        if len(self.velocity_errors) > 0 and len(self.timestamp_jitters) > 0:
            error_arr = np.array(self.velocity_errors)
            jitter_arr = np.array(self.timestamp_jitters)

            rospy.loginfo("TARGET 3: DATA QUALITY - Encoder Accuracy")
            rospy.loginfo(f"  Velocity Error Mean: {np.mean(error_arr):.4f} m/s")
            rospy.loginfo(f"  Velocity Error Max:  {np.max(error_arr):.4f} m/s")
            rospy.loginfo(f"  Timestamp Jitter Avg: {np.mean(jitter_arr):.2f} ms")
            rospy.loginfo(f"  Timestamp Jitter Max: {np.max(jitter_arr):.2f} ms")

            # Pass/Fail criteria
            avg_error = np.mean(error_arr)
            avg_jitter = np.mean(jitter_arr)
            if avg_error < 0.05 and avg_jitter < 5.0:
                rospy.loginfo(f"  Status: ✓ PASS")
            elif avg_error < 0.1 and avg_jitter < 10.0:
                rospy.logwarn(f"  Status: ⚠ WARNING")
            else:
                rospy.logerr(f"  Status: ✗ FAIL")

        rospy.loginfo("=" * 80 + "\n")

        # === RECOMMENDATIONS ===
        if len(self.odom_timestamps) > 0:
            dt_arr = np.array(self.odom_timestamps)
            actual_freq = 1.0 / np.mean(dt_arr)

            if actual_freq > 25:
                rospy.loginfo("💡 RECOMMENDATION: Current frequency is good. Can potentially reduce to save CPU.")
            elif actual_freq < 20:
                rospy.logwarn("⚠️  RECOMMENDATION: Frequency too low. Increase to at least 25Hz.")

            if self.packet_loss_count > 0:
                rospy.logwarn("⚠️  RECOMMENDATION: Packet loss detected. Check serial connection and Arduino timing.")


if __name__ == "__main__":
    try:
        test = EncoderPerformanceTest()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
