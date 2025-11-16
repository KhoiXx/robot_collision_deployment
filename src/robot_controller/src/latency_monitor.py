#!/usr/bin/env python3
"""
Latency Monitor - Measure end-to-end control loop latency

Measures time from sensor data (LiDAR) published to cmd_vel received.
This captures the full control loop including:
- Network transmission (Jetson → Master)
- AMCL localization
- Model inference
- Network transmission (Master → Jetson)

Usage:
    python3 latency_monitor.py --robot_id 0

Author: Claude AI
Date: 2025-11-16
"""

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from collections import deque


class LatencyMonitor:
    def __init__(self, robot_id=0):
        self.robot_id = robot_id
        self.scan_times = deque(maxlen=10)  # Store recent scan timestamps

        # Subscribe to sensor input (start of control loop)
        rospy.Subscriber(f'/robot_{robot_id}/scan', LaserScan, self.scan_callback)

        # Subscribe to control output (end of control loop)
        rospy.Subscriber(f'/robot_{robot_id}/cmd_vel', Twist, self.cmd_vel_callback)

        # Publish latency statistics
        self.latency_pub = rospy.Publisher(f'/robot_{robot_id}/control_latency', Float32, queue_size=10)

        # Statistics
        self.latencies = []
        self.max_samples = 100
        self.sample_count = 0

        rospy.loginfo(f"Latency Monitor started for robot_{robot_id}")

    def scan_callback(self, msg):
        """Record scan timestamp"""
        self.scan_times.append(msg.header.stamp)

    def cmd_vel_callback(self, msg):
        """Calculate latency when cmd_vel received"""
        if len(self.scan_times) == 0:
            return

        # Use most recent scan timestamp
        scan_time = self.scan_times[-1]
        now = rospy.Time.now()

        # Calculate latency in milliseconds
        latency = (now - scan_time).to_sec() * 1000

        # Filter out unrealistic values
        if latency < 0 or latency > 1000:
            rospy.logwarn(f"Unrealistic latency: {latency:.1f}ms - skipping")
            return

        # Store latency
        self.latencies.append(latency)
        if len(self.latencies) > self.max_samples:
            self.latencies.pop(0)

        # Publish current latency
        self.latency_pub.publish(latency)

        self.sample_count += 1

        # Log statistics every 10 samples
        if self.sample_count % 10 == 0:
            self._log_statistics()

    def _log_statistics(self):
        """Log latency statistics"""
        if len(self.latencies) == 0:
            return

        avg_latency = np.mean(self.latencies)
        std_latency = np.std(self.latencies)
        min_latency = np.min(self.latencies)
        max_latency = np.max(self.latencies)
        p95_latency = np.percentile(self.latencies, 95)

        rospy.loginfo(f"[Robot {self.robot_id}] Latency (ms) - "
                     f"avg: {avg_latency:.1f} ± {std_latency:.1f}, "
                     f"min: {min_latency:.1f}, "
                     f"max: {max_latency:.1f}, "
                     f"p95: {p95_latency:.1f}")

        # Warnings
        if avg_latency > 200:
            rospy.logerr(f"❌ CRITICAL LATENCY: {avg_latency:.1f}ms > 200ms!")
        elif avg_latency > 150:
            rospy.logwarn(f"⚠️  HIGH LATENCY: {avg_latency:.1f}ms > 150ms")
        elif avg_latency > 100:
            rospy.logwarn(f"⚠️  Moderate latency: {avg_latency:.1f}ms > 100ms")


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Monitor control loop latency')
    parser.add_argument('--robot_id', type=int, default=0, help='Robot ID (0, 1, 2, ...)')
    args = parser.parse_args()

    try:
        rospy.init_node(f'latency_monitor_{args.robot_id}')
        monitor = LatencyMonitor(args.robot_id)

        rospy.loginfo("="*60)
        rospy.loginfo(f"Latency Monitor Running for Robot {args.robot_id}")
        rospy.loginfo("Monitor topics:")
        rospy.loginfo(f"  Input:  /robot_{args.robot_id}/scan")
        rospy.loginfo(f"  Output: /robot_{args.robot_id}/cmd_vel")
        rospy.loginfo(f"  Stats:  /robot_{args.robot_id}/control_latency")
        rospy.loginfo("="*60)

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Latency monitor shutting down")
    except KeyboardInterrupt:
        rospy.loginfo("Latency monitor interrupted by user")
