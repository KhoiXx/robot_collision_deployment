#!/usr/bin/env python3
"""
Trajectory Logger for Robot Navigation
Records robot trajectory and publishes Path for RViz visualization

Author: Claude
Date: 2025-11-23
"""
import os
import csv
from datetime import datetime
from pathlib import Path

import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path as NavPath
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse


class TrajectoryLogger:
    def __init__(self, robot_index=0):
        self.robot_index = robot_index

        # Trajectory data storage
        self.trajectory_data = []
        self.current_goal = None
        self.is_recording = False
        self.start_time = None

        # Path message for RViz
        self.path_msg = NavPath()
        self.path_msg.header.frame_id = "map"

        # Output directory
        self.output_dir = Path.home() / "trajectory_logs"
        self.output_dir.mkdir(exist_ok=True)

        # Current session file
        self.current_csv_file = None
        self.csv_writer = None
        self.csv_file_handle = None

        # Parameters
        self.sample_interval = rospy.get_param("~sample_interval", 0.1)  # 10 Hz logging
        self.last_sample_time = rospy.Time.now()

        # Subscribers
        rospy.Subscriber(
            f"/robot_{robot_index}/amcl_pose",
            PoseWithCovarianceStamped,
            self.amcl_callback
        )
        rospy.Subscriber(
            f"/robot_{robot_index}/cmd_vel",
            Twist,
            self.cmd_vel_callback
        )
        rospy.Subscriber(
            f"/robot_{robot_index}/move_base_simple/goal",
            PoseStamped,
            self.goal_callback
        )

        # Publisher for RViz Path visualization
        self.path_pub = rospy.Publisher(
            f"/robot_{robot_index}/trajectory",
            NavPath,
            queue_size=1,
            latch=True  # Keep last message for new subscribers
        )

        # Services
        rospy.Service(
            f"/robot_{robot_index}/start_recording",
            Trigger,
            self.start_recording_service
        )
        rospy.Service(
            f"/robot_{robot_index}/stop_recording",
            Trigger,
            self.stop_recording_service
        )
        rospy.Service(
            f"/robot_{robot_index}/clear_trajectory",
            Trigger,
            self.clear_trajectory_service
        )

        # Current state
        self.current_pose = None
        self.current_vel = [0.0, 0.0]

        rospy.loginfo(f"TrajectoryLogger initialized for robot_{robot_index}")
        rospy.loginfo(f"  - Output dir: {self.output_dir}")
        rospy.loginfo(f"  - Path topic: /robot_{robot_index}/trajectory")
        rospy.loginfo(f"  - Services: start_recording, stop_recording, clear_trajectory")

    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        """Process AMCL pose and record trajectory"""
        self.current_pose = msg.pose.pose

        # Check sample interval
        current_time = rospy.Time.now()
        if (current_time - self.last_sample_time).to_sec() < self.sample_interval:
            return
        self.last_sample_time = current_time

        # Always update Path for RViz (even when not recording to CSV)
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose = msg.pose.pose
        self.path_msg.poses.append(pose_stamped)
        self.path_msg.header.stamp = current_time
        self.path_pub.publish(self.path_msg)

        # Record to CSV if recording is active
        if self.is_recording and self.csv_writer:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y

            # Extract yaw from quaternion
            quat = msg.pose.pose.orientation
            import tf
            euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
            theta = euler[2]

            # Covariance (diagonal elements: xx, yy, theta-theta)
            cov = msg.pose.covariance
            cov_xx = cov[0]
            cov_yy = cov[7]
            cov_tt = cov[35]

            # Goal info
            goal_x = self.current_goal[0] if self.current_goal else 0.0
            goal_y = self.current_goal[1] if self.current_goal else 0.0

            # Distance to goal
            if self.current_goal:
                import math
                dist_to_goal = math.sqrt((x - goal_x)**2 + (y - goal_y)**2)
            else:
                dist_to_goal = 0.0

            # Time since start
            elapsed = (current_time - self.start_time).to_sec() if self.start_time else 0.0

            # Write row
            self.csv_writer.writerow([
                f"{elapsed:.3f}",
                f"{x:.4f}",
                f"{y:.4f}",
                f"{theta:.4f}",
                f"{self.current_vel[0]:.4f}",
                f"{self.current_vel[1]:.4f}",
                f"{goal_x:.4f}",
                f"{goal_y:.4f}",
                f"{dist_to_goal:.4f}",
                f"{cov_xx:.6f}",
                f"{cov_yy:.6f}",
                f"{cov_tt:.6f}"
            ])

            # Store in memory too
            self.trajectory_data.append({
                'time': elapsed,
                'x': x,
                'y': y,
                'theta': theta,
                'v': self.current_vel[0],
                'w': self.current_vel[1],
                'goal_x': goal_x,
                'goal_y': goal_y,
                'dist_to_goal': dist_to_goal
            })

    def cmd_vel_callback(self, msg: Twist):
        """Track current velocity"""
        self.current_vel = [msg.linear.x, msg.angular.z]

    def goal_callback(self, msg: PoseStamped):
        """Handle new goal - auto start recording"""
        self.current_goal = [msg.pose.position.x, msg.pose.position.y]
        rospy.loginfo(f"New goal received: ({self.current_goal[0]:.2f}, {self.current_goal[1]:.2f})")

        # Auto-start recording when goal is received
        if not self.is_recording:
            self._start_recording()

    def _start_recording(self):
        """Start recording trajectory to CSV"""
        if self.is_recording:
            rospy.logwarn("Already recording!")
            return False

        # Create new CSV file
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.current_csv_file = self.output_dir / f"trajectory_{timestamp}.csv"

        self.csv_file_handle = open(self.current_csv_file, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file_handle)

        # Write header
        self.csv_writer.writerow([
            'time', 'x', 'y', 'theta', 'v', 'w',
            'goal_x', 'goal_y', 'dist_to_goal',
            'cov_xx', 'cov_yy', 'cov_tt'
        ])

        self.trajectory_data = []
        self.start_time = rospy.Time.now()
        self.is_recording = True

        rospy.loginfo(f"Started recording to: {self.current_csv_file}")
        return True

    def _stop_recording(self):
        """Stop recording and close CSV file"""
        if not self.is_recording:
            rospy.logwarn("Not recording!")
            return False

        self.is_recording = False

        if self.csv_file_handle:
            self.csv_file_handle.close()
            self.csv_file_handle = None
            self.csv_writer = None

        rospy.loginfo(f"Stopped recording. Saved to: {self.current_csv_file}")
        rospy.loginfo(f"Total points recorded: {len(self.trajectory_data)}")

        return True

    def start_recording_service(self, req):
        """Service to start recording"""
        success = self._start_recording()
        msg = f"Recording started: {self.current_csv_file}" if success else "Already recording"
        return TriggerResponse(success=success, message=msg)

    def stop_recording_service(self, req):
        """Service to stop recording"""
        success = self._stop_recording()
        msg = f"Recording stopped: {self.current_csv_file}" if success else "Not recording"
        return TriggerResponse(success=success, message=msg)

    def clear_trajectory_service(self, req):
        """Service to clear trajectory display"""
        self.path_msg.poses = []
        self.path_pub.publish(self.path_msg)
        self.trajectory_data = []
        return TriggerResponse(success=True, message="Trajectory cleared")

    def shutdown(self):
        """Cleanup on shutdown"""
        if self.is_recording:
            self._stop_recording()
        rospy.loginfo("TrajectoryLogger shutdown complete")


def main():
    rospy.init_node('trajectory_logger')

    robot_index = rospy.get_param("~robot_index", 0)
    logger = TrajectoryLogger(robot_index)

    rospy.on_shutdown(logger.shutdown)

    rospy.loginfo("="*50)
    rospy.loginfo("TrajectoryLogger READY")
    rospy.loginfo("="*50)
    rospy.loginfo("Usage:")
    rospy.loginfo("  - Auto-starts recording when goal is set in RViz")
    rospy.loginfo("  - Or manually: rosservice call /robot_0/start_recording")
    rospy.loginfo("  - Stop: rosservice call /robot_0/stop_recording")
    rospy.loginfo("  - Clear RViz path: rosservice call /robot_0/clear_trajectory")
    rospy.loginfo("  - View in RViz: Add Path display for /robot_0/trajectory")
    rospy.loginfo("="*50)

    rospy.spin()


if __name__ == "__main__":
    main()
