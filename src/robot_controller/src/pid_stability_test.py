#!/usr/bin/env python3

import csv
import os
import struct
import time
from datetime import datetime

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from robot_control import RobotControl
from settings import *


class PIDStabilityTest:
    def __init__(self, robot_index=0, test_wheel="left", output_dir="pid_test_results"):
        self.robot = RobotControl(robot_index, port="/dev/esp", baudrate=115200)
        self.test_wheel = test_wheel
        self.wheel_id = LEFT_WHEEL if test_wheel == "left" else RIGHT_WHEEL

        # Create output directory
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)

        # Data storage
        self.data_points = []
        self.test_start_time = None
        self.collecting = False

        rospy.loginfo(f"PID Stability Test initialized for {test_wheel} wheel")

    def request_pid_data(self):
        """Request PID data from ESP32"""
        self.robot.clear_buffer("input")
        # Send DIRECTLY to avoid queue
        self.robot._send_command(GET_PID_DATA, "<f", self.wheel_id)
        rospy.sleep(0.02)
        return self.read_pid_data()

    def read_pid_data(self, timeout=0.5):
        """Read PID data from serial"""
        start_time = time.time()

        # while (time.time() - start_time) < timeout:
        if self.robot.port.in_waiting >= 22:  # Full packet size: 'B' + 22 bytes
            # Read first byte
            datatype = self.robot.port.read(1)
            if datatype == b'B':
                # Read remaining 22 bytes
                received_data = self.robot.port.read(22)
                if received_data[0] == GET_PID_DATA and len(received_data) >= 21:
                    # Parse: CMD(1) | timestamp(4) | setpoint(4) | current(4) | error(4) | output(4) | CRC(1)
                    try:
                        timestamp,setpoint, current, error, output = struct.unpack('<Lffff', received_data[1:-1])

                        print(f"Setpoint: {setpoint}, Current: {current}, Error: {error}, Output: {output}")
                        # Verify CRC (optional)
                        crc_received = received_data[-1]

                        return {
                            'timestamp': timestamp,
                            'setpoint': setpoint,
                            'current': current,
                            'error': error,
                            'output': output
                        }
                    except Exception as e:
                        rospy.logerr(f"Error parsing PID data: {e}")
                        return None
            elif datatype == b'S':
                # Text response, read and skip
                line = self.robot.port.readline().decode().strip()
                rospy.logwarn(f"Skipping text response: S{line}")

        time.sleep(0.01)

        return None

    def set_speed(self, speed):
        """Set target speed for the test wheel"""
        # Send command DIRECTLY to avoid queue issues
        if self.test_wheel == "left":
            self.robot._send_command(CMD_VEL_LEFT, "<f", float(speed))
        else:
            self.robot._send_command(CMD_VEL_RIGHT, "<f", float(speed))

        rospy.loginfo(f"Set {self.test_wheel} wheel speed to {speed} m/s")
        # Wait for ESP32 to process
        rospy.sleep(0.05)

    def run_step_response_test(self, target_speeds, duration_per_step=5.0):
        """
        Run step response test with different target speeds

        Args:
            target_speeds: List of target speeds to test (m/s)
            duration_per_step: Duration for each step (seconds)
        """
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_filename = os.path.join(
            self.output_dir,
            f"step_response_{self.test_wheel}_{timestamp_str}.csv"
        )

        rospy.loginfo(f"Starting step response test, saving to {csv_filename}")

        with open(csv_filename, 'w', newline='') as csvfile:
            fieldnames = ['test_time', 'timestamp_ms', 'target_speed', 'setpoint', 'current', 'error', 'output']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()

            for target_speed in target_speeds:
                rospy.loginfo(f"Testing target speed: {target_speed} m/s")

                # Set new target speed
                self.set_speed(target_speed)
                rospy.sleep(0.1)  # Let command be processed

                test_start = time.time()

                while (time.time() - test_start) < duration_per_step:
                    if rospy.is_shutdown():
                        break

                    # Request and read PID data
                    pid_data = self.request_pid_data()

                    if pid_data:
                        test_time = time.time() - test_start
                        writer.writerow({
                            'test_time': f"{test_time:.3f}",
                            'timestamp_ms': pid_data['timestamp'],
                            'target_speed': f"{target_speed:.3f}",
                            'setpoint': f"{pid_data['setpoint']:.3f}",
                            'current': f"{pid_data['current']:.3f}",
                            'error': f"{pid_data['error']:.3f}",
                            'output': f"{pid_data['output']:.3f}"
                        })

                    rospy.sleep(0.1)  # Sample at 10Hz

                rospy.loginfo(f"Completed test for speed {target_speed} m/s")

        # Stop the motor
        self.set_speed(0.0)
        rospy.loginfo(f"Test complete! Data saved to {csv_filename}")
        return csv_filename

    def run_disturbance_test(self, target_speed=0.5, duration=10.0):
        """
        Run test to observe how system responds to maintaining speed
        (useful for observing steady-state behavior and noise rejection)
        """
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_filename = os.path.join(
            self.output_dir,
            f"disturbance_{self.test_wheel}_{timestamp_str}.csv"
        )

        rospy.loginfo(f"Starting disturbance rejection test at {target_speed} m/s")

        with open(csv_filename, 'w', newline='') as csvfile:
            fieldnames = ['test_time', 'timestamp_ms', 'setpoint', 'current', 'error', 'output']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()

            # Set target speed
            self.set_speed(target_speed)
            rospy.sleep(0.1)

            test_start = time.time()

            while (time.time() - test_start) < duration:
                if rospy.is_shutdown():
                    break

                pid_data = self.request_pid_data()
                print(pid_data)
                if pid_data:
                    test_time = time.time() - test_start
                    writer.writerow({
                        'test_time': f"{test_time:.3f}",
                        'timestamp_ms': pid_data['timestamp'],
                        'setpoint': f"{pid_data['setpoint']:.3f}",
                        'current': f"{pid_data['current']:.3f}",
                        'error': f"{pid_data['error']:.3f}",
                        'output': f"{pid_data['output']:.3f}"
                    })

                rospy.sleep(0.05)

        # Stop the motor
        self.set_speed(0.0)
        rospy.loginfo(f"Disturbance test complete! Data saved to {csv_filename}")
        return csv_filename


if __name__ == '__main__':
    rospy.init_node("pid_stability_test")

    # Test parameters
    test_wheel = rospy.get_param('~test_wheel', 'left')  # 'left' or 'right'
    output_dir = rospy.get_param('~output_dir', 'pid_test_results')

    # Create tester
    tester = PIDStabilityTest(
        robot_index=0,
        test_wheel=test_wheel,
        output_dir=output_dir
    )

    rospy.loginfo("Starting PID stability tests...")
    rospy.sleep(1.0)

    try:
        # Test 1: Step response with different speeds (positive and negative)
        target_speeds = [0.2, 0.4, 0.6, -0.3, -0.5, 0.0]
        csv_file = tester.run_step_response_test(
            target_speeds=target_speeds,
            duration_per_step=5.0
        )

        rospy.loginfo("Step response test completed!")
        rospy.sleep(2.0)

        # Test 2: Disturbance rejection
        csv_file = tester.run_disturbance_test(
            target_speed=0.5,
            duration=10.0
        )

        rospy.loginfo("All tests completed successfully!")

    except KeyboardInterrupt:
        rospy.loginfo("Test interrupted by user")
    except Exception as e:
        rospy.logerr(f"Test failed with error: {e}")
    finally:
        tester.set_speed(0.0)
        rospy.loginfo("Motor stopped")
