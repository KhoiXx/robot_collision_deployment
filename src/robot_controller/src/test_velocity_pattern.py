#!/usr/bin/env python3
"""
Velocity Pattern Generator
===========================
Generates various velocity patterns to test encoder response and smoothness.

Patterns:
1. STEP: 0 → max → 0 (instant changes)
2. RAMP: Linear increase/decrease
3. SINE: Smooth oscillation
4. SQUARE: On/off rapid changes
5. RANDOM: Stress test

Usage:
    rosrun robot_controller test_velocity_pattern.py _pattern:=step
    rosrun robot_controller test_velocity_pattern.py _pattern:=ramp
    rosrun robot_controller test_velocity_pattern.py _pattern:=sine
    rosrun robot_controller test_velocity_pattern.py _pattern:=square
    rosrun robot_controller test_velocity_pattern.py _pattern:=random
"""

import rospy
import numpy as np
from geometry_msgs.msg import Twist


class VelocityPatternGenerator:
    def __init__(self):
        rospy.init_node("velocity_pattern_generator")

        # Configuration
        self.robot_namespace = rospy.get_param("~robot_namespace", "/robot_0")
        self.pattern = rospy.get_param("~pattern", "step")  # step, ramp, sine, square, random
        self.max_linear = rospy.get_param("~max_linear", 0.3)  # m/s
        self.max_angular = rospy.get_param("~max_angular", 0.5)  # rad/s
        self.publish_rate = rospy.get_param("~publish_rate", 10.0)  # Hz

        # Publisher
        self.cmd_vel_pub = rospy.Publisher(f"{self.robot_namespace}/cmd_vel", Twist, queue_size=1)

        # Pattern state
        self.start_time = rospy.Time.now()
        self.elapsed = 0.0

        rospy.loginfo("=" * 80)
        rospy.loginfo("VELOCITY PATTERN GENERATOR")
        rospy.loginfo("=" * 80)
        rospy.loginfo(f"Namespace:    {self.robot_namespace}")
        rospy.loginfo(f"Pattern:      {self.pattern.upper()}")
        rospy.loginfo(f"Max Linear:   {self.max_linear:.2f} m/s")
        rospy.loginfo(f"Max Angular:  {self.max_angular:.2f} rad/s")
        rospy.loginfo(f"Publish Rate: {self.publish_rate:.1f} Hz")
        rospy.loginfo("=" * 80)

        # Select pattern function
        self.pattern_func = self.get_pattern_function()

        # Start publishing
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            self.publish_pattern()
            rate.sleep()

    def get_pattern_function(self):
        """Map pattern name to function"""
        patterns = {
            "step": self.pattern_step,
            "ramp": self.pattern_ramp,
            "sine": self.pattern_sine,
            "square": self.pattern_square,
            "random": self.pattern_random,
        }

        if self.pattern not in patterns:
            rospy.logwarn(f"Unknown pattern '{self.pattern}'. Using 'step'.")
            return patterns["step"]

        return patterns[self.pattern]

    def publish_pattern(self):
        """Generate and publish velocity command"""
        self.elapsed = (rospy.Time.now() - self.start_time).to_sec()
        linear, angular = self.pattern_func()

        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular

        self.cmd_vel_pub.publish(cmd)
        rospy.loginfo_throttle(1.0, f"[{self.elapsed:.1f}s] v={linear:.3f} m/s, w={angular:.3f} rad/s")

    # ========================================================================
    # PATTERN GENERATORS
    # ========================================================================

    def pattern_step(self):
        """
        STEP RESPONSE: Test instant response
        0 → max → 0 → -max → 0
        Each step lasts 3 seconds
        """
        cycle = self.elapsed % 12.0  # 12s total cycle

        if cycle < 3.0:
            # Forward max
            return self.max_linear, 0.0
        elif cycle < 6.0:
            # Stop
            return 0.0, 0.0
        elif cycle < 9.0:
            # Rotate max
            return 0.0, self.max_angular
        else:
            # Stop
            return 0.0, 0.0

    def pattern_ramp(self):
        """
        RAMP RESPONSE: Test gradual changes
        Linear increase from 0 to max, then decrease back to 0
        """
        cycle = self.elapsed % 10.0  # 10s cycle

        if cycle < 5.0:
            # Ramp up
            progress = cycle / 5.0
            return self.max_linear * progress, 0.0
        else:
            # Ramp down
            progress = (10.0 - cycle) / 5.0
            return self.max_linear * progress, 0.0

    def pattern_sine(self):
        """
        SINE WAVE: Test smooth oscillation
        Smooth sinusoidal velocity changes
        """
        freq = 0.2  # Hz (5s period)
        linear = (self.max_linear / 2) * (1 + np.sin(2 * np.pi * freq * self.elapsed))
        angular = (self.max_angular / 2) * np.sin(2 * np.pi * freq * self.elapsed * 1.5)

        return linear, angular

    def pattern_square(self):
        """
        SQUARE WAVE: Test rapid on/off changes
        Alternates between max and 0 every second
        """
        cycle = self.elapsed % 2.0

        if cycle < 1.0:
            return self.max_linear, 0.0
        else:
            return 0.0, 0.0

    def pattern_random(self):
        """
        RANDOM: Stress test with random velocities
        Changes every 0.5 seconds
        """
        # Seed based on elapsed time (changes every 0.5s)
        seed = int(self.elapsed / 0.5)
        np.random.seed(seed)

        linear = np.random.uniform(0, self.max_linear)
        angular = np.random.uniform(-self.max_angular, self.max_angular)

        return linear, angular


class InteractiveVelocityControl:
    """
    Interactive keyboard control for manual testing
    """
    def __init__(self):
        rospy.init_node("interactive_velocity_control")

        self.robot_namespace = rospy.get_param("~robot_namespace", "/robot_0")
        self.cmd_vel_pub = rospy.Publisher(f"{self.robot_namespace}/cmd_vel", Twist, queue_size=1)

        rospy.loginfo("=" * 80)
        rospy.loginfo("INTERACTIVE VELOCITY CONTROL")
        rospy.loginfo("=" * 80)
        rospy.loginfo("Commands:")
        rospy.loginfo("  w - Forward (0.1 m/s)")
        rospy.loginfo("  s - Stop")
        rospy.loginfo("  a - Rotate left (0.3 rad/s)")
        rospy.loginfo("  d - Rotate right (0.3 rad/s)")
        rospy.loginfo("  q - Quit")
        rospy.loginfo("=" * 80)

    def run(self):
        """Simple keyboard control loop"""
        import sys
        import select
        import termios
        import tty

        settings = termios.tcgetattr(sys.stdin)

        try:
            tty.setraw(sys.stdin.fileno())

            while not rospy.is_shutdown():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)

                    cmd = Twist()

                    if key == 'w':
                        cmd.linear.x = 0.1
                        rospy.loginfo("Forward: 0.1 m/s")
                    elif key == 's':
                        cmd.linear.x = 0.0
                        cmd.angular.z = 0.0
                        rospy.loginfo("Stop")
                    elif key == 'a':
                        cmd.angular.z = 0.3
                        rospy.loginfo("Rotate left: 0.3 rad/s")
                    elif key == 'd':
                        cmd.angular.z = -0.3
                        rospy.loginfo("Rotate right: -0.3 rad/s")
                    elif key == 'q':
                        rospy.loginfo("Quit")
                        break

                    self.cmd_vel_pub.publish(cmd)

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == "__main__":
    try:
        mode = rospy.get_param("~mode", "pattern")

        if mode == "interactive":
            controller = InteractiveVelocityControl()
            controller.run()
        else:
            VelocityPatternGenerator()

    except rospy.ROSInterruptException:
        pass
