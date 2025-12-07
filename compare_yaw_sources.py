#!/usr/bin/env python3
"""
Compare YAW from different sources:
- Encoder (from odom)
- IMU
- UKF (fusion result)
"""

import rospy
import tf
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

class YawComparator:
    def __init__(self):
        rospy.init_node('yaw_comparator', anonymous=True)

        self.encoder_yaw = None
        self.imu_yaw = None
        self.imu_raw_yaw = None
        self.ukf_yaw = None

        # Track history for stability analysis
        from collections import deque
        self.encoder_history = deque(maxlen=100)
        self.imu_history = deque(maxlen=100)
        self.imu_raw_history = deque(maxlen=100)
        self.ukf_history = deque(maxlen=100)

        # Subscribe to sources
        rospy.Subscriber("/robot_0/odom", Odometry, self.encoder_callback)
        rospy.Subscriber("/robot_0/imu/yaw", Float32, self.imu_yaw_callback)
        rospy.Subscriber("/robot_0/imu/data", Imu, self.imu_raw_callback)
        rospy.Subscriber("/robot_0/odometry/filtered", Odometry, self.ukf_callback)

        rospy.loginfo("="*70)
        rospy.loginfo("YAW COMPARATOR - Finding the correct yaw source")
        rospy.loginfo("="*70)

        # Print comparison every second
        rospy.Timer(rospy.Duration(1.0), self.print_comparison)

    def encoder_callback(self, msg):
        """Encoder-based yaw (from differential drive)"""
        quat = msg.pose.pose.orientation
        euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        self.encoder_yaw = euler[2]
        self.encoder_history.append(euler[2])

    def imu_yaw_callback(self, msg):
        """IMU gyro yaw (processed by ESP)"""
        self.imu_yaw = msg.data
        self.imu_history.append(msg.data)

    def imu_raw_callback(self, msg):
        """IMU raw data from sensor_msgs/Imu"""
        quat = msg.orientation
        euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        self.imu_raw_yaw = euler[2]
        self.imu_raw_history.append(euler[2])

    def ukf_callback(self, msg):
        """UKF fused yaw"""
        quat = msg.pose.pose.orientation
        euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        self.ukf_yaw = euler[2]
        self.ukf_history.append(euler[2])

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def print_comparison(self, event):
        """Print yaw comparison with stability analysis"""
        if self.encoder_yaw is None or self.imu_yaw is None or self.ukf_yaw is None:
            print("Waiting for all data sources...")
            return

        encoder_deg = np.degrees(self.encoder_yaw)
        imu_deg = np.degrees(self.imu_yaw)
        imu_raw_deg = np.degrees(self.imu_raw_yaw) if self.imu_raw_yaw is not None else 0.0
        ukf_deg = np.degrees(self.ukf_yaw)

        # Calculate stability (standard deviation over last 100 samples)
        encoder_std = np.std(self.encoder_history) if len(self.encoder_history) > 10 else 0
        imu_std = np.std(self.imu_history) if len(self.imu_history) > 10 else 0
        imu_raw_std = np.std(self.imu_raw_history) if len(self.imu_raw_history) > 10 else 0
        ukf_std = np.std(self.ukf_history) if len(self.ukf_history) > 10 else 0

        # Calculate differences
        encoder_vs_imu = self.normalize_angle(self.encoder_yaw - self.imu_yaw)
        ukf_vs_imu = self.normalize_angle(self.ukf_yaw - self.imu_yaw)
        ukf_vs_encoder = self.normalize_angle(self.ukf_yaw - self.encoder_yaw)

        if self.imu_raw_yaw is not None:
            imu_vs_raw = self.normalize_angle(self.imu_yaw - self.imu_raw_yaw)

        print("\n" + "="*80)
        print("YAW COMPARISON (Robot stationary)")
        print("="*80)
        print(f"Source          | Current Value | Std Dev (noise) | Stability")
        print("-"*80)
        print(f"Encoder:        | {encoder_deg:7.2f}°     | {np.degrees(encoder_std):6.2f}°       | {'✓ Stable' if np.degrees(encoder_std) < 1 else '⚠️  NOISY'}")
        print(f"IMU (/imu/yaw): | {imu_deg:7.2f}°     | {np.degrees(imu_std):6.2f}°       | {'✓ Stable' if np.degrees(imu_std) < 1 else '⚠️  NOISY'}")
        if self.imu_raw_yaw is not None:
            print(f"IMU (raw):      | {imu_raw_deg:7.2f}°     | {np.degrees(imu_raw_std):6.2f}°       | {'✓ Stable' if np.degrees(imu_raw_std) < 1 else '⚠️  NOISY'}")
        print(f"UKF (fused):    | {ukf_deg:7.2f}°     | {np.degrees(ukf_std):6.2f}°       | {'✓ Stable' if np.degrees(ukf_std) < 1 else '⚠️  NOISY'}")
        print("-"*80)
        print(f"Encoder - IMU:         {np.degrees(encoder_vs_imu):7.2f}°")
        if self.imu_raw_yaw is not None:
            print(f"IMU - IMU(raw):        {np.degrees(imu_vs_raw):7.2f}°")
        print(f"UKF - IMU:             {np.degrees(ukf_vs_imu):7.2f}°")
        print(f"UKF - Encoder:         {np.degrees(ukf_vs_encoder):7.2f}°")
        print("-"*80)

        # Stability analysis
        if np.degrees(imu_std) > 5 or np.degrees(imu_raw_std) > 5:
            print("⚠️⚠️⚠️  IMU VERY NOISY (>5° std dev) - JUMPING!")
            print("   Possible causes:")
            print("   1. IMU sensor faulty/low quality")
            print("   2. Vibration/mechanical noise")
            print("   3. Magnetic interference (if magnetometer used)")
            print("   4. ESP processing issue")
        elif np.degrees(imu_std) > 2 or np.degrees(imu_raw_std) > 2:
            print("⚠️  IMU noisy (>2° std dev)")

        # Accuracy analysis
        if abs(np.degrees(encoder_vs_imu)) > 15:
            print("⚠️  ENCODER vs IMU differ by >15° → One of them is WRONG!")
        elif abs(np.degrees(encoder_vs_imu)) > 10:
            print("⚠️  ENCODER vs IMU differ by >10° → Encoder likely wrong (wheel slip?)")

        if abs(np.degrees(ukf_vs_imu)) < 5:
            print("✓ UKF trusts IMU more (good if IMU stable)")
        elif abs(np.degrees(ukf_vs_encoder)) < 5:
            print("⚠️  UKF trusts Encoder more (may be wrong if encoder drifted)")

        print("="*80)

        # Recommendation
        if abs(np.degrees(encoder_vs_imu)) > 15:
            print("\n💡 RECOMMENDATION:")
            print("   Encoder YAW is significantly different from IMU!")
            print("   Likely causes:")
            print("   1. Wheel slippage (encoder can't detect)")
            print("   2. Incorrect wheel base distance (ROBOT_WHEEL_DISTANCE)")
            print("   3. Uneven floor causing drift")
            print("")
            print("   FIX: Disable encoder YAW in UKF, trust IMU only:")
            print("   odom0_config: [true, true, false, false, false, FALSE, ...]")
            print("                                                    ^^^^^ disable encoder yaw")
            print("")

if __name__ == '__main__':
    try:
        comparator = YawComparator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
