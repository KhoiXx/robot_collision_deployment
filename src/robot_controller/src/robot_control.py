import struct
import tf
import rospy
import serial
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import numpy as np
from collections import deque
from math import sin, cos

from settings import *
from threading import Thread, Lock


class RobotControl:
    def __init__(self, index, port=ROBOT_PORT, baudrate=BAUDRATE) -> None:
        # OPTIMIZATION: Replace PriorityQueue with simple deque (faster for our use case)
        self.cmd_queue = deque(maxlen=3)
        self.queue_lock = Lock()

        self.port = serial.Serial(port, baudrate, timeout=0.01)  # Non-blocking timeout
        rospy.sleep(0.2)
        if self.port.is_open:
            rospy.loginfo(f"Connected to port {port}")
        self._index = index

        # subscribe to command vel topic to get the velocity command
        rospy.Subscriber(f'/robot_{index}/cmd_vel', Twist, self.cmd_vel_callback)
        # update odom topic
        self.odom_pub = rospy.Publisher(f'/robot_{index}/odom', Odometry, queue_size=10)
        # self.imu_sub = rospy.Subscriber(f'/robot_{index}/imu/data', Imu, self.imu_callback)
        self.imu_yaw_sub = rospy.Subscriber(f'/robot_{index}/imu/yaw', Float32, self.imu_yaw_callback)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.imu_yaw = 0.0
        self.imu_yaw_start = None
        self.last_time = rospy.Time.now()
        self.last_time_get_speed = rospy.Time.now()

        # Yaw correction for straight motion drift prevention
        self.yaw_correction_active = False
        self.yaw_at_straight_start = 0.0
        self.last_commanded_v = 0.0
        self.last_commanded_w = 0.0
        self.last_cmd_vel_time = rospy.Time.now()  # Track when last cmd_vel received

        self.imu_vx = 0.0
        self.imu_vy = 0.0
        self.imu_x = 0.0
        self.imu_y = 0.0
        self.last_imu_time = rospy.Time.now()

        # ACCURACY: Thread-safe velocity variables with lock
        self.vel_lock = Lock()
        self.left_vel = 0.0
        self.right_vel = 0.0

        # OPTIMIZATION: Pre-allocate serial send buffer (max 11 bytes for CMD_VEL)
        self.send_buffer = bytearray(11)

        # ACCURACY: Cache odometry covariance matrices to avoid recreation
        self._init_covariance_matrices()

        # rospy.loginfo("Robot")
        rospy.Timer(rospy.Duration(0.05), callback=self.update_odometry)  # 20Hz odometry update
        self.odom_broadcaster = tf.TransformBroadcaster()

        Thread(target=self.serial_worker, daemon=True).start()

    def _init_covariance_matrices(self):
        """OPTIMIZATION: Pre-calculate covariance matrices to avoid runtime recreation"""
        # Pose covariance for moving state
        self.pose_cov_moving = [
            0.001, 0,     0,     0,     0,     0,
            0,     0.001, 0,     0,     0,     0,
            0,     0,     1e6,   0,     0,     0,
            0,     0,     0,     1e6,   0,     0,
            0,     0,     0,     0,     1e6,   0,
            0,     0,     0,     0,     0,     0.05
        ]

        # Twist covariance for moving (low uncertainty)
        self.twist_cov_moving = [
            0.001, 0,     0,     0,     0,     0,
            0,     0.001, 0,     0,     0,     0,
            0,     0,     1e6,   0,     0,     0,
            0,     0,     0,     1e6,   0,     0,
            0,     0,     0,     0,     1e6,   0,
            0,     0,     0,     0,     0,     0.01
        ]

        # Twist covariance for stationary (high uncertainty)
        self.twist_cov_stationary = [
            0.05,  0,     0,     0,     0,     0,
            0,     0.05,  0,     0,     0,     0,
            0,     0,     1e6,   0,     0,     0,
            0,     0,     0,     1e6,   0,     0,
            0,     0,     0,     0,     1e6,   0,
            0,     0,     0,     0,     0,     0.1
        ]

    def serial_worker(self):
        """OPTIMIZATION: Non-blocking queue processing with sleep to avoid busy-wait"""
        while not rospy.is_shutdown():
            with self.queue_lock:
                if len(self.cmd_queue) > 0:
                    command_type, format_str, values = self.cmd_queue.popleft()
                    self._send_command(command_type, format_str, *values)
                else:
                    # OPTIMIZATION: Sleep when queue empty to reduce CPU usage
                    rospy.sleep(0.001)  # 1ms sleep instead of busy loop

    def cmd_vel_callback(self, cmd: Twist):
        v = cmd.linear.x
        w = cmd.angular.z

        # ==================================================================
        # LAYER 2: IMU Yaw Correction for Straight Motion Drift
        # ==================================================================

        abs_w = abs(w)
        abs_v = abs(v)

        # Detect if we're attempting straight motion
        if abs_w < 0.05 and abs_v > 0.1:  # Straight motion threshold
            # Starting straight motion - record initial yaw
            if not self.yaw_correction_active:
                self.yaw_correction_active = True
                self.yaw_at_straight_start = self.imu_yaw

            # Calculate yaw drift
            yaw_drift = self.imu_yaw - self.yaw_at_straight_start

            # Only correct if drift exceeds threshold (0.1 rad = ~5.7 degrees)
            if abs(yaw_drift) > 0.1:
                # Apply conservative correction
                KP_YAW = 0.4
                w_correction = -KP_YAW * yaw_drift

                # Limit max correction to avoid instability
                MAX_CORRECTION = 0.15  # rad/s
                if w_correction > MAX_CORRECTION:
                    w_correction = MAX_CORRECTION
                elif w_correction < -MAX_CORRECTION:
                    w_correction = -MAX_CORRECTION

                # Apply correction
                w += w_correction

                # rospy.loginfo_throttle(1.0, f"Yaw correction: drift={yaw_drift:.3f}, w_corr={w_correction:.3f}")
        else:
            # Not straight motion - disable correction
            self.yaw_correction_active = False

        # Store commanded values
        self.last_commanded_v = v
        self.last_commanded_w = w
        self.last_cmd_vel_time = rospy.Time.now()  # Update timestamp

        # rospy.loginfo(f"Sending command, linear: {v}, spin: {w}")
        self.send_command(
            CMD_VEL,
            "<ff",
            v,
            w,
            priority=1
        )

    def imu_yaw_callback(self, msg: Float32):
        if self.imu_yaw_start is None:
            self.imu_yaw_start = msg.data
        self.imu_yaw = msg.data - self.imu_yaw_start
    # def imu_callback(self, msg: Imu):
    #     now = rospy.Time.now()
    #     dt = (now - self.last_imu_time).to_sec()
    #     self.last_imu_time = now
    #     # q = msg.orientation
    #     # _, _, yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
    #     # self.imu_yaw = yaw

    #     # Tích phân ra vận tốc
    #     self.imu_vx += msg.linear_acceleration.x * dt
    #     self.imu_vy += msg.linear_acceleration.y * dt

    #     # Tích phân ra vị trí
    #     self.imu_x += self.imu_vx * dt
    #     self.imu_y += self.imu_vy * dt

    def calculate_crc(self, data):
        """Calculate XOR checksum"""
        crc = 0
        for byte in data:
            crc ^= byte
        return crc

    def send_command(self, command_type, format_str, *values):
        """
        OPTIMIZATION: Add command to deque (thread-safe)
        """
        with self.queue_lock:
            self.cmd_queue.append((command_type, format_str, values))

    def _send_command(self, command_type, format_str, *values):
        """
        OPTIMIZATION: Send command using pre-allocated buffer
        Removed redundant buffer clears, cancel_write, and flush
        """
        frame_length = len(values) * 4 + 1

        # Build frame in pre-allocated buffer
        self.send_buffer[0] = START_BYTE
        self.send_buffer[1] = command_type
        self.send_buffer[2] = frame_length

        offset = 3
        if frame_length > 1 and format_str:
            # Pack data directly into buffer
            data = struct.pack(format_str, *values)
            self.send_buffer[offset:offset+len(data)] = data
            offset += len(data)

        # Calculate CRC on the frame
        crc = self.calculate_crc(self.send_buffer[:offset])
        self.send_buffer[offset] = crc
        offset += 1

        # OPTIMIZATION: Single write, no flush needed for small packets
        self.port.write(self.send_buffer[:offset])

    def request_speed(self):
        """OPTIMIZATION: Removed input buffer clear - not needed with proper packet parsing"""
        self.send_command(GET_SPEED, None)

    def read_serial(self):
        """ACCURACY: Thread-safe serial read with proper locking"""
        num_bytes = self.port.in_waiting
        if num_bytes >= 1:
            datatype = self.port.read(1)
            if datatype == b'S':
                received_string = self.port.readline().decode().strip()
                return received_string
                # rospy.loginfo(f"Received string from serial: {received_string}")
            elif datatype == b'B':
                received_data = self.port.read(num_bytes - 1)
                if len(received_data) > 0 and received_data[0] == GET_SPEED:
                    if len(received_data) >= 10:
                        received_data = received_data[:10]
                        crc = received_data[-1]
                        left_vel, right_vel = struct.unpack('<ff', received_data[1:-1])

                        # ACCURACY: Thread-safe update with lock
                        with self.vel_lock:
                            self.left_vel = round(left_vel, 4)
                            self.right_vel = round(right_vel, 4)
                        # rospy.loginfo(f"Left vel: {self.left_vel}, Right vel: {self.right_vel}")  # Comment to reduce spam


    def update_odometry(self, timer):
        """
        OPTIMIZATION: Removed blocking sleep - use non-blocking serial read
        Request speed and immediately try to read (non-blocking)
        """
        self.request_speed()
        self.read_serial()  # Non-blocking with 0.01s timeout

        # Check if cmd_vel stopped - FORCE zero if no command for > 0.3s
        current_time = rospy.Time.now()
        time_since_cmd = (current_time - self.last_cmd_vel_time).to_sec()
        CMD_TIMEOUT = 0.3  # seconds

        # ACCURACY: Thread-safe velocity read
        with self.vel_lock:
            left_vel_local = self.left_vel
            right_vel_local = self.right_vel

        if time_since_cmd > CMD_TIMEOUT or (abs(self.last_commanded_v) < 0.01 and abs(self.last_commanded_w) < 0.01):
            # No command or zero command → FORCE velocities to 0
            linear_vel = 0.0
            angular_vel = 0.0
            # rospy.loginfo_throttle(1.0, "Robot stationary - forcing velocity = 0")
        else:
            # Normal operation - calculate from encoders
            linear_vel = (right_vel_local + left_vel_local) / 2
            angular_vel = (right_vel_local - left_vel_local) / ROBOT_WHEEL_DISTANCE

            # DEADBAND: Lọc nhiễu encoder khi đứng im
            VEL_DEADBAND = 0.02  # m/s (2 cm/s) - dưới ngưỡng này = 0
            ANGVEL_DEADBAND = 0.05  # rad/s (~3 độ/s)

            # Debug: Log raw encoder values (only when moving) - REMOVED f-string for performance
            # if abs(linear_vel) > VEL_DEADBAND or abs(angular_vel) > ANGVEL_DEADBAND:
            #     rospy.loginfo_throttle(2.0, f"Moving - Left: {left_vel_local:.4f}, Right: {right_vel_local:.4f}")

            if abs(linear_vel) < VEL_DEADBAND:
                linear_vel = 0.0
            if abs(angular_vel) < ANGVEL_DEADBAND:
                angular_vel = 0.0

        # Nội suy vị trí robot
        current_time = rospy.Time.now()
        self.current_time = current_time
        dt = (current_time - self.last_time).to_sec()

        # CHỈ tích phân khi có chuyển động thực sự
        if abs(linear_vel) > 0.0 or abs(angular_vel) > 0.0:
            delta_x = linear_vel * dt * np.cos(self.current_theta)
            delta_y = linear_vel * dt * np.sin(self.current_theta)
            delta_theta = angular_vel * dt

            # Cập nhật vị trí và hướng của robot
            self.current_x += delta_x
            self.current_y += delta_y
            self.current_theta += delta_theta
        # else: KHÔNG cập nhật gì cả khi đứng im

        # rospy.loginfo(f"Encoder position: x: {self.current_x}, y: {self.current_y};")
        # rospy.loginfo(f"IMU position: x: {self.imu_x}, y: {self.imu_y};")

        self.last_time = current_time

        # Publish dữ liệu odometry
        odom = Odometry()
        odom.header = Header()
        odom.header.stamp = current_time
        odom.header.frame_id = "robot_0/odom"
        odom.child_frame_id = "dummy_base_link"

        # Thiết lập vị trí
        odom.pose.pose.position.x = self.current_x
        odom.pose.pose.position.y = self.current_y
        odom.pose.pose.position.z = 0.0

        # OPTIMIZATION: Fast 2D quaternion (yaw-only rotation) instead of tf.transformations
        half_yaw = self.current_theta * 0.5
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = sin(half_yaw)
        odom.pose.pose.orientation.w = cos(half_yaw)

        # Thiết lập vận tốc
        odom.twist.twist.linear.x = linear_vel
        odom.twist.twist.angular.z = angular_vel

        # OPTIMIZATION: Use pre-calculated covariance matrices
        odom.pose.covariance = self.pose_cov_moving

        # Select twist covariance based on motion state
        is_stationary = (abs(linear_vel) < 0.01 and abs(angular_vel) < 0.02)
        odom.twist.covariance = self.twist_cov_stationary if is_stationary else self.twist_cov_moving

        # Publish odom
        self.odom_pub.publish(odom)

        # BỎQUA: Dùng static TF từ robot_mapping.launch thay vì publish động
        # Lý do: Tránh jump khi robot di chuyển
        # self.odom_broadcaster.sendTransform(
        #     (self.current_x, self.current_y, 0.0),
        #     q,
        #     self.current_time,
        #     "dummy_base_link",
        #     "/robot_0/odom",
        # )
