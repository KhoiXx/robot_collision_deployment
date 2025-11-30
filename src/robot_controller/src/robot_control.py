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
        # PRIORITY: Separate queues - CMD_VEL sent immediately, GET_SPEED can wait
        self.cmd_vel_queue = deque(maxlen=2)  # High priority - speed commands
        self.other_queue = deque(maxlen=3)    # Low priority - telemetry requests
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

        # OPTIMIZATION: Pre-allocate serial send buffer (max 32 bytes to be safe)
        self.send_buffer = bytearray(32)

        # ACCURACY: Cache odometry covariance matrices to avoid recreation
        self._init_covariance_matrices()

        # Initialize TF broadcaster BEFORE Timer (to avoid AttributeError)
        self.odom_broadcaster = tf.TransformBroadcaster()

        # rospy.loginfo("Robot")
        rospy.Timer(rospy.Duration(0.033), callback=self.update_odometry)  # 30Hz odometry update (balance CPU & responsiveness)

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

        # Pose covariance for stationary (high uncertainty - drift prone)
        self.pose_cov_stationary = [
            0.1,   0,     0,     0,     0,     0,
            0,     0.1,   0,     0,     0,     0,
            0,     0,     1e6,   0,     0,     0,
            0,     0,     0,     1e6,   0,     0,
            0,     0,     0,     0,     1e6,   0,
            0,     0,     0,     0,     0,     0.2
        ]

        # Twist covariance for moving (VERY low uncertainty - trust encoder highly)
        self.twist_cov_moving = [
            0.0001, 0,      0,     0,     0,     0,
            0,      0.0001, 0,     0,     0,     0,
            0,      0,      1e6,   0,     0,     0,
            0,      0,      0,     1e6,   0,     0,
            0,      0,      0,     0,     1e6,   0,
            0,      0,      0,     0,     0,     0.001
        ]

        # Twist covariance for stationary (low uncertainty)
        self.twist_cov_stationary = [
            0.001,  0,      0,     0,     0,     0,
            0,      0.001,  0,     0,     0,     0,
            0,      0,      1e6,   0,     0,     0,
            0,      0,      0,     1e6,   0,     0,
            0,      0,      0,     0,     1e6,   0,
            0,      0,      0,     0,     0,     0.01
        ]

    def serial_worker(self):
        """PRIORITY: Process CMD_VEL first (immediate), GET_SPEED second (can wait)"""
        while not rospy.is_shutdown():
            command_to_send = None

            with self.queue_lock:
                # PRIORITY 1: CMD_VEL - send immediately!
                if len(self.cmd_vel_queue) > 0:
                    command_to_send = self.cmd_vel_queue.popleft()
                # PRIORITY 2: GET_SPEED and others - only when no CMD_VEL pending
                elif len(self.other_queue) > 0:
                    command_to_send = self.other_queue.popleft()

            if command_to_send:
                command_type, format_str, values = command_to_send
                self._send_command(command_type, format_str, *values)
            else:
                # Sleep when both queues empty
                rospy.sleep(0.001)

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

        rospy.loginfo(f"Sending command, linear: {v}, spin: {w}")
        self.send_command(
            CMD_VEL,
            "<ff",
            v,
            w
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
        PRIORITY: CMD_VEL goes to high-priority queue, others to low-priority
        """
        with self.queue_lock:
            if command_type == CMD_VEL:
                # HIGH PRIORITY - send immediately, no waiting
                self.cmd_vel_queue.append((command_type, format_str, values))
            else:
                # LOW PRIORITY - GET_SPEED, etc. can wait
                self.other_queue.append((command_type, format_str, values))

    def _send_command(self, command_type, format_str, *values):
        """
        Send command to the ESP using pre-allocated buffer
        """
        # Calculate frame length
        if format_str and len(values) > 0:
            frame_length = len(values) * 4 + 1  # data + CRC
        else:
            frame_length = 1  # only CRC

        # Build header
        header = struct.pack('BBB', START_BYTE, command_type, frame_length)
        send_data = bytearray(header)

        # Add data if present
        if frame_length > 1 and format_str:
            data = struct.pack(format_str, *values)
            send_data += data

        # Add CRC
        crc = self.calculate_crc(send_data)
        send_data.append(crc)

        # Send
        self.port.write(send_data)

    def read_serial(self):
        """
        ROBUST PACKET PARSING: Handle all packet types correctly
        - 'S' prefix: String debug messages
        - 'B' prefix: Binary packets (SPEED, PID_DATA)
        """
        num_bytes = self.port.in_waiting
        if num_bytes < 1:
            return

        # Read packet type prefix
        datatype = self.port.read(1)

        if datatype == b'S':
            # String debug message - read until newline
            received_string = self.port.readline().decode().strip()
            return received_string

        elif datatype == b'B':
            # Binary packet - need to read command type to route correctly
            if num_bytes < 2:
                return  # Not enough data yet

            cmd_type_byte = self.port.read(1)
            if not cmd_type_byte:
                return

            cmd_type = cmd_type_byte[0]

            # Route to appropriate parser
            if cmd_type == GET_SPEED:
                self._parse_speed_packet()
            elif cmd_type == GET_PID_DATA:
                self._parse_pid_packet()
            else:
                # Unknown packet type - clear buffer to resync
                rospy.logwarn(f"Unknown packet type: 0x{cmd_type:02x}")
                self.port.reset_input_buffer()

    def _parse_speed_packet(self):
        """
        Parse SPEED packet: left_vel(4) + right_vel(4) + CRC(1) = 9 bytes
        Total packet: 'B' + GET_SPEED + 9 bytes = 11 bytes
        """
        if self.port.in_waiting < 9:
            return  # Not enough data

        data = self.port.read(9)
        if len(data) < 9:
            return

        # Extract velocities (skip CRC validation for performance)
        left_vel, right_vel = struct.unpack('<ff', data[0:8])

        # Thread-safe update
        with self.vel_lock:
            self.left_vel = round(left_vel, 4)
            self.right_vel = round(right_vel, 4)

    def _parse_pid_packet(self):
        """
        Parse PID_DATA packet: timestamp(4) + setpoint(4) + current(4) + error(4) + output(4) + CRC(1) = 21 bytes
        For now, just discard (only used for debugging)
        """
        if self.port.in_waiting >= 21:
            self.port.read(21)  # Discard PID data


    def update_odometry(self, timer):
        """
        OPTIMIZATION: Arduino auto-pushes speed at 50Hz - just read passively
        No more request-response - lower latency, fresher data
        """
        self.read_serial()  # Passive read - Arduino pushes automatically

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

        # OPTIMIZATION: Use pre-calculated covariance matrices based on motion state
        # is_stationary = (abs(linear_vel) < 0.01 and abs(angular_vel) < 0.02)
        # odom.pose.covariance = self.pose_cov_stationary if is_stationary else self.pose_cov_moving
        # odom.twist.covariance = self.twist_cov_stationary if is_stationary else self.twist_cov_moving

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
