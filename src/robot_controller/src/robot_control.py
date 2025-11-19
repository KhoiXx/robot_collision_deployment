import struct
import tf
import rospy
import serial
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from tf.transformations import quaternion_from_euler
import numpy as np
from queue import PriorityQueue, Empty

from settings import *
from threading import Thread


class RobotControl:
    def __init__(self, index, port=ROBOT_PORT, baudrate=BAUDRATE) -> None:
        self.queue = PriorityQueue(maxsize=3)
        self.port = serial.Serial(port, baudrate, timeout=1)
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

        self.left_vel = 0.0
        self.right_vel = 0.0
        # rospy.loginfo("Robot")
        rospy.Timer(rospy.Duration(0.05), callback=self.update_odometry)  # 20Hz odometry update
        self.odom_broadcaster = tf.TransformBroadcaster()
        
        Thread(target=self.serial_worker, daemon=True).start()

    def serial_worker(self):
        while not rospy.is_shutdown():
            try:
                _, (command_type, format, values) = self.queue.get_nowait()
                self._send_command(command_type, format, *values)
            except Empty:
                continue

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

    def clear_buffer(self, type: str) -> None:
        if type == "all":
            self.port.reset_input_buffer()
            self.port.reset_output_buffer()
        elif type == "input":
            self.port.reset_input_buffer()
        elif type == "output":
            self.port.reset_output_buffer()

    def calculate_crc(self, data):
        crc = 0
        for byte in data:
            crc ^= byte
        return crc
    
    def send_command(self, command_type, format, *values, priority=1):
        """
        Add item to the priority queue in this format:
        tuple: (priority, item)
        """
        self.queue.put_nowait((priority, (command_type, format, values)))
        # print(self.queue.)

    def _send_command(self, command_type, format, *values):
        """Send command to the ESP"""
        self.clear_buffer("output")
        frame_length = len(values) * 4 + 1
        header = struct.pack('BBB', START_BYTE, command_type, frame_length)
        send_data = header
        if frame_length > 1:
            data = struct.pack(format, *values)
            send_data += data
        crc = self.calculate_crc(send_data)
        send_frame = send_data + struct.pack('B', crc)
        # rospy.loginfo(f"Sent frame: {send_frame}")
        self.port.reset_output_buffer()
        self.port.cancel_write()
        self.port.write(send_frame)
        self.port.flush()

    def request_speed(self):
        self.clear_buffer("input")
        self.send_command(GET_SPEED, None, priority=3)
        # self.port.flush()

    def read_serial(self):
        num_bytes = self.port.in_waiting
        if num_bytes >= 1:
            datatype = self.port.read(1)
            if datatype == b'S':
                received_string = self.port.readline().decode().strip()
                return received_string
                rospy.loginfo(f"Received string from serial: {received_string}")
            elif datatype == b'B':
                received_data = self.port.read(num_bytes - 1)
                if received_data[0] == GET_SPEED:
                    received_data = received_data[:10]
                    crc = received_data[-1]
                    left_vel, right_vel = struct.unpack('<ff', received_data[1:-1])
                    self.left_vel = round(left_vel, 4)
                    self.right_vel = round(right_vel, 4)
                    # rospy.loginfo(f"Left vel: {self.left_vel}, Right vel: {self.right_vel}")  # Comment to reduce spam


    def update_odometry(self, timer):
        # self.current_time = rospy.Time.now()
        # print(f"Current: {self.current_time}")
        # print(f"Last time: {self.last_time_get_speed}")
        # print((self.current_time - self.last_time_get_speed).to_sec())
        # self.read_serial()
        # if (self.current_time - self.last_time_get_speed).to_sec() > 0.1:
        # self.read_serial()
        self.request_speed()
        rospy.sleep(0.02)
        self.read_serial()
        # self.last_time_get_speed = self.current_time


        # Check if cmd_vel stopped - FORCE zero if no command for > 0.3s
        time_since_cmd = (rospy.Time.now() - self.last_cmd_vel_time).to_sec()
        CMD_TIMEOUT = 0.3  # seconds

        if time_since_cmd > CMD_TIMEOUT or (abs(self.last_commanded_v) < 0.01 and abs(self.last_commanded_w) < 0.01):
            # No command or zero command → FORCE velocities to 0
            linear_vel = 0.0
            angular_vel = 0.0
            # rospy.loginfo_throttle(1.0, "Robot stationary - forcing velocity = 0")
        else:
            # Normal operation - calculate from encoders
            linear_vel = (self.right_vel + self.left_vel) / 2
            angular_vel = (self.right_vel - self.left_vel) / ROBOT_WHEEL_DISTANCE

            # DEADBAND: Lọc nhiễu encoder khi đứng im
            VEL_DEADBAND = 0.02  # m/s (2 cm/s) - dưới ngưỡng này = 0
            ANGVEL_DEADBAND = 0.05  # rad/s (~3 độ/s)

            # Debug: Log raw encoder values (only when moving)
            if abs(linear_vel) > VEL_DEADBAND or abs(angular_vel) > ANGVEL_DEADBAND:
                rospy.loginfo_throttle(2.0, f"Moving - Left: {self.left_vel:.4f}, Right: {self.right_vel:.4f}")

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
        q = quaternion_from_euler(0, 0, self.current_theta)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        # Thiết lập vận tốc
        odom.twist.twist.linear.x = linear_vel
        odom.twist.twist.angular.z = angular_vel

        # CRITICAL: Set covariance matrices to prevent UKF NaN errors
        # Pose covariance (x, y, z, rotation about X, Y, Z) - 6x6 = 36 elements
        # Wheel odometry is good for position but orientation drifts
        odom.pose.covariance = [
            0.001, 0,     0,     0,     0,     0,      # x variance = 0.001 m^2
            0,     0.001, 0,     0,     0,     0,      # y variance = 0.001 m^2
            0,     0,     1e6,   0,     0,     0,      # z variance = large (not measured)
            0,     0,     0,     1e6,   0,     0,      # roll variance = large (not measured)
            0,     0,     0,     0,     1e6,   0,      # pitch variance = large (not measured)
            0,     0,     0,     0,     0,     0.05    # yaw variance = 0.05 rad^2 (drifts over time)
        ]

        # Twist covariance (vx, vy, vz, vroll, vpitch, vyaw) - 6x6 = 36 elements
        # ĐỘNG: Khi velocity = 0, tăng covariance
        is_stationary = (abs(linear_vel) < 0.01 and abs(angular_vel) < 0.02)
        vel_cov = 0.05 if is_stationary else 0.001  # Tăng covariance khi đứng im
        angvel_cov = 0.1 if is_stationary else 0.01

        odom.twist.covariance = [
            vel_cov, 0,       0,     0,     0,     0,         # vx variance
            0,       vel_cov, 0,     0,     0,     0,         # vy variance
            0,       0,       1e6,   0,     0,     0,         # vz variance = large (not measured)
            0,       0,       0,     1e6,   0,     0,         # vroll variance = large (not measured)
            0,       0,       0,     0,     1e6,   0,         # vpitch variance = large (not measured)
            0,       0,       0,     0,     0,     angvel_cov # vyaw variance
        ]

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
