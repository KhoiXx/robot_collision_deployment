import struct
import tf
import rospy
import serial
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler
import numpy as np

from settings import *


class RobotControl:
    def __init__(self, index, port=ROBOT_PORT, baudrate=BAUDRATE) -> None:
        self.port = serial.Serial(port, baudrate, timeout=1)
        rospy.sleep(0.2)
        if self.port.is_open:
            rospy.loginfo(f"Connected to port {port}")
        self._index = index

        # subscribe to command vel topic to get the velocity command
        rospy.Subscriber(f'/robot_{index}/cmd_vel', Twist, self.cmd_vel_callback)
        # update odom topic
        self.odom_pub = rospy.Publisher(f'/robot_{index}/odom', Odometry, queue_size=10)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.last_time = rospy.Time.now()

        self.left_vel = 0.0
        self.right_vel = 0.0
        # rospy.loginfo("Robot")
        rospy.Timer(rospy.Duration(0.05), callback=self.update_odometry)  # 10Hz
        
        self.odom_broadcaster = tf.TransformBroadcaster()

    def cmd_vel_callback(self, cmd: Twist):
        v = cmd.linear.x
        w = cmd.angular.z
        rospy.loginfo(f"Sending command")
        self.send_command(
            CMD_VEL,
            "<ff",
            v,
            w,
        )

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

    def send_command(self, command_type, format, *values):
        self.clear_buffer("output")
        frame_length = len(values) * 4 + 1
        header = struct.pack('BB', command_type, frame_length)
        send_data = header
        if frame_length > 1:
            data = struct.pack(format, *values)
            send_data += data
            rospy.loginfo(f"Sent frame: {send_data}")
        crc = self.calculate_crc(send_data)
        send_frame = send_data + struct.pack('B', crc)
        self.port.write(send_frame)

    def request_speed(self):
        self.clear_buffer("input")
        self.send_command(GET_SPEED, None)

    def read_serial(self):
        num_bytes = self.port.in_waiting
        if num_bytes >= 1:
            datatype = self.port.read(1)
            if datatype == b'S':
                received_string = self.port.readline().decode().strip()
                rospy.loginfo(f"Received string from serial: {received_string}")
            elif datatype == b'B':
                received_data = self.port.read(num_bytes - 1)
                crc = received_data[-1]
                # print(f"Received frame: {received_data}")
                calculated_crc =  self.calculate_crc(b'B' + received_data[:-1])
                if crc == calculated_crc:
                    if received_data[0] == GET_SPEED:
                        left_vel, right_vel = struct.unpack('<ff', received_data[1:-1])
                        self.left_vel = round(left_vel, 4)
                        self.right_vel = round(right_vel, 4)
                        # rospy.loginfo(f"Left vel: {self.left_vel}, Right vel: {self.right_vel}")


    def update_odometry(self, timer):
        self.current_time = rospy.Time.now()
        self.request_speed()
        # rospy.sleep(0.04)

        self.read_serial()
        
        # Tính toán vận tốc dài và vận tốc góc của robot
        linear_vel = (self.right_vel + self.left_vel) / 2
        angular_vel = (self.right_vel - self.left_vel) / ROBOT_WHEEL_DISTANCE

        # Nội suy vị trí robot
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        delta_x = linear_vel * dt * np.cos(self.current_theta)
        delta_y = linear_vel * dt * np.sin(self.current_theta)
        delta_theta = angular_vel * dt

        # Cập nhật vị trí và hướng của robot
        self.current_x += delta_x
        self.current_y += delta_y
        self.current_theta += delta_theta
        self.last_time = current_time

        # Publish dữ liệu odometry
        odom = Odometry()
        odom.header = Header()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
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

        # Publish odom
        self.odom_pub.publish(odom)
        self.odom_broadcaster.sendTransform(
            (self.current_x, self.current_y, 0.0),
            q,
            self.current_time,
            f"dummy_base_link",
            "robot_0/odom",
        )
