#!/usr/bin/env python3

import ast
import os
import queue
import struct
import threading
import time

import matplotlib.animation as animation  # type: ignore
import matplotlib.pyplot as plt  # type: ignore
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from model.net import CNNPolicy
from robot_control import RobotControl
from settings import *

data_queue = queue.Queue()


def set_speed():
    cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    while True:
        speed = input("Input speed")
        print(f"Current set speed: {speed}")
        data_queue.put(float(speed))
        if speed.lower() == "exit":
            break

def command_callback(msg: String, robot: RobotControl):
    global wait
    wait = True
    print(msg)
    received_string = msg.data
    splited_list = received_string.split(',')
    command_type = int(splited_list[0])
    format = splited_list[1]
    values = []
    for i in splited_list[2:]:
        values.append(float(i))
    robot.send_command(command_type, format, *values)

# Thiết lập vẽ đồ thị tốc độ bánh xe trái
fig, ax = plt.subplots()
x_data, y_data = [0], [0.0]
line, = ax.plot(x_data, y_data)
ax.set_title('Real-time Left Wheel Velocity')
ax.set_xlabel('Time (seconds)')
ax.set_ylabel('Velocity (m/s)')
start_time = time.time()

def update_plot():
    # Lấy giá trị tốc độ từ robot
    if robot:
        speed = robot.right_vel
        x_data.append(time.time() - start_time)
        y_data.append(speed)

        # Giới hạn số điểm vẽ trên đồ thị để tránh việc đồ thị quá tải
        if len(x_data) > 100:
            x_data.pop(0)
            y_data.pop(0)

        line.set_xdata(x_data)
        line.set_ydata(y_data)
        ax.relim()
        ax.autoscale_view()
        fig.canvas.draw()
        fig.canvas.flush_events()

if __name__=='__main__':
    wait = False
    cfp = os.path.abspath(__file__)
    cwd = os.path.dirname(cfp)
    rospy.init_node("test_node")
    rospy.loginfo(f"Current working directory: {cwd}")
    robot = RobotControl(0, port="/dev/esp", baudrate=115200)

    # threading.Thread(target=set_speed)
    rospy.Subscriber("/manual_command", String, lambda msg: command_callback(msg, robot))

    plt.ion()  # Bật chế độ vẽ không chặn
    plt.show()

    try:
        while not rospy.is_shutdown():
            update_plot()

            if wait:
                rospy.sleep(0.5)
                wait = False

            robot.request_speed()
            rospy.sleep(0.1)
            robot.read_serial()

    except KeyboardInterrupt:
        rospy.loginfo("Node bị dừng bằng Ctrl + C")

    rospy.spin()
