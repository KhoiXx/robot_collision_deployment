#!/home/khoint/thesis/deployment/src/.venv/bin/python3

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

import pandas as pd
from datetime import datetime

log = []
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
    global setpoint
    wait = True
    print(msg)
    received_string = msg.data
    splited_list = received_string.split(',')
    command_type = int(splited_list[0])
    format = splited_list[1]
    values = []
    for i in splited_list[2:]:
        values.append(float(i))
    print(f"command: {splited_list[2]}")
    robot.send_command(command_type, format, *values)
    if command_type == CMD_VEL_RIGHT:
        setpoint = float(splited_list[2])
    # robot.send_command(4, "<ff", 0.2,0.01)

# Thiết lập vẽ đồ thị tốc độ bánh xe trái
# fig, ax = plt.subplots()
# x_data, y_data = [0], [0.0]
# line, = ax.plot(x_data, y_data, marker='o', linestyle='-', color='blue', markersize=4)
# ax.set_title('Real-time Left Wheel Velocity')
# ax.set_xlabel('Time (seconds)')
# ax.set_ylabel('Velocity (m/s)')
# ax.grid(True)

# start_time = time.time()

# def update_plot():
#     # Lấy giá trị tốc độ từ robot
#     if robot:
#         print("update plot")
#         speed = robot.right_vel
#         x_data.append(time.time() - start_time)
#         y_data.append(speed)

#         # Giới hạn số điểm vẽ trên đồ thị để tránh việc đồ thị quá tải
#         if len(x_data) > 2000:
#             x_data.pop(0)
#             y_data.pop(0)

#         line.set_xdata(x_data)
#         line.set_ydata(y_data)
#         ax.set_xlim(max(0, x_data[-1] - 10), x_data[-1] + 1)
#         y_margin = 0.1
#         y_min, y_max = min(y_data), max(y_data)
#         if y_min == y_max:
#             y_min -= y_margin
#             y_max += y_margin
#         else:
#             y_min -= abs(y_min) * y_margin
#             y_max += abs(y_max) * y_margin
#         ax.set_ylim(y_min, y_max)
#         # ax.relim()
#         # ax.autoscale_view()
#         fig.canvas.draw()
#         fig.canvas.flush_events()

import pyqtgraph as pg
from pyqtgraph.Qt import mkQApp, exec_

import time, sys


x_data, y_data, setpoint_data = [], [], []
start_time = time.time()


def setup_gui(robot):
    app = mkQApp()
    win = pg.GraphicsLayoutWidget(title="Realtime Plot")
    plot = win.addPlot(title="Right Wheel Speed")
    curve = plot.plot(pen='y')
    target_curve = plot.plot(pen=pg.mkPen('r', style=pg.QtCore.Qt.DashLine))  # setpoint

    win.show()

    def update():
        current_time = time.time() - start_time
        try:
            value = float(robot.right_vel)
        except Exception as e:
            rospy.logwarn(f"[GUI] robot.right_vel error: {e}")
            value = 0.0
        
        x_data.append(current_time)
        y_data.append(value)
        setpoint_data.append(setpoint)


        if len(x_data) > 250:
            x_data.pop(0)
            y_data.pop(0)
            setpoint_data.pop(0)

        curve.setData(x_data, y_data)
        target_curve.setData(x_data, setpoint_data)
        y_values = y_data + setpoint_data
        y_min = min(y_values)
        y_max = max(y_values)

        # Giãn trục nếu khoảng cách quá nhỏ
        min_range = 0.1  # hoặc 0.05 tùy bạn
        current_range = y_max - y_min

        if current_range < min_range:
            center = (y_max + y_min) / 2
            y_min = center - min_range / 2
            y_max = center + min_range / 2

        plot.setYRange(y_min, y_max)



    timer = pg.QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(50)
    exec_()

if __name__=='__main__':
    global robot
    global setpoint
    wait = False
    cfp = os.path.abspath(__file__)
    cwd = os.path.dirname(cfp)
    rospy.init_node("test_node")
    rospy.loginfo(f"Current working directory: {cwd}")
    robot = RobotControl(0, port="/dev/esp", baudrate=115200)
    setpoint = 0.0

    # threading.Thread(target=set_speed)
    rospy.Subscriber("/manual_command", String, lambda msg: command_callback(msg, robot))

    # plt.ion()  # Bật chế độ vẽ không chặn
    # plt.show()

    gui_thread = threading.Thread(target=setup_gui, args=(robot,), daemon=True)
    gui_thread.start()
    try:
        i = 0
        while not rospy.is_shutdown():
            # update_plot()

            # if wait:
            #     rospy.sleep(0.05)
            #     wait = False

            robot.request_speed()
            rospy.sleep(0.1)
            # try:
            output = robot.read_serial()
            # output1  = output.split(',')
            # print(output1)
            # current_time = rospy.Time.now()
            # dt = datetime.fromtimestamp(current_time.to_sec())
            # log.append({"timestamp": dt.strftime("%H:%M:%S"), "target_speed": setpoint, "measured_speed": output1[1], "pid_output": output1[2]})
            # i += 1
            # df = pd.DataFrame(log)
            # if i > 10:
            #     print("tocsv")
            #     df.to_csv("speed.csv", index=False)
            #     i=0
            # except Exception as e:
            #     print(e)
            #     continue

    except KeyboardInterrupt:
        rospy.loginfo("Node bị dừng bằng Ctrl + C")

    rospy.spin()
