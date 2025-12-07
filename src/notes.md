Bai toan robot phuc vu trong nha hang:
- Hien tai bi gioi han boi thiet lap kho khan
- Cac robot khong tranh duoc nhau
- Toc do cham

~~Chua co node dieu khien esp~~
Con mot so TODO va FIXME

~~Publish Odom~~
~~Chay model~~

Nghien cuu move_base/cancel


- cac thiet bi trung cau hinh

Sillicon labs: Lidar va esp moi -> lidar nhan truoc  ID_REVISION=0100
+ esp
+ lidar
QinHeng electronic: esp cu va imu -> esp moi nhan truoc


+ esp cu revision id = 0264 - QiHeng
+ imu revision id = 8233 - QiHeng
+ lidar revision id = 0100 - silicon labs
+ esp moi revision id = 0100 - silicon labs

à tôi hiểu rồi. để tôi mô tả cho bản hiểu. Hiện tại tôi đang cần dùng ukf để fusion các cảm biến. robot của tôi dùng thông tin của encoder để publish lên topic là robot_0/odom, sau đó tôi muốn ukf fusion giữa thông tin imu và encoder đó để publish lên topic /odom/filtered 
nên tôi đã config nó như sau
ukf_localization_node:
  frequency: 30  # Tần số cập nhật UKF
  sensor_timeout: 0.1
  two_d_mode: true  # Kích hoạt chế độ 2D, giả định rằng robot di chuyển trên mặt phẳng
  odom0: $(arg robot_namespace)/odom
  odom0_config: [true, true, false, false, false, true, true, true, false, false, false, true]
  imu0: $(arg robot_namespace)/imu/data
  imu0_config: [false, false, false, true, true, true, false, false, false, true, true, true]
  imu0_queue_size: 10
  imu0_remove_gravitational_acceleration: true
  odom_frame: $(arg robot_namespace)/odom
  base_link_frame: dummy_base_link
  world_frame: /map
  publish_tf: true

sudo ./src/catkin/bin/catkin_make_isolated --install --pkg rviz -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 --install-space /opt/ros/noetic 
rosdep install --from-paths ./src --ignore-packages-from-source --rosdistro noetic -y -r


## BUILD FROM SOURCE FOR ROS NOETIC IN UBUNTU 18.04
https://github.com/dnovischi/jetson-tutorials/blob/main/jetson-nano-ros-noetic-install.md 


### Build PyKDL
- PyKDL phai build from source vi khong tuong thich voi bionic. 
- https://github.com/orocos/orocos_kinematics_dynamics/tree/master/python_orocos_kdl/PyKDL
- Lam theo huong dan build trong INSTALL.md cua `orocos_kdl` truoc 
- Build `python_orocos_kdl` - required cmake 3.12
- Phai sua file cmake cua `python_orocos_kdl` thanh 3.10
- Build tiep tuc, neu loi thi track toi package nao bi sai phien ban roi cai dat dung phien ban yeu cau
- Buil thanh cong roi thi lam tiep theo huong dan o dau

### Build `rviz` package
- Rviz se bao cac loi lien quan toi `qOverride`. Dung chatgpt  de thay doi cho can sua 
- sudo ./src/catkin/bin/catkin_make_isolated --install --pkg rviz -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 --install-space /opt/ros/noetic  -j1
Chay lenh nay de chi build rieng rviz

## START UP
Remote to robot 
ssh khoixx@192.168.3.16

#### On server
<!-- source devel/setup.bash -->
export ROS_MASTER_URI=http://192.168.3.13:11311
export ROS_HOSTNAME=192.168.3.13
roscore
roslaunch robot_controller robot_navigation.launch
roslaunch robot_controller teleop.launch
roslaunch robot_controller robot_mapping.launch
roslaunch robot_controller master_navigation.launch num_robots:=1

roslaunch robot_controller cartographer_mapping.launch
roslaunch robot_controller cartographer_navigation.launch map_name:="case_test_10"

#### On robot
export ROS_MASTER_URI=http://192.168.3.13:11311
export ROS_HOSTNAME=192.168.3.16
roslaunch robot_controller robot_bringup.launch
