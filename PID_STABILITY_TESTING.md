# PID Stability Testing Guide

Hướng dẫn chi tiết để test và chứng minh tính ổn định của hệ thống PID controller cho robot.

## Tổng quan

Hệ thống này cho phép bạn:
1. Thu thập dữ liệu PID real-time từ ESP32
2. Chạy các test tự động (step response, disturbance rejection)
3. Phân tích và visualize kết quả
4. Tạo báo cáo chứng minh tính ổn định

## Các file liên quan

### ESP32 Code
- `src/esp32/controller_copy/controller_copy.ino` - Thêm command GET_PID_DATA để log PID data
- `src/esp32/controller_copy/MotorController.cpp` - PID controller implementation

### ROS Nodes
- `src/robot_controller/src/pid_stability_test.py` - Node để chạy stability tests
- `src/robot_controller/src/analyze_pid_stability.py` - Script phân tích và vẽ đồ thị
- `src/robot_controller/src/settings.py` - Cấu hình commands

### Launch Files
- `src/robot_controller/launch/test.launch` - Launch file với mode PID testing

## Cách sử dụng

### Bước 1: Nạp code mới vào ESP32

```bash
# Compile và upload code trong Arduino IDE hoặc PlatformIO
# File: src/esp32/controller_copy/controller_copy.ino
```

### Bước 2: Chạy PID Stability Test

#### Test bánh trái:
```bash
roslaunch robot_controller test.launch test_mode:=pid_stability test_wheel:=left
```

#### Test bánh phải:
```bash
roslaunch robot_controller test.launch test_mode:=pid_stability test_wheel:=right
```

#### Tùy chỉnh output directory:
```bash
roslaunch robot_controller test.launch test_mode:=pid_stability test_wheel:=left output_dir:=/path/to/results
```

### Bước 3: Phân tích kết quả

Sau khi test chạy xong, bạn sẽ có file CSV trong thư mục `pid_test_results/`:
- `step_response_left_YYYYMMDD_HHMMSS.csv` - Kết quả step response test
- `disturbance_left_YYYYMMDD_HHMMSS.csv` - Kết quả disturbance rejection test

Chạy script phân tích:

```bash
cd src/robot_controller/src
python3 analyze_pid_stability.py ../pid_test_results/step_response_left_20241115_123456.csv
```

Kết quả:
- File `*_report.txt` - Báo cáo text với metrics chi tiết
- File `*_analysis.png` - Đồ thị phân tích toàn diện

## Các chỉ số ổn định

### 1. Step Response Metrics

- **Rise Time**: Thời gian để đáp ứng đạt từ 10% đến 90% setpoint
  - Tốt: < 0.5s
  - Chấp nhận được: 0.5s - 1.0s

- **Settling Time**: Thời gian để ổn định trong ±2% setpoint
  - Tốt: < 1.0s
  - Chấp nhận được: 1.0s - 2.0s

- **Overshoot**: Độ vượt quá setpoint (%)
  - Tốt: < 10%
  - Chấp nhận được: 10% - 20%
  - Kém: > 20%

- **Steady-State Error**: Sai số xác lập
  - Tốt: < 0.01
  - Chấp nhận được: < 0.05

### 2. Error Statistics

- **Mean Absolute Error (MAE)**: Trung bình sai số tuyệt đối
  - Excellent: < 5
  - Good: < 10
  - Acceptable: < 20

- **Standard Deviation**: Độ dao động của error
  - Excellent: < 10
  - Good: < 20
  - Acceptable: < 30

- **RMSE**: Root mean square error
  - Càng nhỏ càng tốt

## Các test được thực hiện

### 1. Step Response Test
Test đáp ứng với các mức tốc độ khác nhau:
- 0.2 m/s
- 0.4 m/s
- 0.6 m/s
- -0.3 m/s (chiều ngược)
- -0.5 m/s (chiều ngược)
- 0.0 m/s (dừng)

Mỗi step kéo dài 5 giây.

### 2. Disturbance Rejection Test
Giữ tốc độ cố định (0.5 m/s) trong 10 giây để quan sát:
- Khả năng duy trì setpoint
- Ảnh hưởng của nhiễu
- Dao động xung quanh setpoint

## Kết quả mẫu

Sau khi chạy, bạn sẽ có các đồ thị:

1. **Setpoint vs Actual** - So sánh tốc độ đặt và tốc độ thực
2. **Tracking Error** - Sai số theo thời gian
3. **Control Output** - Tín hiệu điều khiển PWM
4. **Error Distribution** - Phân bố sai số (histogram)
5. **Statistics Summary** - Bảng thống kê tổng hợp

## Chứng minh ổn định

Hệ thống được coi là **ổn định** khi:

1. ✓ Đáp ứng hội tụ về setpoint (không dao động vô hạn)
2. ✓ Settling time hữu hạn
3. ✓ Overshoot < 20%
4. ✓ Steady-state error nhỏ
5. ✓ Error distribution tập trung quanh 0
6. ✓ Không có dao động tăng dần theo thời gian

## Troubleshooting

### ESP32 không gửi dữ liệu
- Kiểm tra serial port: `/dev/esp`
- Kiểm tra baudrate: 115200
- Verify code đã được upload

### ROS node không chạy
```bash
# Kiểm tra node có executable permission
chmod +x src/robot_controller/src/pid_stability_test.py
chmod +x src/robot_controller/src/analyze_pid_stability.py

# Build workspace
catkin_make
source devel/setup.bash
```

### Không có dữ liệu trong CSV
- Kiểm tra ESP32 có nhận được command không (xem Serial Monitor)
- Verify serial connection
- Kiểm tra PID loop có đang chạy không

## Tùy chỉnh Test Parameters

Sửa trong `pid_stability_test.py`:

```python
# Thay đổi tốc độ test
target_speeds = [0.1, 0.3, 0.5, 0.7, -0.2, -0.4]

# Thay đổi thời gian mỗi step
duration_per_step = 3.0  # seconds

# Thay đổi sample rate
rospy.sleep(0.05)  # 20Hz -> 0.05s
```

## Lưu ý

1. **Đảm bảo robot được đặt ở nơi an toàn** khi chạy test
2. Motor sẽ quay thật, chuẩn bị sẵn sàng để dừng khẩn cấp (Ctrl+C)
3. Kiểm tra battery đủ để chạy test hoàn chỉnh
4. Kết quả tốt nhất khi robot không có tải (không tiếp đất)

## Ví dụ sử dụng cho luận văn

Bạn có thể dùng kết quả này để:

1. Đưa vào phần **Kết quả thực nghiệm**
2. Chứng minh hệ thống PID hoạt động ổn định
3. So sánh với lý thuyết (Lyapunov, Pole placement, etc.)
4. Đánh giá chất lượng điều khiển

## Liên hệ

Nếu có vấn đề, kiểm tra:
- Serial connection
- ROS topics: `rostopic list`
- Node status: `rosnode list`
