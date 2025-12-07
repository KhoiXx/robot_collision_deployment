#include <ESP32Encoder.h>
#include <PID_v2.h>

#define CLK 14       // Chân CLK của encoder
#define DT 27        // Chân DT của encoder
#define IN1 17       // Chân điều khiển động cơ IN1
#define IN2 16       // Chân điều khiển động cơ IN2
#define ENA 5        // Chân PWM động cơ

const int pulsesPerRevolution = 15000;   // Số xung mỗi vòng quay
const float wheelRadius = 0.0342;         // Bán kính bánh xe (m)
const float wheelCircumference = 2 * 3.14159 * wheelRadius;  // Chu vi bánh xe (m)

// Biến PID
double setpoint = 0.0, input = 0.0, output = 0.0;
double kp = 1.0, ki = 0.0, kd = 0.0;  // Giá trị khởi tạo PID

// Khởi tạo đối tượng PID
PID_v2 myPID(kp, ki, kd, PID::Direct);

// Khởi tạo encoder
ESP32Encoder encoder;
long previousPosition = 0;
unsigned long previousTime = 0;

// Hàm khởi tạo
void setup() {
  Serial.begin(115200);
  encoder.attachHalfQuad(DT, CLK);
  encoder.setCount(0);

  // Thiết lập động cơ
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  // Cấu hình PID
  myPID.Start(input, output, setpoint);
  // myPID.setOutputLimits(-255, 255);      // Giới hạn đầu ra cho PWM
  myPID.SetSampleTime(50);              // Chu kỳ lấy mẫu PID (ms)
}

float map_data(float x, float in_min, float in_max, float out_min, float out_max) {
  const float run = in_max - in_min;
  if (run == 0.0) {
    log_e("map(): Invalid input range, min == max");
    return -1;  // AVR returns -1, SAM returns 0
  }
  const float rise = out_max - out_min;
  const float delta = x - in_min;
  return (delta * rise) / run + out_min;
}

// Hàm tính toán tốc độ hiện tại của encoder
float calculateSpeed() {
  unsigned long currentTime = millis();
  long currentPosition = encoder.getCount() / 2;
  
  // Tính thay đổi vị trí và thời gian
  long positionChange = currentPosition - previousPosition;
  unsigned long timeChange = currentTime - previousTime;

  float speed = 0.0;
  if (timeChange > 0) {
    float pulsesPerSecond = (float)positionChange / ((float)timeChange / 1000.0);
    speed = (pulsesPerSecond / pulsesPerRevolution) * wheelCircumference;
  }

  // Cập nhật giá trị cũ
  previousPosition = currentPosition;
  previousTime = currentTime;

  return speed;
}

// Hàm điều khiển động cơ
void controlMotor(int pwmValue) {
  if (pwmValue > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  analogWrite(ENA, abs(pwmValue));
}

// Hàm để xử lý các lệnh Serial
void processSerialInput() {
  if (Serial.available() > 0) {
    String inputString = Serial.readStringUntil('\n');
    inputString.trim();

    // Kiểm tra lệnh nhập vào
    if (inputString.startsWith("Kp=")) {
      kp = inputString.substring(3).toFloat();
      myPID.SetTunings(kp, ki, kd);
      Serial.print("Kp updated to: ");
      Serial.println(kp);
    } else if (inputString.startsWith("Ki=")) {
      ki = inputString.substring(3).toFloat();
      myPID.SetTunings(kp, ki, kd);
      Serial.print("Ki updated to: ");
      Serial.println(ki);
    } else if (inputString.startsWith("Kd=")) {
      kd = inputString.substring(3).toFloat();
      myPID.SetTunings(kp, ki, kd);
      Serial.print("Kd updated to: ");
      Serial.println(kd);
    } else if (inputString.startsWith("Setpoint=")) {
      setpoint = inputString.substring(9).toFloat();
      long setpoint_pulse = map_data(setpoint, -1.15, 1.15, -255.0, 255.0);
      myPID.Setpoint(setpoint_pulse);
      Serial.print("Setpoint (m/s) updated to: ");
      Serial.println(setpoint_pulse);
    }
  }
}


void loop() {
  // Xử lý các lệnh Serial để cập nhật thông số PID
  processSerialInput();

  // Tính tốc độ hiện tại từ encoder
  input = calculateSpeed();
  Serial.print("Current Speed (m/s): ");
  Serial.println(input);

  int input_pulse = map_data(input, -1.15, 1.15, -255, 255);

  // Tính toán đầu ra của PID
  output = myPID.Run(input_pulse);

  // Điều khiển động cơ dựa trên đầu ra PID
  Serial.println(output);
  controlMotor((int)output);

  delay(50);
}
