#include <Arduino.h>
#include <MotorController.h>

#define START_BYTE 0x02
#define END_BYTE 0x03
#define CMD_VEL 0x01
#define GET_SPEED 0x02
#define TUNE_PID 0x03

#define LEFT_WHEEL 0
#define RIGHT_WHEEL 1

#define ROBOT_WHEEL_DISTANCE 0.23

const int baudRate = 9600;
float leftWheelSpeed = 0.0;
float rightWheelSpeed = 0.0;

MotorController leftWheel(14, 27, 16, 17, 4, 15000, 0.033, 11.2, 4.8, 0.0, 50, 1.15);
MotorController rightWheel(26, 25, 1, 3, 22, 15000, 0.033, 11.2, 4.8, 0.0, 50, 1.15);

void setup() {
    Serial.begin(baudRate);
    Serial.println("Start serial");
}

uint8_t calculate_crc(const uint8_t *data, size_t length) {
    uint8_t crc = 0;
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
    }
    return crc;
}

void processSerialInput() {
    int numBytes = Serial.available();
    if (numBytes >= 3) {
        char packet[numBytes];
        String serialData = Serial.readString();
        serialData.trim();
        if (serialData.startsWith("S") && serialData.endsWith("E")) {
            String message = serialData.substring(1, -1);
        }
    }
}

void send_packet(uint8_t command_type, const uint8_t *data, size_t length) {
    Serial.write(START_BYTE);
    Serial.write(command_type);
    Serial.write(data, length);
    uint8_t crc = calculate_crc(data, length);
    Serial.write(crc);
    Serial.write(END_BYTE);
}

void loop() {
    leftWheel.runPID();
    rightWheel.runPID();
    leftWheelSpeed = leftWheel.getCurrentSpeed();
    rightWheelSpeed = rightWheel.getCurrentSpeed();
    Serial.print("speed: ");
    Serial.print(leftWheelSpeed);
    Serial.print(", output: ");
    Serial.println(leftWheel.getOutput());
    if (Serial.available() > 0) {
        String inputString = Serial.readStringUntil('\n');
        if (inputString.startsWith("Setpoint=")) {
            float setpoint = inputString.substring(9).toFloat();
            leftWheel.setSpeed(setpoint);
            Serial.print("Setpoint (m/s) updated to: ");
            Serial.println(setpoint);
        }
    }
    delay(50);
}