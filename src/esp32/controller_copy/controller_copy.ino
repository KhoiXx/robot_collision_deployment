#include <Arduino.h>
#include <MotorController.h>

// #define START_BYTE 0x02
// #define END_BYTE 0x03
// #define CMD_VEL 0x01
// #define GET_SPEED 0x02
// #define TUNE_PID 0x03
#define CMD_VEL 1
#define CMD_VEL_LEFT 2
#define CMD_VEL_RIGHT 3
#define GET_SPEED 4
#define TUNE_PID_LEFT 5
#define TUNE_PID_RIGHT 6

#define LEFT_WHEEL 0
#define RIGHT_WHEEL 1

#define ROBOT_WHEEL_DISTANCE 0.23

unsigned long lastPIDUpdate = 0;
const unsigned long pidSampleTime = 20; 

const int baudRate = 115200;
float leftWheelSpeed = 0.0;
float rightWheelSpeed = 0.0;

MotorController leftWheel(14, 27, 16, 17, 4, 15000, 0.033, 11.6, 4.9, 0.04, pidSampleTime, 1.15, 1);
MotorController rightWheel(2, 15, 21, 22, 23, 15000, 0.033, 11.6, 4.9, 0.04, pidSampleTime, 1.15, -1);

void setup() {
    Serial.begin(baudRate);
    while(Serial.available() > 0) {
        char t = Serial.read();
    }
    Serial.println("Start serial");
}

uint8_t calculateCRC(const uint8_t *data, size_t length) {
    uint8_t crc = 0;
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
    }
    return crc;
}

void tunePID(String side, byte* data) {
    MotorController* selectWheel = side == "left" ? &leftWheel : &rightWheel;
    
    float receivedKp = *((float*)(data));        // Đọc 4 byte đầu
    float receivedKi = *((float*)(data + 4));    // Đọc 4 byte tiếp theo
    float receivedKd = *((float*)(data + 8));    // Đọc 4 byte cuối cùng

    const float _kp = receivedKp == -1.0 ? selectWheel->kp : receivedKp;
    const float _ki = receivedKi == -1.0 ? selectWheel->ki : receivedKi;
    const float _kd = receivedKd == -1.0 ? selectWheel->kd : receivedKd;
    selectWheel->setTunings(_kp,_ki,_kd);
    char buffer[50];
    snprintf(buffer, sizeof(buffer), "Kp:%.2f, Ki:%.2f, Kd:%.2f", _kp, _ki, _kd);
    
    Serial.print("S");
    Serial.println(buffer);
}

void sendSpeed() {
    leftWheelSpeed = leftWheel.getCurrentSpeed();
    rightWheelSpeed = rightWheel.getCurrentSpeed();
    uint8_t buffer[11];
    buffer[0] = 'B';

    buffer[1] = GET_SPEED; // length = len(data) + 1 (crc)
    memcpy(&buffer[2], &leftWheelSpeed, sizeof(float));
    memcpy(&buffer[6], &rightWheelSpeed, sizeof(float));
    uint8_t crc = calculateCRC(buffer, 10);
    buffer[10] = crc;

    Serial.write(buffer, sizeof(buffer));
    Serial.flush();
}

void processSerialInput() {
    float v, w, setLeftWheelSpeed, setRightWheelSpeed;
    int numBytes = Serial.available();
    if (numBytes >= 3) {
        byte commandCode = Serial.read();
        byte length = Serial.read();
        if (numBytes >= length) {
            byte data[20]; // without crc
            byte crcFromData = calculateCRC(&commandCode, 1) ^ calculateCRC(&length, 1);
            if ((length - 1) > 0) {
                Serial.readBytes(data, length - 1);
                crcFromData ^= calculateCRC(data, length - 1);
            } 
            byte crc = Serial.read();
            // Serial.print("S");
            // Serial.print(commandCode);
            // Serial.print(",");
            // Serial.print(length);
            // Serial.print(",");
            // Serial.print(crc);
            // Serial.print(",");
            // Serial.println(crcFromData);
            if (crcFromData == crc) {
                switch (commandCode)
                {
                case CMD_VEL:
                    v = *((float*) &data[0]);
                    w = *((float*) &data[4]);
                    setLeftWheelSpeed = v - (w * ROBOT_WHEEL_DISTANCE / 2.0);
                    leftWheel.setSpeed(setLeftWheelSpeed);
                    setRightWheelSpeed = v + (w * ROBOT_WHEEL_DISTANCE / 2.0);
                    rightWheel.setSpeed(setRightWheelSpeed);
                    break;
                case CMD_VEL_LEFT:
                    v = *((float*) &data[0]);
                    leftWheel.setSpeed(v);
                    break;
                case CMD_VEL_RIGHT:
                    v = *((float*) &data[0]);
                    rightWheel.setSpeed(v);
                    break;
                case TUNE_PID_LEFT:
                    tunePID("left", data);
                    break;
                case TUNE_PID_RIGHT:
                    tunePID("right", data);
                    break;
                case GET_SPEED:
                    sendSpeed();
                    break;
                default:
                    break;
                }
            }
        }
    }
    while(Serial.available() > 0) {
        char t = Serial.read();
    }
}


void loop() {
    unsigned long currentMillis = millis();
    if (currentMillis - lastPIDUpdate >= pidSampleTime) {
        lastPIDUpdate = currentMillis;

        // Run the PID control for both wheels
        leftWheel.runPID();
        rightWheel.runPID();
    }
    processSerialInput();
    // delay(50);
}