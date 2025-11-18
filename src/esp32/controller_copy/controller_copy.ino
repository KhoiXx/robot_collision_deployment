#include <Arduino.h>
#include <MotorController.h>


enum ParseState { WAIT_START, WAIT_CMD, WAIT_LEN, WAIT_DATA, WAIT_CRC };
ParseState parseState = WAIT_START;

byte commandCode;
byte dataLength;
byte dataBuffer[32];
byte dataIndex = 0;
byte expectedCRC;

#define START_BYTE 0xAB
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
#define GET_PID_DATA 7

#define LEFT_WHEEL 0
#define RIGHT_WHEEL 1

#define ROBOT_WHEEL_DISTANCE 0.21

unsigned long lastPIDUpdate = 0;
const unsigned long pidSampleTime = 20; 

const int baudRate = 115200;
float leftWheelSpeed = 0.0;
float rightWheelSpeed = 0.0;

// Variables for wheel synchronization
float setLeftWheelSpeed = 0.0;   // Track commanded speed for sync logic
float setRightWheelSpeed = 0.0;
float currentAngularVel = 0.0;    // Angular velocity for mode detection

MotorController leftWheel(14, 27, 16, 17, 4, 15000, 0.033, 11.6, 5.2, 0.01, pidSampleTime, 1.15, DIRECT);
MotorController rightWheel(18, 19, 21, 22, 23, 15000, 0.033, 11.6, 5.2, 0.01, pidSampleTime, 1.15, REVERSE);


float Kpv(float set_vel){
    float abs_vel = abs(set_vel);
    if(abs_vel < 0.08){
        return 10.9;
    } else {
        return 16.8*abs_vel*abs_vel + 6.8*abs_vel + 11.1;
    }
}

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
    // Serial.print("S");
    // Serial.print(" Current speed: ");
    // Serial.println(leftWheelSpeed);
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

void sendPIDData(bool side) {
    MotorController* wheel = side ? &leftWheel : &rightWheel;

    unsigned long timestamp = millis();
    float setpoint = wheel->pid.GetSetpoint();
    float current = wheel->currentSpeedPulse;
    float error = setpoint - current;
    float output = wheel->getOutput();

    // NO DEBUG LOG - binary data only!

    // Packet structure: 'B' | CMD | timestamp(4) | setpoint(4) | current(4) | error(4) | output(4) | CRC
    uint8_t buffer[23];
    buffer[0] = 'B';
    buffer[1] = GET_PID_DATA;

    memcpy(&buffer[2], &timestamp, sizeof(unsigned long));
    memcpy(&buffer[6], &setpoint, sizeof(float));
    memcpy(&buffer[10], &current, sizeof(float));
    memcpy(&buffer[14], &error, sizeof(float));
    memcpy(&buffer[18], &output, sizeof(float));

    uint8_t crc = calculateCRC(buffer, 22);
    buffer[22] = crc;

    Serial.write(buffer, sizeof(buffer));
    Serial.flush();
}

void handleCommand(byte commandCode, byte* data, byte length) {
    float v, w;
    // Serial.print("S");
    // Serial.print(" Handling command: ");
    // Serial.println(commandCode);
    switch (commandCode)
    {
        case CMD_VEL:
            v = *((float*) &data[0]);
            w = *((float*) &data[4]);

            // Calculate wheel speeds from v and w
            setLeftWheelSpeed = v - (w * ROBOT_WHEEL_DISTANCE / 2.0);
            setRightWheelSpeed = v + (w * ROBOT_WHEEL_DISTANCE / 2.0);
            currentAngularVel = w;  // Store for sync logic

            // Set speeds to motors
            leftWheel.setSpeed(setLeftWheelSpeed, Kpv(setLeftWheelSpeed));
            rightWheel.setSpeed(setRightWheelSpeed, Kpv(setRightWheelSpeed));
            break;
        case CMD_VEL_LEFT:
            v = *((float*) &data[0]);
            leftWheel.setSpeed(v, Kpv(v));
            break;
        case CMD_VEL_RIGHT:
            v = *((float*) &data[0]);
            rightWheel.setSpeed(v, Kpv(v));
            break;
        case TUNE_PID_LEFT:
            tunePID("left", data);
            break;
        case TUNE_PID_RIGHT:
            tunePID("right", data);
            break;
        case GET_SPEED:
            // Serial.print("S");
            // Serial.print(" CurrentSpeed: ");
            // Serial.print(leftWheel.currentSpeed);
            // Serial.print("; leftSetpoint: ");
            // Serial.print(leftWheel.pid.GetSetpoint());
            // Serial.print("; currentSpeedPulse: ");
            // Serial.print(leftWheel.currentSpeedPulse);
            // Serial.print("; leftOutput: ");
            // Serial.println(leftWheel.getOutput());
            sendSpeed();
            break;
        case GET_PID_DATA:
            // data[0] is wheel selection: 0 = left, 1 = right
            // NO DEBUG LOG - binary response only!
            sendPIDData(*((float*) &data[0])== 0.0);
            break;
        default:
            break;
    }
}

void parseSerialOneByte() {
    if (Serial.available()) {
        byte incoming = Serial.read();
        // Serial.print("S");
        // Serial.print("State: ");
        // Serial.print(parseState);
        // Serial.print("; Read value: ");
        // Serial.print("\\x");
        // if (incoming < 16) Serial.print("0");
        // Serial.print(incoming, HEX);
        switch (parseState) {
            case WAIT_START:
                if (incoming == START_BYTE) {
                    parseState = WAIT_CMD;
                    expectedCRC = incoming;
                }
                break;

            case WAIT_CMD:
                commandCode = incoming;
                expectedCRC ^= incoming;
                parseState = WAIT_LEN;
                break;

            case WAIT_LEN:
                expectedCRC ^= incoming;
                dataLength = incoming - 1; // Remove CRC from length
                dataIndex = 0;
                parseState = (dataLength > 0) ? WAIT_DATA : WAIT_CRC;
                break;

            case WAIT_DATA:
                dataBuffer[dataIndex++] = incoming;
                expectedCRC ^= incoming;
                if (dataIndex >= dataLength) {
                    parseState = WAIT_CRC;
                }
                break;

            case WAIT_CRC:
                // Serial.print("; Expected CRC: ");
                // Serial.print("\\x");
                // if (expectedCRC < 16) Serial.print("0");
                // Serial.print(expectedCRC, HEX);
                if (incoming == expectedCRC) {
                    handleCommand(commandCode, dataBuffer, dataLength);
                }
                parseState = WAIT_START;
                break;
        }
    }
}


float value=0.0;
int pulse=0;
unsigned long start = millis();
bool first_time = true;
void loop() {
    unsigned long currentMillis = millis();
    if (currentMillis - lastPIDUpdate >= pidSampleTime) {
        lastPIDUpdate = currentMillis;

        // Run the PID control for both wheels
        leftWheel.runPID();
        rightWheel.runPID();

        // ==================================================================
        // WHEEL SYNCHRONIZATION: Cross-coupling to prevent straight drift
        // ==================================================================

        float absAngularVel = abs(currentAngularVel);

        // Determine motion mode and sync strength
        float Ksync = 0.0;
        const int MAX_SYNC_ADJUSTMENT = 25;  // Limit adjustment to ±25 PWM for stability

        if (absAngularVel < 0.05) {
            // Pure straight motion (forward/backward) - STRONG sync
            Ksync = 0.25;
        } else if (absAngularVel < 0.3) {
            // Mixed motion (curved path) - WEAK sync
            Ksync = 0.08;
        }
        // else: Pure rotation (w >= 0.3) - NO sync

        // Apply synchronization only if Ksync > 0
        if (Ksync > 0.0) {
            float leftSpeed = leftWheel.getCurrentSpeed();
            float rightSpeed = rightWheel.getCurrentSpeed();
            float speedError = leftSpeed - rightSpeed;

            // Calculate adjustment with limit
            int syncAdjustment = (int)(Ksync * speedError * 100.0);  // Scale for PWM range

            // Clamp adjustment to prevent oscillation
            if (syncAdjustment > MAX_SYNC_ADJUSTMENT) syncAdjustment = MAX_SYNC_ADJUSTMENT;
            if (syncAdjustment < -MAX_SYNC_ADJUSTMENT) syncAdjustment = -MAX_SYNC_ADJUSTMENT;

            // Apply cross-coupling: left slows down, right speeds up (or vice versa)
            leftWheel.adjustOutput(-syncAdjustment);
            rightWheel.adjustOutput(syncAdjustment);
        }
    }
    parseSerialOneByte();
    // delay(50);
    // if (Serial.available()) {
    //     value = Serial.parseFloat();
    //     pulse = leftWheel.mapData(value, -12, 12.0, -255, 255);
    //     leftWheel.controlMotor(pulse);
    //     if (first_time && value == 0.0) {
    //         Serial.println("time,voltage,pwm,theta");
    //         start = millis();
    //         first_time = false;
    //     }
    // }
    // unsigned long currentMillis = millis() - start;
    // if (!first_time){

    //     Serial.print(currentMillis);
    //     Serial.print(",");
    //     Serial.print(value);
    //     Serial.print(",");
    //     Serial.print(pulse);
    //     Serial.print(",");
    //     Serial.println(leftWheel.getEncoderPulse());
    // }
}