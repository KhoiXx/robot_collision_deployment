#include <Arduino.h>

#define START_BYTE 0x02
#define END_BYTE 0x03
#define CMD_VEL 0x01
#define REQ_ENCODER 0x02

const int baudRate = 115200;
float leftWheelSpeed = 0.0;
float rightWheelSpeed = 0.0;

void setup() {
    Serial.begin(baudRate);
}

uint8_t calculate_crc(const uint8_t *data, size_t length) {
    uint8_t crc = 0;
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
    }
    return crc;
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
    if (Serial.available() >= 10) {
        uint8_t packet[10];
        Serial.readBytes(packet, 10);

        if (packet[0] == START_BYTE && packet[9] == END_BYTE) {
            uint8_t command_type = packet[1];
            uint8_t crc = calculate_crc(packet + 1, 8);

            if (crc == packet[8]) {
                if (command_type == CMD_VEL) {
                    float x_vel, w_vel;
                    memcpy(&x_vel, packet + 2, 4);
                    memcpy(&w_vel, packet + 6, 4);

                    leftWheelSpeed = x_vel - (w_vel * WHEEL_BASE / 2.0);
                    rightWheelSpeed = x_vel + (w_vel * WHEEL_BASE / 2.0);
                } else if (command_type == REQ_ENCODER) {
                    uint8_t response[8];
                    memcpy(response, &leftWheelSpeed, 4);
                    memcpy(response + 4, &rightWheelSpeed, 4);
                    send_packet(command_type, response, 8);
                }
            }
        }
    }
}
