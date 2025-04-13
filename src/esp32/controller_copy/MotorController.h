#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include <Arduino.h>
#include <ESP32Encoder.h>
#include <PID_v2.h>

class MotorController{
    public:
        MotorController(
            u_int8_t clkPin_, // CLK of encoder
            u_int8_t dtPin_,  // DT of encoder
            u_int8_t in1Pin_, // IN1 to H-bridge
            u_int8_t in2Pin_, // IN2 to H-bridge
            u_int8_t enPin_,  // PWM signal to H-bridge
            int pulsesPerRevolution_,
            float wheelRadius_,
            double Kp, double Ki, double Kd,
            int sampleTime,
            float maxSpeed_,
            PID::Direction dir
        );
        void setTunings(double Kp, double Ki, double Kd);
        void setSpeed(float speed);
        float calculateSpeed();
        void runPID();
        float getCurrentSpeed();
        long getEncoderPulse();
        int getOutput();
        double kp, ki, kd;
        float currentSpeed;
        int outputPwm;
        int currentSpeedPulse;
        PID::Direction direction;
        PID_v2 pid;
        
    private:
        ESP32Encoder encoder;
        u_int8_t clkPin; // CLK of encoder
        u_int8_t dtPin;  // DT of encoder
        u_int8_t in1Pin; // IN1 to H-bridge
        u_int8_t in2Pin; // IN2 to H-bridge
        u_int8_t enPin;  // PWM signal to H-bridge
        int pulsesPerRevolution;
        float wheelRadius;
        long prevPosition;
        unsigned long prevTime;
        const float wheelCircumference;
        float mapData(float x, float in_min, float in_max, float out_min, float out_max);
        void controlMotor(int pwmValue);
        float maxSpeed;
        int reverse;
        int reverseAtStart;
};

#endif