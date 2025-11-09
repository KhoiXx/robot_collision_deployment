#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

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
            double input,
            double output,
            double setpoint,
            double Kp, double Ki, double Kd,
            int sampleTime,
        )
        void setTunings(double Kp, double Ki, double Kd);
        void setSpeed(float speed);
    
    private:
        PID_v2 pid;
        ESP32Encoder encoder;
        u_int8_t clkPin; // CLK of encoder
        u_int8_t dtPin;  // DT of encoder
        u_int8_t in1Pin; // IN1 to H-bridge
        u_int8_t in2Pin; // IN2 to H-bridge
        u_int8_t enPin;  // PWM signal to H-bridge
        int pulsesPerRevolution;
        float wheelRadius;
};

#endif