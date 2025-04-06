#include <MotorController.h>

MotorController::MotorController(
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
    int sampleTime;
) : pid(double Kp, double Ki, double Kd, PID::Direct),
    encoder(),
    clkPin(clkPin_),
    dtPin(dtPin_),
    in1Pin(in1Pin_),
    in2Pin(in2Pin_),
    enPin(enPin_),
    pulsesPerRevolution(pulsesPerRevolution_),
    wheelRadius(wheelRadius_), {
    pid.Start(0.0, 0.0, 0.0);
    pid.SetSampleTime(sampleTime);
}