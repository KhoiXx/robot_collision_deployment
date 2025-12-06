#include <MotorController.h>

MotorController::MotorController(
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
) :
    pid(Kp, Ki, Kd, dir),
    encoder(),
    clkPin(clkPin_),
    dtPin(dtPin_),
    in1Pin(in1Pin_),
    in2Pin(in2Pin_),
    enPin(enPin_),
    pulsesPerRevolution(pulsesPerRevolution_),
    wheelRadius(wheelRadius_),
    prevPosition(0),
    prevTime(0),
    wheelCircumference(2 * 3.14159 * wheelRadius_),
    maxSpeed(maxSpeed_),
    currentSpeed(0.0)
{
    kp = Kp; ki = Ki; kd = Kd;
    pid.Start(0.0, 0.0, 0.0);
    pid.SetOutputLimits(-255,255);
    pid.SetSampleTime(sampleTime);
    encoder.attachHalfQuad(dtPin, clkPin);
    encoder.setCount(0);

    // Thiết lập chân của động cơ
    pinMode(in1Pin, OUTPUT);
    pinMode(in2Pin, OUTPUT);
    pinMode(enPin, OUTPUT);
    outputPwm = 0;
    currentSpeedPulse = 0;
    direction = dir;

    // OPTIMIZATION 1: Cache direction multiplier (calculated once instead of every loop)
    // DIRECT = 0 → directionMultiplier = 1
    // REVERSE = 1 → directionMultiplier = -1
    directionMultiplier = (dir == DIRECT) ? 1 : -1;

    // Initialize velocity ramping variables
    targetSpeed = 0.0;
    currentRampedSpeed = 0.0;
    maxAcceleration = 2.5;  // Default: 2.5 m/s^2 for smooth acceleration
    lastRampTime = millis();
}

void MotorController::setTunings(double Kp, double Ki, double Kd) {
    pid.SetTunings(Kp, Ki, Kd);
    kp = Kp; ki = Ki; kd = Kd;
}

void MotorController::setSpeed(float speed, float Kp) {
    if (Kp != -1.0f) {
        setTunings(Kp, ki, kd);
    }
    // Store target speed for ramping (don't set directly to PID)
    targetSpeed = speed;

    // If ramping is disabled (maxAcceleration <= 0), set speed immediately
    if (maxAcceleration <= 0.0) {
        currentRampedSpeed = speed;
        long setPulses = this->mapData(speed, -maxSpeed, maxSpeed, -255, 255);
        pid.Setpoint(setPulses);
    }
}

float MotorController::mapData(float x, float in_min, float in_max, float out_min, float out_max) {
    const float run = in_max - in_min;
    if (run == 0.0) {
        log_e("map(): Invalid input range, min == max");
        return -1;
    }
    const float rise = out_max - out_min;
    const float delta = x - in_min;
    return (delta * rise) / run + out_min;
}

float MotorController::calculateSpeed() {
    unsigned long currentTime = millis();
    // OPTIMIZATION 1: Use cached directionMultiplier instead of calculating (1-2*int(direction))
    long currentPosition = directionMultiplier * encoder.getCount() / 2;

    // Tính thay đổi vị trí và thời gian
    long positionChange = currentPosition - prevPosition;
    unsigned long timeChange = currentTime - prevTime;

    float speed = 0.0;
    if (timeChange > 0) {
        // OPTIMIZATION: Use multiplication instead of division (faster)
        // Before: positionChange / (timeChange / 1000.0)
        // After: positionChange * 1000.0 / timeChange
        float pulsesPerSecond = (float)positionChange * 1000.0 / (float)timeChange;
        speed = (pulsesPerSecond / pulsesPerRevolution) * wheelCircumference;
    }

    // Cập nhật giá trị cũ
    prevPosition = currentPosition;
    prevTime = currentTime;

    this->currentSpeed = speed;
    return speed;
}

void MotorController::controlMotor(int pwmValue) {
    if (pwmValue >0) {
        digitalWrite(in1Pin, HIGH);
        digitalWrite(in2Pin, LOW);
    }
    else {
        digitalWrite(in1Pin, LOW);
        digitalWrite(in2Pin, HIGH);
    }
    analogWrite(enPin, abs(pwmValue));
    if (pwmValue == 0) {
        // Kích hoạt chế độ phanh hoàn toàn
        digitalWrite(in1Pin, HIGH);
        digitalWrite(in2Pin, HIGH);
        delay(0.1);
    } 
}

float MotorController::getCurrentSpeed() {
    return this->currentSpeed;
}

long MotorController::getEncoderPulse() {
    // OPTIMIZATION 1: Use cached directionMultiplier
    return directionMultiplier * encoder.getCount() / 2;
}

int MotorController::getOutput() {
    return this->outputPwm;
}

void MotorController::runPID() {
    currentSpeed = this->calculateSpeed();
    currentSpeedPulse = this->mapData(currentSpeed, -maxSpeed, maxSpeed, -255, 255);

    // Apply velocity ramping if enabled
    if (maxAcceleration > 0.0) {
        unsigned long currentTime = millis();
        float dt = (currentTime - lastRampTime) / 1000.0;  // Convert to seconds
        lastRampTime = currentTime;

        // Calculate ramped speed
        currentRampedSpeed = applyRamp(targetSpeed, currentRampedSpeed, dt);

        // Set ramped speed as PID setpoint
        long setPulses = this->mapData(currentRampedSpeed, -maxSpeed, maxSpeed, -255, 255);
        pid.Setpoint(setPulses);
    }

    const double output = pid.Run(currentSpeedPulse);
    this->controlMotor((int)output);
    outputPwm = (int)output;
}

void MotorController::adjustOutput(int adjustment) {
    // Apply adjustment while keeping PWM within safe limits [-255, 255]
    int adjustedPwm = outputPwm + adjustment;

    // Clamp to valid PWM range
    if (adjustedPwm > 255) adjustedPwm = 255;
    if (adjustedPwm < -255) adjustedPwm = -255;

    // Apply adjusted PWM to motor
    this->controlMotor(adjustedPwm);
    outputPwm = adjustedPwm;
}

void MotorController::setMaxAcceleration(float maxAccel) {
    maxAcceleration = maxAccel;
}

float MotorController::applyRamp(float target, float current, float dt) {
    // Calculate the difference between target and current speed
    float speedDiff = target - current;

    // If we're already at target, return target
    if (abs(speedDiff) < 0.001) {
        return target;
    }

    // Calculate maximum allowed change in this time step
    float maxChange = maxAcceleration * dt;

    // Apply the change with acceleration limit
    if (speedDiff > maxChange) {
        // Need to accelerate, but limit the increase
        return current + maxChange;
    } else if (speedDiff < -maxChange) {
        // Need to decelerate, but limit the decrease
        return current - maxChange;
    } else {
        // Can reach target in this step
        return target;
    }
}