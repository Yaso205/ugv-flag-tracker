#pragma once

#include <Arduino.h>

class MotorControl {
public:
    void begin();

    // Signed speed: -255 to +255. Positive means forward after the per-motor reverse setting.
    void setSpeeds(int leftSpeed, int rightSpeed);
    void stop();

private:
    void attachPwm(uint8_t pin, uint8_t channel);
    void writePwm(uint8_t pin, uint8_t channel, uint8_t duty);
    void setOneMotor(int speed, uint8_t pwmPin, uint8_t in1Pin,
                     uint8_t in2Pin, uint8_t channel, bool reversed);
};
