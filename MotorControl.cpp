#include "MotorControl.h"
#include "Config.h"

#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif
#ifndef ESP_ARDUINO_VERSION_MAJOR
#define ESP_ARDUINO_VERSION_MAJOR 2
#endif

void MotorControl::begin() {
    pinMode(Config::LEFT_IN1_PIN, OUTPUT);
    pinMode(Config::LEFT_IN2_PIN, OUTPUT);
    pinMode(Config::RIGHT_IN1_PIN, OUTPUT);
    pinMode(Config::RIGHT_IN2_PIN, OUTPUT);

    attachPwm(Config::LEFT_PWM_PIN, 0);
    attachPwm(Config::RIGHT_PWM_PIN, 1);
    stop();
}

void MotorControl::attachPwm(uint8_t pin, uint8_t channel) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    (void)channel;
    ledcAttach(pin, Config::MOTOR_PWM_FREQUENCY_HZ, Config::MOTOR_PWM_RESOLUTION_BITS);
#else
    ledcSetup(channel, Config::MOTOR_PWM_FREQUENCY_HZ, Config::MOTOR_PWM_RESOLUTION_BITS);
    ledcAttachPin(pin, channel);
#endif
}

void MotorControl::writePwm(uint8_t pin, uint8_t channel, uint8_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    (void)channel;
    ledcWrite(pin, duty);
#else
    (void)pin;
    ledcWrite(channel, duty);
#endif
}

void MotorControl::setOneMotor(int speed, uint8_t pwmPin, uint8_t in1Pin,
                               uint8_t in2Pin, uint8_t channel, bool reversed) {
    speed = constrain(speed, -255, 255);
    if (reversed) speed = -speed;

    if (speed == 0) {
        digitalWrite(in1Pin, LOW);
        digitalWrite(in2Pin, LOW);
        writePwm(pwmPin, channel, 0);
        return;
    }

    if (speed > 0) {
        digitalWrite(in1Pin, HIGH);
        digitalWrite(in2Pin, LOW);
    } else {
        digitalWrite(in1Pin, LOW);
        digitalWrite(in2Pin, HIGH);
    }

    writePwm(pwmPin, channel, static_cast<uint8_t>(abs(speed)));
}

void MotorControl::setSpeeds(int leftSpeed, int rightSpeed) {
    setOneMotor(leftSpeed, Config::LEFT_PWM_PIN, Config::LEFT_IN1_PIN,
                Config::LEFT_IN2_PIN, 0, Config::LEFT_MOTOR_REVERSED);
    setOneMotor(rightSpeed, Config::RIGHT_PWM_PIN, Config::RIGHT_IN1_PIN,
                Config::RIGHT_IN2_PIN, 1, Config::RIGHT_MOTOR_REVERSED);
}

void MotorControl::stop() {
    setSpeeds(0, 0);
}
