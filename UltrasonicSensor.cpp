#include "UltrasonicSensor.h"
#include "Config.h"

#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.393701

long duration;
float distanceCm;
float distanceInch;

void UltrasonicSensor::begin() {
    pinMode(Config::ULTRASONIC_TRIG_PIN, OUTPUT);
    pinMode(Config::ULTRASONIC_ECHO_PIN, INPUT);
    digitalWrite(Config::ULTRASONIC_TRIG_PIN, LOW);
}

bool UltrasonicSensor::readDistanceCM(float &distanceCm) {
    digitalWrite(Config::ULTRASONIC_TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(Config::ULTRASONIC_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(Config::ULTRASONIC_TRIG_PIN, LOW);

    duration = pulseIn(Config::ULTRASONIC_ECHO_PIN, HIGH);

    const unsigned long echoTimeUs = pulseIn(
        Config::ULTRASONIC_ECHO_PIN, HIGH, Config::ULTRASONIC_TIMEOUT_US);

    if (echoTimeUs == 0) return false;

    // Speed of sound is approximately 0.0343 cm per microsecond.
    distanceCm = (static_cast<float>(echoTimeUs) * 0.0343f) / 2.0f;

    return distanceCm >= 2.0f && distanceCm <= 350.0f;
}
