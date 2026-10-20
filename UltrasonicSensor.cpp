#include "UltrasonicSensor.h"
#include "Config.h"

void UltrasonicSensor::begin() {
    pinMode(Config::ULTRASONIC_TRIG_PIN, OUTPUT);
    pinMode(Config::ULTRASONIC_ECHO_PIN, INPUT);
    digitalWrite(Config::ULTRASONIC_TRIG_PIN, LOW);
}

bool UltrasonicSensor::readDistanceCM(float &distanceCm) {
    digitalWrite(Config::ULTRASONIC_TRIG_PIN, LOW);
    delayMicroseconds(3);
    digitalWrite(Config::ULTRASONIC_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(Config::ULTRASONIC_TRIG_PIN, LOW);

    const unsigned long echoTimeUs = pulseIn(
        Config::ULTRASONIC_ECHO_PIN, HIGH, Config::ULTRASONIC_TIMEOUT_US);

    if (echoTimeUs == 0) return false;

    // Speed of sound is approximately 0.0343 cm per microsecond.
    distanceCm = (static_cast<float>(echoTimeUs) * 0.0343f) / 2.0f;

    return distanceCm >= 2.0f && distanceCm <= 350.0f;
}
