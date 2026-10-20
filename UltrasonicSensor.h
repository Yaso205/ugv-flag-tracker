#pragma once

#include <Arduino.h>

class UltrasonicSensor {
public:
    void begin();

    // Returns true only for a plausible measurement. Distance is written to distanceCm.
    bool readDistanceCM(float &distanceCm);
};
