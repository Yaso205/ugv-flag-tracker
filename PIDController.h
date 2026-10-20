#pragma once
#include <Arduino.h>

class PIDController {
public:
    void begin();
    void reset();                                   // call when starting/restarting an approach
    void sendMeasuredDistanceCM(float distanceCm);  // run one PID update
    int  readForwardSpeed();                        // latest output, 0..MAX_APPROACH_SPEED

private:
    float integral_ = 0.0f;
    float lastMeasurement_ = 0.0f;
    float filteredDerivative_ = 0.0f;
    int   output_ = 0;
    uint32_t lastUpdateMs_ = 0;
    bool  hasLast_ = false;
};