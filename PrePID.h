#pragma once

#include <Arduino.h>

// PrePID filters sensor noise and estimates how quickly distance is changing.
// It does NOT calculate a PID control output.
class PrePID {
public:
    bool update(float rawDistanceCm, bool readingValid, uint32_t nowMs);
    bool isFresh(uint32_t nowMs) const;
    float distanceCM() const;
    float closingSpeedCMPerSec() const;
    uint32_t lastUpdateMs() const;

private:
    float medianOfAvailableSamples() const;

    float samples_[3] = {0.0f, 0.0f, 0.0f};
    uint8_t sampleCount_ = 0;
    uint8_t nextSample_ = 0;
    bool initialized_ = false;
    float filteredDistanceCm_ = 0.0f;
    float closingSpeedCmPerSec_ = 0.0f;
    uint32_t lastUpdateMs_ = 0;
};
