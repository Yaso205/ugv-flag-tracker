#include "PrePID.h"
#include "Config.h"

namespace {
constexpr float EMA_ALPHA = 0.30f;  // Higher = reacts faster; lower = smoother.
constexpr float MAX_CLOSING_SPEED_CM_PER_SEC = 250.0f;
}

float PrePID::medianOfAvailableSamples() const {
    if (sampleCount_ == 1) return samples_[0];
    if (sampleCount_ == 2) return (samples_[0] + samples_[1]) / 2.0f;

    float a = samples_[0];
    float b = samples_[1];
    float c = samples_[2];
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) { float t = b; b = c; c = t; }
    if (a > b) { float t = a; a = b; b = t; }
    return b;
}

bool PrePID::update(float rawDistanceCm, bool readingValid, uint32_t nowMs) {
    if (!readingValid || rawDistanceCm < 2.0f || rawDistanceCm > 350.0f) {
        return false;
    }

    const float previousFiltered = filteredDistanceCm_;
    const uint32_t previousTime = lastUpdateMs_;

    samples_[nextSample_] = rawDistanceCm;
    nextSample_ = (nextSample_ + 1) % 3;
    if (sampleCount_ < 3) ++sampleCount_;

    const float medianDistance = medianOfAvailableSamples();

    if (!initialized_) {
        filteredDistanceCm_ = medianDistance;
        closingSpeedCmPerSec_ = 0.0f;
        initialized_ = true;
    } else {
        filteredDistanceCm_ += EMA_ALPHA * (medianDistance - filteredDistanceCm_);

        const uint32_t elapsedMs = nowMs - previousTime;
        if (elapsedMs > 0) {
            // Positive speed means the UGV is getting closer to the object.
            closingSpeedCmPerSec_ =
                (previousFiltered - filteredDistanceCm_) * 1000.0f / elapsedMs;
            closingSpeedCmPerSec_ = constrain(
                closingSpeedCmPerSec_,
                -MAX_CLOSING_SPEED_CM_PER_SEC,
                MAX_CLOSING_SPEED_CM_PER_SEC);
        }
    }

    lastUpdateMs_ = nowMs;
    return true;
}

bool PrePID::isFresh(uint32_t nowMs) const {
    return initialized_ && (nowMs - lastUpdateMs_ <= Config::SENSOR_STALE_MS);
}

float PrePID::distanceCM() const {
    return filteredDistanceCm_;
}

float PrePID::closingSpeedCMPerSec() const {
    return closingSpeedCmPerSec_;
}

uint32_t PrePID::lastUpdateMs() const {
    return lastUpdateMs_;
}
