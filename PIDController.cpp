#include "PIDController.h"
#include "Config.h"

void PIDController::begin() {
    reset();
}

void PIDController::reset() {
    integral_ = 0.0f;
    filteredDerivative_ = 0.0f;
    output_ = 0;
    hasLast_ = false;
}

void PIDController::sendMeasuredDistanceCM(float distanceCm) {
    const uint32_t now = millis();

    // First sample (or first after reset): just remember it, no derivative yet.
    if (!hasLast_) {
        lastMeasurement_ = distanceCm;
        lastUpdateMs_ = now;
        hasLast_ = true;
        output_ = 0;
        return;
    }

    const uint32_t dtMs = now - lastUpdateMs_;
    if (dtMs == 0) return;

    // Long gap (sensor dropout, target lost): old integral/derivative are meaningless.
    if (dtMs > Config::PID_MAX_DT_MS) {
        integral_ = 0.0f;
        filteredDerivative_ = 0.0f;
        lastMeasurement_ = distanceCm;
        lastUpdateMs_ = now;
        output_ = 0;
        return;
    }

    const float dt = dtMs / 1000.0f;
    float error = distanceCm - Config::TARGET_DISTANCE_CM;   // + = too far

    // Derivative of the measurement (target is constant, so d(error)/dt == d(measurement)/dt),
    // low-pass filtered because ultrasonic readings are noisy.
    const float rawDerivative = (distanceCm - lastMeasurement_) / dt;
    filteredDerivative_ += Config::PID_DERIVATIVE_ALPHA * (rawDerivative - filteredDerivative_);

    lastMeasurement_ = distanceCm;
    lastUpdateMs_ = now;

    // Close enough: stop.
    if (fabsf(error) <= Config::PID_DEADBAND_CM) {
        output_ = 0;
        return;
    }

    const float pTerm = Config::PID_KP * error;
    const float dTerm = Config::PID_KD * filteredDerivative_;  // negative while approaching = brake

    // Anti-windup: don't keep integrating while the output is pinned at a limit
    // and the error would push it further into that limit.
    const float unsaturated = pTerm + Config::PID_KI * integral_ + dTerm;
    const bool pinnedLow  = (unsaturated <= 0.0f) && (error < 0.0f);
    const bool pinnedHigh = (unsaturated >= Config::MAX_APPROACH_SPEED) && (error > 0.0f);
    if (!pinnedLow && !pinnedHigh) {
        integral_ += error * dt;
        integral_ = constrain(integral_, -Config::PID_INTEGRAL_LIMIT, Config::PID_INTEGRAL_LIMIT);
    }

    float out = pTerm + Config::PID_KI * integral_ + dTerm;
    out = constrain(out, 0.0f, static_cast<float>(Config::MAX_APPROACH_SPEED));

    // Motors won't turn below a certain PWM, so lift small nonzero commands.
    if (out > 0.0f && out < Config::MIN_MOVING_SPEED) {
        out = Config::MIN_MOVING_SPEED;
    }

    output_ = static_cast<int>(lroundf(out));
}

int PIDController::readForwardSpeed() {
    return output_;
}