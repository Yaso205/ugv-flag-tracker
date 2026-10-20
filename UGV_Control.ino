#include "Config.h"
#include "MotorControl.h"
#include "UltrasonicSensor.h"
#include "PrePID.h"
#include "PIDController.h"
// #include "CameraCommunication.h"  // Temporarily disable camera support.

namespace {
enum class MissionState {
    SEARCHING,
    TRACKING,
    COMPLETE,
    OBSTACLE_HALT
};

MotorControl motors;
UltrasonicSensor ultrasonic;
PrePID prePid;
PIDController softwarePid;
// CameraCommunication camera;

MissionState missionState = MissionState::SEARCHING;
uint32_t lastSensorReadMs = 0;
// uint32_t lastCameraMessageProcessed = 0;
uint32_t lastArrivalSampleMs = 0;
// uint8_t detectionCount = 0;
// uint8_t lostFrameCount = 0;
// uint8_t arrivalStableCount = 0;
bool flagVisited = false;

void printState(const char *message) {
    Serial.println(message);
}

void completeMission() {
    flagVisited = true;
    missionState = MissionState::COMPLETE;
    motors.stop();
    printState("Flag reached. Marked visited; mission complete.");
}

/* Camera-dependent code temporarily disabled.
void updateCameraState(uint32_t nowMs) {
    const CameraData &frame = camera.data();
    if (frame.messageCounter == lastCameraMessageProcessed) return;
    lastCameraMessageProcessed = frame.messageCounter;

    if (missionState == MissionState::SEARCHING) {
        if (frame.detected) {
            if (detectionCount < Config::DETECTIONS_TO_CONFIRM) ++detectionCount;
            if (detectionCount >= Config::DETECTIONS_TO_CONFIRM) {
                missionState = MissionState::TRACKING;
                lostFrameCount = 0;
                printState("Flag confirmed. Tracking.");
            }
        } else {
            detectionCount = 0;
        }
    } else if (missionState == MissionState::TRACKING) {
        if (frame.detected) {
            lostFrameCount = 0;
        } else {
            if (lostFrameCount < Config::LOST_FRAMES_BEFORE_SEARCH) ++lostFrameCount;
            if (lostFrameCount >= Config::LOST_FRAMES_BEFORE_SEARCH) {
                missionState = MissionState::SEARCHING;
                detectionCount = 0;
                arrivalStableCount = 0;
                printState("Flag lost. Searching again.");
            }
        }
    }
    (void)nowMs;
}

float normalizedCameraError(const CameraData &frame) {
    if (frame.imageWidth <= 0) return 0.0f;
    const float center = frame.imageWidth / 2.0f;
    return constrain((frame.centerX - center) / center, -1.0f, 1.0f);
}

void checkArrival(const CameraData &frame, float distanceCm) {
    const uint32_t sensorTime = prePid.lastUpdateMs();
    if (sensorTime == lastArrivalSampleMs) return;
    lastArrivalSampleMs = sensorTime;

    const float error = normalizedCameraError(frame);
    const bool centered = fabsf(error) <= Config::ARRIVAL_CENTER_TOLERANCE;
    const bool inTargetBand = distanceCm >= Config::ARRIVAL_MIN_CM &&
                              distanceCm <= Config::ARRIVAL_MAX_CM;

    if (centered && inTargetBand) {
        if (arrivalStableCount < Config::ARRIVAL_STABLE_READINGS) ++arrivalStableCount;
    } else {
        arrivalStableCount = 0;
    }

    // If it passes slightly inside the target distance while centered, stop for safety too.
    if (centered && distanceCm < Config::ARRIVAL_MIN_CM && distanceCm >= Config::OBSTACLE_STOP_CM) {
        completeMission();
        return;
    }

    if (arrivalStableCount >= Config::ARRIVAL_STABLE_READINGS) {
        completeMission();
    }
}

void followFlag(const CameraData &frame, float distanceCm) {
    const float error = normalizedCameraError(frame);

    // Turn in place until the flag is near the middle of the image.
    // If your steering direction is backwards, change motor reversal settings in Config.h.
    if (fabsf(error) > Config::CAMERA_CENTER_DEADZONE) {
        const int turnSpeed = constrain(
            static_cast<int>(fabsf(error) * Config::MAX_TURN_SPEED),
            65, Config::MAX_TURN_SPEED);
        if (error < 0.0f) {
            motors.setSpeeds(-turnSpeed, turnSpeed);  // Flag is on the left.
        } else {
            motors.setSpeeds(turnSpeed, -turnSpeed);  // Flag is on the right.
        }
        return;
    }

    // Only move forward after camera alignment. The physical PID sets forward speed.
    const int forwardSpeed = softwarePid.readForwardSpeed();
    const int steering = static_cast<int>(error * 45.0f);
    const int leftSpeed = constrain(forwardSpeed + steering, -255, 255);
    const int rightSpeed = constrain(forwardSpeed - steering, -255, 255);
    (void)distanceCm;
    motors.setSpeeds(leftSpeed, rightSpeed);
}
*/
}  // namespace

void setup() {
    Serial.begin(115200);  // USB debug output; camera data uses Serial1.
    motors.begin();
    ultrasonic.begin();
    softwarePid.begin();
    // camera.begin();  // Temporarily disable camera support.
    motors.stop();

    Serial.println("UGV controller ready.");
    Serial.println("Waiting for camera messages and a valid distance reading.");
}

void loop() {
    const uint32_t nowMs = millis();
    // camera.poll();  // Temporarily disable camera support.

    if (nowMs - lastSensorReadMs >= Config::SENSOR_INTERVAL_MS) {
        lastSensorReadMs = nowMs;
        float rawDistanceCm = 0.0f;
        if (ultrasonic.readDistanceCM(rawDistanceCm)) {
            if (prePid.update(rawDistanceCm, true, nowMs)) {
                softwarePid.sendMeasuredDistanceCM(prePid.distanceCM());
            }
        }
    }

    if (missionState == MissionState::COMPLETE || flagVisited) {
        motors.stop();
        return;
    }

    if (missionState == MissionState::OBSTACLE_HALT) {
        motors.stop();
        return;
    }

    // Do not move using stale sensor information.
    if (!prePid.isFresh(nowMs)) {
        motors.stop();
        return;
    }

    const float distanceCm = prePid.distanceCM();
    if (distanceCm <= Config::OBSTACLE_STOP_CM) {
        missionState = MissionState::OBSTACLE_HALT;
        motors.stop();
        printState("Safety stop: object is too close. Reset after checking the path.");
        return;
    }

    /* Camera-driven mission behavior temporarily disabled.
    updateCameraState(nowMs);
    const CameraData &frame = camera.data();

    if (missionState == MissionState::SEARCHING) {
        // Pause rotation while confirming a possible detection so it stays in view.
        if (detectionCount > 0) {
            motors.stop();
        } else {
            motors.setSpeeds(-Config::SCAN_SPEED, Config::SCAN_SPEED);
        }
        return;
    }

    if (missionState == MissionState::TRACKING) {
        if (!frame.detected) {
            motors.stop();
            return;
        }

        checkArrival(frame, distanceCm);
        if (missionState == MissionState::COMPLETE) {
            motors.stop();
            return;
        }

        followFlag(frame, distanceCm);
    }
    */
    motors.stop();
}
