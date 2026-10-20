#pragma once

#include <Arduino.h>

// Adjust hardware pins and tuning values here. The rest of the project uses these names.
namespace Config {

// Motor driver: each motor needs PWM + two direction inputs.
constexpr uint8_t LEFT_PWM_PIN  = 27;//blue
constexpr uint8_t LEFT_IN1_PIN  = 19;
constexpr uint8_t LEFT_IN2_PIN  = 21;
constexpr uint8_t RIGHT_PWM_PIN = 23;//green
constexpr uint8_t RIGHT_IN1_PIN = 26;
constexpr uint8_t RIGHT_IN2_PIN = 25;
//left orange & brown | right red & yellow 

// Change to true for a motor whose wires make positive speed run backward.
constexpr bool LEFT_MOTOR_REVERSED  = false;
constexpr bool RIGHT_MOTOR_REVERSED = false;

// HC-SR04-style ultrasonic sensor. ECHO must be reduced to 3.3 V before the ESP32 pin.
constexpr uint8_t ULTRASONIC_TRIG_PIN = 13;
constexpr uint8_t ULTRASONIC_ECHO_PIN = 14;
constexpr uint32_t ULTRASONIC_TIMEOUT_US = 25000;
constexpr uint32_t SENSOR_INTERVAL_MS = 80;
constexpr uint32_t SENSOR_STALE_MS = 400;

// UART from the camera/CV controller to the main ESP32.
// Connect camera TX -> this RX, and share GND. TX is included for optional two-way communication.
constexpr int CAMERA_RX_PIN = 16;
constexpr int CAMERA_TX_PIN = 17;
constexpr uint32_t CAMERA_BAUD = 115200;
constexpr uint32_t CAMERA_STALE_MS = 600;
constexpr float CAMERA_MIN_CONFIDENCE = 0.60f;

// Software PID tuning (error is in cm, output is in PWM units 0..MAX_APPROACH_SPEED)
constexpr float PID_KP = 3.0f;               // 50 cm of error -> ~150 PWM
constexpr float PID_KI = 0.4f;
constexpr float PID_KD = 1.2f;
constexpr float PID_INTEGRAL_LIMIT = 40.0f;  // max stored integral (cm*s)
constexpr float PID_DERIVATIVE_ALPHA = 0.3f; // 0..1, lower = smoother but slower
constexpr float PID_DEADBAND_CM = 1.0f;      // inside this, output is 0
constexpr int   MIN_MOVING_SPEED = 60;       // lowest PWM that actually moves your robot
constexpr uint32_t PID_MAX_DT_MS = 500;      // gap longer than this resets the PID

// Motion and mission tuning.
constexpr uint8_t MOTOR_PWM_RESOLUTION_BITS = 8;
constexpr uint32_t MOTOR_PWM_FREQUENCY_HZ = 20000;
constexpr int MAX_APPROACH_SPEED = 155;
constexpr int SCAN_SPEED = 70;
constexpr int MAX_TURN_SPEED = 115;
constexpr float CAMERA_CENTER_DEADZONE = 0.12f;
constexpr float ARRIVAL_CENTER_TOLERANCE = 0.10f;
constexpr float TARGET_DISTANCE_CM = 50.0f;
constexpr float ARRIVAL_MIN_CM = 48.0f;
constexpr float ARRIVAL_MAX_CM = 52.0f;
constexpr uint8_t ARRIVAL_STABLE_READINGS = 4;
constexpr float OBSTACLE_STOP_CM = 18.0f;
constexpr uint8_t DETECTIONS_TO_CONFIRM = 3;
constexpr uint8_t LOST_FRAMES_BEFORE_SEARCH = 2;

}  // namespace Config
