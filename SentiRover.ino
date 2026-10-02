/**
 * @file SentiRover.ino
 * @brief High-Performance Non-Blocking Autonomous Table-Safe Robot Main Entry
 * @author Senior Embedded Systems Engineer
 * @date 2026
 *
 * System Architecture & Hardware Connections (User Exact Wiring):
 * - Servo Motor: Pin 6
 * - L298N Motor Driver:
 *     Left Motor Forward  (IN1): Pin 3
 *     Left Motor Backward (IN2): Pin 2
 *     Right Motor Forward (IN3): Pin 5
 *     Right Motor Backward(IN4): Pin 4
 * - HC-SR04 Ultrasonic Sensor: Trig Pin 13, Echo Pin 12
 * - 4x IR Downward Cliff Sensors:
 *     Left Forward (Front-Left): Pin 8
 *     Right Forward (Front-Right): Pin 9
 *     Right Backward (Rear-Right): Pin 10
 *     Left Backward (Rear-Left): Pin 11
 * - HC-05 Bluetooth Module: RX Pin A4, TX Pin A5
 */

#include "Config.h"
#include "TaskScheduler.h"
#include "MotorDriver.h"
#include "EdgeSensorArray.h"
#include "DistanceScanner.h"
#include "BluetoothLink.h"
#include "RobotController.h"

// ============================================================
// SYSTEM HARDWARE INSTANTIATION (USER EXACT PIN CONFIG)
// ============================================================

// 4-Pin L298N Motor Driver
MotorDriver motors(
    PIN_MOTOR_LEFT_FWD,  PIN_MOTOR_LEFT_REV,
    PIN_MOTOR_RIGHT_FWD, PIN_MOTOR_RIGHT_REV,
    MOTOR_RAMP_STEP
);

// Downward IR Edge Sensors (4x Sensors: 2 Front, 2 Rear)
EdgeSensorArray edgeSensors(
    PIN_EDGE_FRONT_LEFT, PIN_EDGE_FRONT_RIGHT,
    PIN_EDGE_REAR_LEFT,  PIN_EDGE_REAR_RIGHT,
    EDGE_FLOOR_ACTIVE_LEVEL, EDGE_DEBOUNCE_SAMPLES
);

// Non-Blocking Ultrasonic Distance Scanner
DistanceScanner scanner(
    PIN_US_TRIG, PIN_US_ECHO, PIN_SERVO,
    SERVO_CENTER_ANGLE, SERVO_LEFT_ANGLE, SERVO_RIGHT_ANGLE
);

// Telemetry & Control Link (HC-05)
BluetoothLink bluetooth(PIN_BT_RX, PIN_BT_TX, BT_BAUD_RATE, BT_HEARTBEAT_TIMEOUT);

// System Master Controller FSM
RobotController robot(motors, edgeSensors, scanner, bluetooth, OBSTACLE_THRESHOLD_CM);

// Cooperative Task Scheduler Pool Allocation
TaskScheduler scheduler;

// Wrapper function shims for task callbacks
void taskSafetyWrapper()      { robot.processSafety(); }
void taskMotorRampWrapper()   { robot.processMotorRamp(); }
void taskScannerWrapper()     { robot.processScanner(); }
void taskBluetoothWrapper()   { robot.processBluetooth(); }
void taskDiagnosticsWrapper() { robot.processDiagnostics(); }

// ============================================================
// SYSTEM SETUP & INITIALIZATION
// ============================================================

void setup() {
    Serial.begin(115200); // High-speed hardware serial telemetry

    // Initialize Subsystems
    robot.begin();

    // Register Tasks with Scheduler
    scheduler.addTask(taskSafetyWrapper,      TASK_PERIOD_SAFETY_MS); // 100 Hz 360° Cliff Safety
    scheduler.addTask(taskMotorRampWrapper,   TASK_PERIOD_MOTOR_MS);  // 100 Hz Motor Speed Ramping
    scheduler.addTask(taskScannerWrapper,     TASK_PERIOD_SCAN_MS);   // 33 Hz Non-Blocking Scanner
    scheduler.addTask(taskBluetoothWrapper,   TASK_PERIOD_BT_MS);     // 50 Hz Bluetooth Polling
    scheduler.addTask(taskDiagnosticsWrapper, TASK_PERIOD_DIAG_MS);   // 1 Hz Telemetry Diagnostic
}

// ============================================================
// EXECUTIVE SUPERVISORY LOOP (Zero Delay - Deterministic Tick)
// ============================================================

void loop() {
    scheduler.tick();
}