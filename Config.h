/**
 * @file Config.h
 * @brief System Hardware Pin Configuration, Parameters, and Macro Definitions
 * @author Senior Embedded Systems Engineer
 * @date 2026
 *
 * SentiRover Autonomous Table-Safe Robotic System
 * Targeted Microcontroller: Microchip ATmega328P (Arduino UNO / Nano)
 */

#ifndef SENTI_ROVER_CONFIG_H
#define SENTI_ROVER_CONFIG_H

#include <Arduino.h>

// ============================================================
// HARDWARE PIN ALLOCATIONS (VERIFIED WORKING HARDWARE SNIPPET)
// ============================================================

// -- L298N Motor Driver Pins (Exact hardware mapping) --
#define PIN_MOTOR_LEFT_REV    2    // MLa (Left Motor 1st pin)
#define PIN_MOTOR_LEFT_FWD    3    // MLb (Left Motor 2nd pin)
#define PIN_MOTOR_RIGHT_REV   4    // MRa (Right Motor 1st pin)
#define PIN_MOTOR_RIGHT_FWD   5    // MRb (Right Motor 2nd pin)

// -- HC-SR04 Ultrasonic & Servo Scanner (Exact hardware mapping) --
#define PIN_US_TRIG           13   // Trig Pin Of HC-SR04
#define PIN_US_ECHO           12   // Echo Pin Of HC-SR04
#define PIN_SERVO             6    // Myservo signal pin

// -- IR Downward Edge Sensors (4x Sensors: 2 Front, 2 Rear) --
#define PIN_EDGE_FRONT_LEFT   8    // Front-Left IR cliff sensor (Left Forward)
#define PIN_EDGE_FRONT_RIGHT  9    // Front-Right IR cliff sensor (Right Forward)
#define PIN_EDGE_REAR_LEFT    11   // Rear-Left IR cliff sensor (Left Backward)
#define PIN_EDGE_REAR_RIGHT   10   // Rear-Right IR cliff sensor (Right Backward)

// -- HC-05 Bluetooth Module (SoftwareSerial) --
#define PIN_BT_RX             A4   // SoftwareSerial RX (Connects to HC-05 TX)
#define PIN_BT_TX             A5   // SoftwareSerial TX (Connects to HC-05 RX)

// ============================================================
// SYSTEM PARAMETERS & CALIBRATION
// ============================================================

// Motor Drive Speeds (0 - 255 PWM)
#define MOTOR_NORMAL_SPEED     180
#define MOTOR_TURN_SPEED       190
#define MOTOR_RAMP_STEP        20   // PWM step per 10ms for responsive acceleration

// IR Edge Sensor Logic
#define EDGE_FLOOR_ACTIVE_LEVEL LOW // Digital level corresponding to floor present
#define EDGE_DEBOUNCE_SAMPLES   3   // Number of consecutive samples required for edge confirmation

// Distance Scanner Settings (Matching verified hardware calibration)
#define OBSTACLE_THRESHOLD_CM  15   // Trigger obstacle avoidance distance (15 cm)
#define SERVO_CENTER_ANGLE     90   // Center look angle
#define SERVO_LEFT_ANGLE       180  // Left look angle (180 deg)
#define SERVO_RIGHT_ANGLE      0    // Right look angle (0 deg)
#define SERVO_STEP_DELAY_MS    15   // Time per degree sweep step (non-blocking)
#define US_TIMEOUT_US          20000UL // Ultrasonic echo timeout (max ~3.4m)

// Communication & Safety Fail-Safes
#define BT_BAUD_RATE           9600
#define BT_HEARTBEAT_TIMEOUT   3000UL // Auto-stop motors after 3s of comm inactivity in manual mode

// Scheduler Task Frequencies (ms)
#define TASK_PERIOD_SAFETY_MS   10   // 100 Hz 360° Cliff detection task
#define TASK_PERIOD_MOTOR_MS    10   // 100 Hz Smooth PWM acceleration ramping
#define TASK_PERIOD_SCAN_MS     30   // ~33 Hz Distance scanner & servo state machine
#define TASK_PERIOD_BT_MS       20   // 50 Hz Bluetooth serial processing
#define TASK_PERIOD_DIAG_MS    1000  // 1 Hz Telemetry & debug heartbeat

#endif // SENTI_ROVER_CONFIG_H
