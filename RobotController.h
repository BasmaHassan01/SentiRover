/**
 * @file RobotController.h
 * @brief High-Performance Deterministic Robot State Machine Controller
 * @author Senior Embedded Systems Engineer
 */

#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include "Config.h"
#include "IMotorController.h"
#include "EdgeSensorArray.h"
#include "DistanceScanner.h"
#include "BluetoothLink.h"

enum RobotState {
    STATE_BOOT,
    STATE_MANUAL_DRIVE,
    STATE_AUTO_FORWARD,
    STATE_AUTO_EDGE_RECOVERY,
    STATE_AUTO_SCANNING_OBSTACLE,
    STATE_AUTO_OBSTACLE_RECOVERY,
    STATE_EMERGENCY_STOP
};

enum EdgeRecoveryStep {
    EDGE_REC_STOP_1,
    EDGE_REC_BACKWARD,
    EDGE_REC_STOP_2,
    EDGE_REC_TURN,
    EDGE_REC_COMPLETE
};

class RobotController {
public:
    RobotController(IMotorController &motors,
                    EdgeSensorArray &edgeSensors,
                    DistanceScanner &scanner,
                    BluetoothLink &bluetooth,
                    int obstacleThresholdCm = OBSTACLE_THRESHOLD_CM);

    void begin();
    
    // Scheduler Task Callbacks
    void processSafety();       // 100 Hz: High-priority cliff check
    void processMotorRamp();     // 100 Hz: PWM acceleration step
    void processScanner();       // ~33 Hz: Obstacle scanner state machine tick
    void processBluetooth();     // 50 Hz: Non-blocking serial command processing
    void processDiagnostics();   // 1 Hz: Telemetry output

private:
    IMotorController &_motors;
    EdgeSensorArray &_edgeSensors;
    DistanceScanner &_scanner;
    BluetoothLink &_bluetooth;

    int _obstacleThresholdCm;
    bool _autoMode;
    RobotState _state;

    // Non-blocking recovery state machine variables
    EdgeRecoveryStep _recoveryStep;
    unsigned long _recoveryTimer;
    DriveDirection _recoveryTurnDir;
    unsigned long _recoveryTurnDuration;

    void handleBluetoothCommand(char cmd);
    void evaluateAutonomousLogic();
    void triggerEdgeRecovery(DriveDirection turnDir, unsigned long turnMs);
    void updateEdgeRecoveryFSM();
    void triggerObstacleAvoidance();
};

#endif // ROBOT_CONTROLLER_H
