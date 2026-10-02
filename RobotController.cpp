/**
 * @file RobotController.cpp
 * @brief High-Performance Deterministic Robot State Machine Implementation with Restricted Manual Driving & 'W' Mode Toggle
 * @author Senior Embedded Systems Engineer
 */

#include "RobotController.h"

RobotController::RobotController(IMotorController &motors,
                                 EdgeSensorArray &edgeSensors,
                                 DistanceScanner &scanner,
                                 BluetoothLink &bluetooth,
                                 int obstacleThresholdCm)
    : _motors(motors), _edgeSensors(edgeSensors), _scanner(scanner),
      _bluetooth(bluetooth), _obstacleThresholdCm(obstacleThresholdCm),
      _autoMode(true), _state(STATE_BOOT),
      _recoveryStep(EDGE_REC_COMPLETE), _recoveryTimer(0),
      _recoveryTurnDir(DIR_STOP), _recoveryTurnDuration(0) {}

void RobotController::begin() {
    _motors.begin();
    _edgeSensors.begin();
    _scanner.begin();
    _bluetooth.begin();

    _state = STATE_AUTO_FORWARD;
    _motors.setCommand(DIR_FORWARD, MOTOR_NORMAL_SPEED);

    Serial.println(F("========================================"));
    Serial.println(F(" SENTIROVER FIRMWARE READY"));
    Serial.println(F(" Restricted Driving & 'W' Mode Active"));
    Serial.println(F("========================================"));
    _bluetooth.send("ROBOT READY (W TOGGLE ACTIVE)");
}

void RobotController::processSafety() {
    // Ticked at 100Hz by TaskScheduler
    _edgeSensors.update();

    bool frontLeftSafe  = _edgeSensors.isFrontLeftSafe();
    bool frontRightSafe = _edgeSensors.isFrontRightSafe();
    bool rearLeftSafe   = _edgeSensors.isRearLeftSafe();
    bool rearRightSafe  = _edgeSensors.isRearRightSafe();

    // ----------------------------------------------------
    // ACTIVE REAL-TIME SAFETY RESTRICTIONS (BOTH AUTO & MANUAL)
    // ----------------------------------------------------

    // 1. REAR SAFETY INTERLOCK: Stop immediately if driving backward near a rear cliff
    if (_motors.getCurrentDirection() == DIR_BACKWARD) {
        if (!rearLeftSafe || !rearRightSafe) {
            Serial.println(F("REAR CLIFF INTERLOCK! Emergency Motor Stop."));
            _bluetooth.send("RESTRICTION: REAR CLIFF STOP");
            _motors.stopImmediate();
            if (_state == STATE_AUTO_EDGE_RECOVERY) {
                // Skip backward phase and proceed directly to turning
                _recoveryStep = EDGE_REC_STOP_2;
                _recoveryTimer = millis();
            }
            return;
        }
    }

    // 2. FRONT SAFETY INTERLOCK IN MANUAL MODE: Stop if driving forward near cliff/obstacle
    if (!_autoMode && _motors.getCurrentDirection() == DIR_FORWARD) {
        int dist = _scanner.getDistance();
        bool obstacleAhead = (dist > 0 && dist < _obstacleThresholdCm);

        if (!frontLeftSafe || !frontRightSafe) {
            Serial.println(F("MANUAL FWD INTERLOCK: FRONT CLIFF!"));
            _bluetooth.send("RESTRICTION: FRONT CLIFF STOP");
            _motors.stopImmediate();
            return;
        } else if (obstacleAhead) {
            Serial.println(F("MANUAL FWD INTERLOCK: OBSTACLE AHEAD!"));
            _bluetooth.send("RESTRICTION: OBSTACLE STOP");
            _motors.stopImmediate();
            return;
        }
    }

    // 3. CONNECTION WATCHDOG FAIL-SAFE (Manual Mode)
    if (!_autoMode) {
        if (_bluetooth.isHeartbeatTimedOut() && _motors.getCurrentDirection() != DIR_STOP) {
            Serial.println(F("BT HEARTBEAT TIMEOUT - EMERGENCY STOP"));
            _bluetooth.send("FAIL-SAFE: COMMS TIMEOUT");
            _motors.stopImmediate();
        }
        return;
    }

    // ----------------------------------------------------
    // AUTONOMOUS FRONT CLIFF OVERRIDES (Priority 1)
    // ----------------------------------------------------
    if (_state != STATE_AUTO_EDGE_RECOVERY) {
        if (!frontLeftSafe && !frontRightSafe) {
            Serial.println(F("BOTH FRONT CLIFFS DETECTED! Reversing/Turning."));
            _bluetooth.send("CLIFF! FRONT BOTH");
            triggerEdgeRecovery(DIR_TURN_RIGHT, 600);
            return;
        }

        if (!frontLeftSafe) {
            Serial.println(F("FRONT-LEFT CLIFF DETECTED! Turning Right."));
            _bluetooth.send("CLIFF! FRONT-LEFT");
            triggerEdgeRecovery(DIR_TURN_RIGHT, 550);
            return;
        }

        if (!frontRightSafe) {
            Serial.println(F("FRONT-RIGHT CLIFF DETECTED! Turning Left."));
            _bluetooth.send("CLIFF! FRONT-RIGHT");
            triggerEdgeRecovery(DIR_TURN_LEFT, 550);
            return;
        }
    }

    // Process Edge Recovery FSM step if active
    if (_state == STATE_AUTO_EDGE_RECOVERY) {
        updateEdgeRecoveryFSM();
    } else if (_state == STATE_AUTO_FORWARD) {
        evaluateAutonomousLogic();
    }
}

void RobotController::processMotorRamp() {
    _motors.updateRamp();
}

void RobotController::processScanner() {
    _scanner.updateScan();

    if (_state == STATE_AUTO_SCANNING_OBSTACLE) {
        if (!_scanner.isScanning()) {
            ScanResult result = _scanner.getLatestScanResult();

            Serial.println(F("--- SCAN COMPLETE ---"));
            Serial.print(F("L: ")); Serial.print(result.leftDistance);
            Serial.print(F(" C: ")); Serial.print(result.centerDistance);
            Serial.print(F(" R: ")); Serial.println(result.rightDistance);

            bool frontLeftSafe  = _edgeSensors.isFrontLeftSafe();
            bool frontRightSafe = _edgeSensors.isFrontRightSafe();

            if (frontLeftSafe && result.leftDistance > result.rightDistance && result.leftDistance > _obstacleThresholdCm) {
                triggerEdgeRecovery(DIR_TURN_LEFT, 600);
            } else if (frontRightSafe && result.rightDistance >= result.leftDistance && result.rightDistance > _obstacleThresholdCm) {
                triggerEdgeRecovery(DIR_TURN_RIGHT, 600);
            } else {
                triggerEdgeRecovery(DIR_TURN_RIGHT, 750); // Reverse & 180 turn
            }
        }
    }
}

void RobotController::processBluetooth() {
    char cmd;
    while (_bluetooth.readCommand(cmd)) {
        handleBluetoothCommand(cmd);
    }
}

void RobotController::processDiagnostics() {
    // 1 Hz Telemetry
}

void RobotController::handleBluetoothCommand(char cmd) {
    _bluetooth.resetHeartbeat();

    Serial.print(F("BT Cmd: "));
    Serial.println(cmd);

    char uppercaseCmd = toupper(cmd);

    switch (uppercaseCmd) {
        case 'W': // Mode Toggle Signal: Switch between Auto and Manual
            _autoMode = !_autoMode;
            _motors.stopImmediate();
            if (_autoMode) {
                _state = STATE_AUTO_FORWARD;
                _motors.setCommand(DIR_FORWARD, MOTOR_NORMAL_SPEED);
                _bluetooth.send("MODE: AUTONOMOUS (W)");
                Serial.println(F("TOGGLE 'W': AUTONOMOUS MODE ACTIVE"));
            } else {
                _state = STATE_MANUAL_DRIVE;
                _bluetooth.send("MODE: MANUAL (W)");
                Serial.println(F("TOGGLE 'W': MANUAL MODE ACTIVE"));
            }
            break;

        case 'A': // Explicit Autonomous mode command
            _autoMode = true;
            _state = STATE_AUTO_FORWARD;
            _motors.setCommand(DIR_FORWARD, MOTOR_NORMAL_SPEED);
            _bluetooth.send("MODE: AUTONOMOUS");
            break;

        case 'M': // Explicit Manual mode command
            _autoMode = false;
            _state = STATE_MANUAL_DRIVE;
            _motors.stopImmediate();
            _bluetooth.send("MODE: MANUAL");
            break;

        case 'F': // Forward Command (With Safety Restrictions)
            if (!_autoMode) {
                bool frontSafe = _edgeSensors.isFrontSafe();
                int dist = _scanner.getDistance();
                bool obstacleAhead = (dist > 0 && dist < _obstacleThresholdCm);

                if (!frontSafe) {
                    Serial.println(F("REJECT 'F': FRONT CLIFF DETECTED!"));
                    _bluetooth.send("BLOCKED: FRONT CLIFF");
                    _motors.stopImmediate();
                } else if (obstacleAhead) {
                    Serial.println(F("REJECT 'F': OBSTACLE AHEAD!"));
                    _bluetooth.send("BLOCKED: OBSTACLE AHEAD");
                    _motors.stopImmediate();
                } else {
                    _motors.setCommand(DIR_FORWARD, MOTOR_NORMAL_SPEED);
                }
            }
            break;

        case 'B': // Backward Command (With Safety Restrictions)
            if (!_autoMode) {
                bool rearSafe = _edgeSensors.isRearSafe();

                if (!rearSafe) {
                    Serial.println(F("REJECT 'B': REAR CLIFF DETECTED!"));
                    _bluetooth.send("BLOCKED: REAR CLIFF");
                    _motors.stopImmediate();
                } else {
                    _motors.setCommand(DIR_BACKWARD, MOTOR_NORMAL_SPEED);
                }
            }
            break;

        case 'L': // Turn Left
            if (!_autoMode) {
                _motors.setCommand(DIR_TURN_LEFT, MOTOR_TURN_SPEED);
            }
            break;

        case 'R': // Turn Right
            if (!_autoMode) {
                _motors.setCommand(DIR_TURN_RIGHT, MOTOR_TURN_SPEED);
            }
            break;

        case 'S': // Stop
            _motors.setCommand(DIR_STOP);
            break;

        default:
            break;
    }
}

void RobotController::evaluateAutonomousLogic() {
    int distance = _scanner.getDistance();

    if (distance > 0 && distance < _obstacleThresholdCm) {
        Serial.println(F("OBSTACLE DETECTED! Scanning."));
        _bluetooth.send("OBSTACLE!");
        _motors.setCommand(DIR_STOP);
        _state = STATE_AUTO_SCANNING_OBSTACLE;
        _scanner.startScan();
    } else {
        if (_motors.getCurrentDirection() != DIR_FORWARD) {
            _motors.setCommand(DIR_FORWARD, MOTOR_NORMAL_SPEED);
        }
    }
}

void RobotController::triggerEdgeRecovery(DriveDirection turnDir, unsigned long turnMs) {
    _state = STATE_AUTO_EDGE_RECOVERY;
    _recoveryStep = EDGE_REC_STOP_1;
    _recoveryTimer = millis();
    _recoveryTurnDir = turnDir;
    _recoveryTurnDuration = turnMs;
    _motors.setCommand(DIR_STOP);
}

void RobotController::updateEdgeRecoveryFSM() {
    unsigned long elapsed = millis() - _recoveryTimer;

    switch (_recoveryStep) {
        case EDGE_REC_STOP_1:
            if (elapsed >= 100) {
                if (_edgeSensors.isRearSafe()) {
                    _motors.setCommand(DIR_BACKWARD, MOTOR_NORMAL_SPEED);
                    _recoveryStep = EDGE_REC_BACKWARD;
                } else {
                    _recoveryStep = EDGE_REC_STOP_2;
                }
                _recoveryTimer = millis();
            }
            break;

        case EDGE_REC_BACKWARD:
            if (elapsed >= 400 || !_edgeSensors.isRearSafe()) {
                _motors.setCommand(DIR_STOP);
                _recoveryStep = EDGE_REC_STOP_2;
                _recoveryTimer = millis();
            }
            break;

        case EDGE_REC_STOP_2:
            if (elapsed >= 100) {
                _motors.setCommand(_recoveryTurnDir, MOTOR_TURN_SPEED);
                _recoveryStep = EDGE_REC_TURN;
                _recoveryTimer = millis();
            }
            break;

        case EDGE_REC_TURN:
            if (elapsed >= _recoveryTurnDuration) {
                _motors.setCommand(DIR_FORWARD, MOTOR_NORMAL_SPEED);
                _recoveryStep = EDGE_REC_COMPLETE;
                _state = STATE_AUTO_FORWARD;
            }
            break;

        case EDGE_REC_COMPLETE:
        default:
            _state = STATE_AUTO_FORWARD;
            break;
    }
}
