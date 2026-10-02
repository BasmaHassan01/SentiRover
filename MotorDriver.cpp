/**
 * @file MotorDriver.cpp
 * @brief Implementation of 4-Pin L298N Motor Driver with Smooth PWM Speed Ramping
 * @author Senior Embedded Systems Engineer
 */

#include "MotorDriver.h"

MotorDriver::MotorDriver(uint8_t leftFwdPin, uint8_t leftRevPin,
                         uint8_t rightFwdPin, uint8_t rightRevPin,
                         uint8_t rampStep)
    : _leftFwdPin(leftFwdPin), _leftRevPin(leftRevPin),
      _rightFwdPin(rightFwdPin), _rightRevPin(rightRevPin),
      _rampStep(rampStep),
      _currentDir(DIR_STOP), _targetDir(DIR_STOP),
      _currentSpeed(0), _targetSpeed(0) {}

void MotorDriver::begin() {
    pinMode(_leftFwdPin, OUTPUT);
    pinMode(_leftRevPin, OUTPUT);
    pinMode(_rightFwdPin, OUTPUT);
    pinMode(_rightRevPin, OUTPUT);

    stopImmediate();
}

void MotorDriver::setCommand(DriveDirection dir, uint8_t targetSpeed) {
    if (dir == DIR_STOP) {
        _targetSpeed = 0;
        _targetDir = DIR_STOP;
        return;
    }

    if (_currentDir != DIR_STOP && _currentDir != dir) {
        _targetSpeed = 0; // Ramp down to 0 first on direction change
        _targetDir = dir;
    } else {
        _targetDir = dir;
        _targetSpeed = targetSpeed;
        _currentDir = dir;
    }
}

void MotorDriver::stopImmediate() {
    _currentDir = DIR_STOP;
    _targetDir = DIR_STOP;
    _currentSpeed = 0;
    _targetSpeed = 0;
    applyHardwarePins(DIR_STOP, 0);
}

void MotorDriver::updateRamp() {
    if (_currentSpeed == _targetSpeed && _currentDir == _targetDir) {
        return; // Steady state
    }

    if (_currentSpeed < _targetSpeed) {
        if ((uint16_t)_currentSpeed + _rampStep >= _targetSpeed) {
            _currentSpeed = _targetSpeed;
        } else {
            _currentSpeed += _rampStep;
        }
    } else if (_currentSpeed > _targetSpeed) {
        if (_currentSpeed <= _rampStep || _currentSpeed - _rampStep <= _targetSpeed) {
            _currentSpeed = _targetSpeed;
        } else {
            _currentSpeed -= _rampStep;
        }
    }

    if (_currentSpeed == 0 && _targetDir != _currentDir) {
        _currentDir = _targetDir;
    }

    applyHardwarePins(_currentDir, _currentSpeed);
}

void MotorDriver::applyHardwarePins(DriveDirection dir, uint8_t speed) {
    switch (dir) {
        case DIR_FORWARD:
            analogWrite(_leftFwdPin, speed);   digitalWrite(_leftRevPin, LOW);
            analogWrite(_rightFwdPin, speed);  digitalWrite(_rightRevPin, LOW);
            break;

        case DIR_BACKWARD:
            digitalWrite(_leftFwdPin, LOW);    analogWrite(_leftRevPin, speed);
            digitalWrite(_rightFwdPin, LOW);   analogWrite(_rightRevPin, speed);
            break;

        case DIR_TURN_LEFT:
            digitalWrite(_leftFwdPin, LOW);    analogWrite(_leftRevPin, speed);
            analogWrite(_rightFwdPin, speed);  digitalWrite(_rightRevPin, LOW);
            break;

        case DIR_TURN_RIGHT:
            analogWrite(_leftFwdPin, speed);   digitalWrite(_leftRevPin, LOW);
            digitalWrite(_rightFwdPin, LOW);   analogWrite(_rightRevPin, speed);
            break;

        case DIR_STOP:
        default:
            digitalWrite(_leftFwdPin, LOW);    digitalWrite(_leftRevPin, LOW);
            digitalWrite(_rightFwdPin, LOW);   digitalWrite(_rightRevPin, LOW);
            break;
    }
}
