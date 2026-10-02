/**
 * @file MotorDriver.h
 * @brief 4-Pin L298N H-Bridge Driver with Non-Blocking PWM Acceleration Ramping
 * @author Senior Embedded Systems Engineer
 */

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "IMotorController.h"

class MotorDriver : public IMotorController {
public:
    MotorDriver(uint8_t leftFwdPin, uint8_t leftRevPin,
                uint8_t rightFwdPin, uint8_t rightRevPin,
                uint8_t rampStep = 15);

    void begin() override;
    void setCommand(DriveDirection dir, uint8_t targetSpeed = 160) override;
    void stopImmediate() override;
    void updateRamp() override;
    DriveDirection getCurrentDirection() const override { return _currentDir; }
    bool isTargetReached() const override { return _currentSpeed == _targetSpeed; }

private:
    uint8_t _leftFwdPin;
    uint8_t _leftRevPin;
    uint8_t _rightFwdPin;
    uint8_t _rightRevPin;
    uint8_t _rampStep;

    DriveDirection _currentDir;
    DriveDirection _targetDir;
    uint8_t _currentSpeed;
    uint8_t _targetSpeed;

    void applyHardwarePins(DriveDirection dir, uint8_t speed);
};

#endif // MOTOR_DRIVER_H
