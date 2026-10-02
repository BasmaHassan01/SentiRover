/**
 * @file IMotorController.h
 * @brief Abstract Interface for Motor Controllers
 * @author Senior Embedded Systems Engineer
 */

#ifndef I_MOTOR_CONTROLLER_H
#define I_MOTOR_CONTROLLER_H

#include <Arduino.h>

enum DriveDirection {
    DIR_STOP,
    DIR_FORWARD,
    DIR_BACKWARD,
    DIR_TURN_LEFT,
    DIR_TURN_RIGHT
};

class IMotorController {
public:
    virtual ~IMotorController() {}

    virtual void begin() = 0;
    virtual void setCommand(DriveDirection dir, uint8_t targetSpeed = 160) = 0;
    virtual void stopImmediate() = 0;
    virtual void updateRamp() = 0; // Periodic non-blocking PWM acceleration ramping tick
    virtual DriveDirection getCurrentDirection() const = 0;
    virtual bool isTargetReached() const = 0;
};

#endif // I_MOTOR_CONTROLLER_H
