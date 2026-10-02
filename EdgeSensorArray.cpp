/**
 * @file EdgeSensorArray.cpp
 * @brief Implementation of Debounced 4x Edge Sensor Array
 * @author Senior Embedded Systems Engineer
 */

#include "EdgeSensorArray.h"

EdgeSensorArray::EdgeSensorArray(uint8_t frontLeftPin, uint8_t frontRightPin,
                                 uint8_t rearLeftPin, uint8_t rearRightPin,
                                 int floorActiveLevel, uint8_t debounceSamples)
    : _frontLeftPin(frontLeftPin), _frontRightPin(frontRightPin),
      _rearLeftPin(rearLeftPin), _rearRightPin(rearRightPin),
      _floorActiveLevel(floorActiveLevel), _debounceSamples(debounceSamples),
      _frontLeftHistory(0), _frontRightHistory(0),
      _rearLeftHistory(0), _rearRightHistory(0),
      _frontLeftSafe(true), _frontRightSafe(true),
      _rearLeftSafe(true), _rearRightSafe(true) {}

void EdgeSensorArray::begin() {
    pinMode(_frontLeftPin, INPUT);
    pinMode(_frontRightPin, INPUT);
    pinMode(_rearLeftPin, INPUT);
    pinMode(_rearRightPin, INPUT);

    // Initial safe assumptions
    _frontLeftSafe  = true;
    _frontRightSafe = true;
    _rearLeftSafe   = true;
    _rearRightSafe  = true;
}

void EdgeSensorArray::update() {
    bool rawFrontLeft  = (digitalRead(_frontLeftPin)  == _floorActiveLevel);
    bool rawFrontRight = (digitalRead(_frontRightPin) == _floorActiveLevel);
    bool rawRearLeft   = (digitalRead(_rearLeftPin)   == _floorActiveLevel);
    bool rawRearRight  = (digitalRead(_rearRightPin)  == _floorActiveLevel);

    // Shift history registers
    _frontLeftHistory  = ((_frontLeftHistory  << 1) | (rawFrontLeft  ? 1 : 0)) & 0x07;
    _frontRightHistory = ((_frontRightHistory << 1) | (rawFrontRight ? 1 : 0)) & 0x07;
    _rearLeftHistory   = ((_rearLeftHistory   << 1) | (rawRearLeft   ? 1 : 0)) & 0x07;
    _rearRightHistory  = ((_rearRightHistory  << 1) | (rawRearRight  ? 1 : 0)) & 0x07;

    // Filter decision: consensus over last 3 samples
    if (_frontLeftHistory == 0x07)       _frontLeftSafe = true;
    else if (_frontLeftHistory == 0x00)  _frontLeftSafe = false;

    if (_frontRightHistory == 0x07)      _frontRightSafe = true;
    else if (_frontRightHistory == 0x00) _frontRightSafe = false;

    if (_rearLeftHistory == 0x07)        _rearLeftSafe = true;
    else if (_rearLeftHistory == 0x00)   _rearLeftSafe = false;

    if (_rearRightHistory == 0x07)       _rearRightSafe = true;
    else if (_rearRightHistory == 0x00)  _rearRightSafe = false;
}
