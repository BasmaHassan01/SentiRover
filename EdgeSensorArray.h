/**
 * @file EdgeSensorArray.h
 * @brief Debounced 4x IR Downward Edge Sensor Array (2 Front, 2 Rear)
 * @author Senior Embedded Systems Engineer
 */

#ifndef EDGE_SENSOR_ARRAY_H
#define EDGE_SENSOR_ARRAY_H

#include <Arduino.h>

class EdgeSensorArray {
public:
    EdgeSensorArray(uint8_t frontLeftPin, uint8_t frontRightPin,
                    uint8_t rearLeftPin, uint8_t rearRightPin,
                    int floorActiveLevel = LOW, uint8_t debounceSamples = 3);

    void begin();
    
    /**
     * @brief Periodic sampling task to update debounced sensor states.
     * Called by TaskScheduler at 100Hz.
     */
    void update();

    bool isFrontLeftSafe() const  { return _frontLeftSafe; }
    bool isFrontRightSafe() const { return _frontRightSafe; }
    bool isRearLeftSafe() const   { return _rearLeftSafe; }
    bool isRearRightSafe() const  { return _rearRightSafe; }

    bool isFrontSafe() const { return _frontLeftSafe && _frontRightSafe; }
    bool isRearSafe() const  { return _rearLeftSafe && _rearRightSafe; }
    bool isAllSafe() const   { return isFrontSafe() && isRearSafe(); }

private:
    uint8_t _frontLeftPin, _frontRightPin;
    uint8_t _rearLeftPin, _rearRightPin;
    int _floorActiveLevel;
    uint8_t _debounceSamples;

    // Rolling sample accumulators (3-bit window)
    uint8_t _frontLeftHistory;
    uint8_t _frontRightHistory;
    uint8_t _rearLeftHistory;
    uint8_t _rearRightHistory;

    bool _frontLeftSafe;
    bool _frontRightSafe;
    bool _rearLeftSafe;
    bool _rearRightSafe;
};

#endif // EDGE_SENSOR_ARRAY_H
