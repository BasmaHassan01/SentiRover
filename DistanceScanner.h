/**
 * @file DistanceScanner.h
 * @brief Non-Blocking Ultrasonic & Servo Surroundings Scanner
 * @author Senior Embedded Systems Engineer
 */

#ifndef DISTANCE_SCANNER_H
#define DISTANCE_SCANNER_H

#include <Arduino.h>
#include <Servo.h>

struct ScanResult {
    int leftDistance;
    int centerDistance;
    int rightDistance;
    bool isComplete;
};

enum ScanState {
    SCAN_IDLE,
    SCAN_MOVING_TO_CENTER,
    SCAN_READING_CENTER,
    SCAN_MOVING_TO_LEFT,
    SCAN_READING_LEFT,
    SCAN_MOVING_TO_RIGHT,
    SCAN_READING_RIGHT,
    SCAN_RETURNING_CENTER,
    SCAN_DONE
};

class DistanceScanner {
public:
    static const int NO_ECHO = 999;

    DistanceScanner(uint8_t trigPin, uint8_t echoPin, uint8_t servoPin,
                    int centerAngle = 90, int leftAngle = 180, int rightAngle = 0);

    void begin();
    
    /**
     * @brief Measures distance straight ahead (uses non-blocking pulse calculation).
     */
    int getDistance();

    /**
     * @brief Initiates an asynchronous non-blocking sweep scan.
     */
    void startScan();

    /**
     * @brief Ticked by TaskScheduler to step through sweep state machine without blocking.
     */
    void updateScan();

    bool isScanning() const { return _state != SCAN_IDLE && _state != SCAN_DONE; }
    ScanResult getLatestScanResult() const { return _lastResult; }

    void centerServo();

private:
    uint8_t _trigPin, _echoPin, _servoPin;
    int _centerAngle, _leftAngle, _rightAngle;
    Servo _servo;

    ScanState _state;
    unsigned long _stateTimer;
    ScanResult _lastResult;

    int measureDistanceInternal();
};

#endif // DISTANCE_SCANNER_H
