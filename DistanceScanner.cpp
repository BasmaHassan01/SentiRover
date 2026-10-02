/**
 * @file DistanceScanner.cpp
 * @brief Non-Blocking Ultrasonic & Servo Surroundings Scanner Implementation (Matching Hardware Snippet Calibration)
 * @author Senior Embedded Systems Engineer
 */

#include "DistanceScanner.h"

DistanceScanner::DistanceScanner(uint8_t trigPin, uint8_t echoPin, uint8_t servoPin,
                                 int centerAngle, int leftAngle, int rightAngle)
    : _trigPin(trigPin), _echoPin(echoPin), _servoPin(servoPin),
      _centerAngle(centerAngle), _leftAngle(leftAngle), _rightAngle(rightAngle),
      _state(SCAN_IDLE), _stateTimer(0) {
    _lastResult = {NO_ECHO, NO_ECHO, NO_ECHO, false};
}

void DistanceScanner::begin() {
    pinMode(_trigPin, OUTPUT);
    pinMode(_echoPin, INPUT);
    digitalWrite(_trigPin, LOW);

    _servo.attach(_servoPin);
    centerServo();
}

void DistanceScanner::centerServo() {
    _servo.write(_centerAngle);
}

int DistanceScanner::getDistance() {
    return measureDistanceInternal();
}

int DistanceScanner::measureDistanceInternal() {
    digitalWrite(_trigPin, LOW);
    delayMicroseconds(2);

    digitalWrite(_trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(_trigPin, LOW);

    // Timeout of 20000us limits pulseIn execution to <= 20ms maximum
    long duration = pulseIn(_echoPin, HIGH, 20000UL);

    if (duration == 0) {
        return NO_ECHO;
    }

    // Exact formula from verified hardware code snippet (duration / 58.2)
    return (int)(duration / 58.2);
}

void DistanceScanner::startScan() {
    _state = SCAN_MOVING_TO_CENTER;
    _stateTimer = millis();
    _lastResult.isComplete = false;
    _servo.write(_centerAngle);
}

void DistanceScanner::updateScan() {
    if (_state == SCAN_IDLE || _state == SCAN_DONE) {
        return;
    }

    unsigned long now = millis();

    switch (_state) {
        case SCAN_MOVING_TO_CENTER:
            if (now - _stateTimer >= 200) { // Non-blocking travel time
                _state = SCAN_READING_CENTER;
            }
            break;

        case SCAN_READING_CENTER:
            _lastResult.centerDistance = measureDistanceInternal();
            _servo.write(_rightAngle); // 0 degrees
            _stateTimer = now;
            _state = SCAN_MOVING_TO_RIGHT;
            break;

        case SCAN_MOVING_TO_RIGHT:
            if (now - _stateTimer >= 400) {
                _state = SCAN_READING_RIGHT;
            }
            break;

        case SCAN_READING_RIGHT:
            _lastResult.rightDistance = measureDistanceInternal();
            _servo.write(_leftAngle); // 180 degrees
            _stateTimer = now;
            _state = SCAN_MOVING_TO_LEFT;
            break;

        case SCAN_MOVING_TO_LEFT:
            if (now - _stateTimer >= 500) {
                _state = SCAN_READING_LEFT;
            }
            break;

        case SCAN_READING_LEFT:
            _lastResult.leftDistance = measureDistanceInternal();
            _servo.write(_centerAngle); // 90 degrees
            _stateTimer = now;
            _state = SCAN_RETURNING_CENTER;
            break;

        case SCAN_RETURNING_CENTER:
            if (now - _stateTimer >= 300) {
                _lastResult.isComplete = true;
                _state = SCAN_DONE;
            }
            break;

        default:
            _state = SCAN_IDLE;
            break;
    }
}
