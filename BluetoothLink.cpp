/**
 * @file BluetoothLink.cpp
 * @brief Bluetooth Telemetry Link Implementation
 * @author Senior Embedded Systems Engineer
 */

#include "BluetoothLink.h"

BluetoothLink::BluetoothLink(uint8_t rxPin, uint8_t txPin, long baudRate,
                             unsigned long heartbeatTimeoutMs)
    : _serial(rxPin, txPin), _baudRate(baudRate),
      _heartbeatTimeoutMs(heartbeatTimeoutMs), _lastActivityMs(0) {}

void BluetoothLink::begin() {
    _serial.begin(_baudRate);
    _lastActivityMs = millis();
}

bool BluetoothLink::readCommand(char &command) {
    if (!_serial.available()) {
        return false;
    }
    command = _serial.read();
    _lastActivityMs = millis();
    return true;
}

void BluetoothLink::send(const char *message) {
    _serial.println(message);
}

bool BluetoothLink::isHeartbeatTimedOut() const {
    return (millis() - _lastActivityMs) > _heartbeatTimeoutMs;
}

void BluetoothLink::resetHeartbeat() {
    _lastActivityMs = millis();
}
