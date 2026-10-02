/**
 * @file BluetoothLink.h
 * @brief Non-Blocking Serial Telemetry & Control Link with Heartbeat Watchdog
 * @author Senior Embedded Systems Engineer
 */

#ifndef BLUETOOTH_LINK_H
#define BLUETOOTH_LINK_H

#include <Arduino.h>
#include <SoftwareSerial.h>

class BluetoothLink {
public:
    BluetoothLink(uint8_t rxPin, uint8_t txPin, long baudRate = 9600,
                  unsigned long heartbeatTimeoutMs = 3000);

    void begin();
    
    /**
     * @brief Polls incoming serial byte non-blockingly.
     */
    bool readCommand(char &command);

    /**
     * @brief Transmits status payload over Bluetooth.
     */
    void send(const char *message);

    /**
     * @brief Checks whether remote control link has timed out (for manual mode fail-safe).
     */
    bool isHeartbeatTimedOut() const;
    
    void resetHeartbeat();

private:
    SoftwareSerial _serial;
    long _baudRate;
    unsigned long _heartbeatTimeoutMs;
    unsigned long _lastActivityMs;
};

#endif // BLUETOOTH_LINK_H
