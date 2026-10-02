# SentiRover: Autonomous Table-Safe Robotic System

[![Board: ATmega328P](https://img.shields.io/badge/Board-ATmega328P%20%2F%20Arduino%20UNO-blue.svg)](https://www.microchip.com/)
[![Language: C++11](https://img.shields.io/badge/Language-C%2B%2B11-green.svg)](https://en.cppreference.com/)
[![Sensors: 4x IR Cliff Protection](https://img.shields.io/badge/Sensors-4x%20IR%20%282%20Front%2C%202%20Rear%29-brightgreen.svg)]()
[![Control: Restricted Manual %2B W Mode](https://img.shields.io/badge/Control-Restricted%20Manual%20%2B%20%27W%27%20Toggle-purple.svg)]()
[![Architecture: Non--Blocking FSM](https://img.shields.io/badge/Architecture-Non--Blocking%20FSM%20%2B%20TaskScheduler-orange.svg)]()
[![License: MIT](https://img.shields.io/badge/License-MIT-brightgreen.svg)](LICENSE)

An industrial-grade, non-blocking autonomous desktop rover featuring **full 360-degree table-edge safety** with **4 IR downward sensors (2 Front, 2 Rear)**, obstacle navigation, restricted manual controls, and remote telemetry control over Bluetooth.

---

## 📸 System Overview & Control Modes

**SentiRover** operates in two modes, toggled instantly over Bluetooth via the `'W'` signal:
- **Autonomous Mode (`'W'` Toggle / `'A'`)**: Full obstacle scanning and 360° cliff recovery.
- **Restricted Manual Mode (`'W'` Toggle / `'M'`)**: Direct Bluetooth directional commands (`'F'`, `'B'`, `'L'`, `'R'`, `'S'`) enforced by **real-time safety interlocks**.

---

## 🛑 Real-Time Movement Safety Restrictions

Even in manual driving mode, SentiRover will **refuse dangerous commands** and **actively override motor drive** to prevent catastrophic falls or collisions:

1. **Forward Command Restriction (`'F'`)**:
   - **Cliff Interlock**: If Front-Left or Front-Right IR sensors detect a cliff edge, `'F'` is rejected, motors are stopped immediately, and `"BLOCKED: FRONT CLIFF"` telemetry is returned.
   - **Obstacle Interlock**: If the ultrasonic sensor measures an obstacle $\le 25\text{ cm}$, `'F'` is rejected, motors are stopped, and `"BLOCKED: OBSTACLE AHEAD"` is returned.
2. **Backward Command Restriction (`'B'`)**:
   - **Cliff Interlock**: If Rear-Left or Rear-Right IR sensors detect a cliff edge behind the vehicle, `'B'` is rejected, motors are killed immediately, and `"BLOCKED: REAR CLIFF"` is returned.
3. **Active Real-Time Interlock**:
   - If the robot is already moving forward/backward and a cliff or obstacle appears under its path, the motors are **killed instantly (0 ms delay)** by the 100 Hz safety task.

---

## 📶 Bluetooth Command Protocol (HC-05)

Commands are transmitted over RFCOMM Bluetooth serial at **9600 Baud**.

| Character | Action / Function | Safety Restrictions Enforced |
| :--- | :--- | :--- |
| **`'W'`** | **Toggle Operating Mode** | Toggles between **Autonomous** & **Manual** mode |
| **`'F'`** | **Move Forward** | Blocked if Front Cliff or Obstacle detected ($\le 25\text{ cm}$) |
| **`'B'`** | **Move Backward** | Blocked if Rear Cliff detected |
| **`'L'`** | **Turn Left** | Active spin turn |
| **`'R'`** | **Turn Right** | Active spin turn |
| **`'S'`** | **Stop Motors** | Immediate motor power cut |
| **`'A'`** | **Force Autonomous Mode**| Initiates full FSM self-navigation |
| **`'M'`** | **Force Manual Mode** | Enables restricted manual driving |

---

## 🔌 Hardware Connections & Pinout Matrix

| Subsystem | Signal Name | MCU Pin | Hardware Connection | Notes / Description |
| :--- | :--- | :--- | :--- | :--- |
| **Motor Driver (L298N)** | **Left Motor Forward** | `Pin 3` | L298N IN1 | Left Motor FWD (PWM on D3) |
| | **Left Motor Backward** | `Pin 2` | L298N IN2 | Left Motor REV (D2) |
| | **Right Motor Forward** | `Pin 5` | L298N IN3 | Right Motor FWD (PWM on D5) |
| | **Right Motor Backward** | `Pin 4` | L298N IN4 | Right Motor REV (D4) |
| **Ultrasonic & Servo** | **Servo Signal** | `Pin 6` | SG90 Servo PWM | Pan Servo Mount (Timer 0 PWM) |
| | **Ultrasonic Trig** | `Pin 13` | HC-SR04 Trig | Ultrasonic Trigger Pulse Output |
| | **Ultrasonic Echo** | `Pin 12` | HC-SR04 Echo | Ultrasonic Pulse Return Input |
| **IR Cliff Sensors** | **Left Forward IR** | `Pin 8` | Front-Left IR Module | Downward Front-Left Edge Sensor |
| | **Right Forward IR** | `Pin 9` | Front-Right IR Module| Downward Front-Right Edge Sensor |
| | **Right Backward IR** | `Pin 10` | Rear-Right IR Module | Downward Rear-Right Edge Sensor |
| | **Left Backward IR** | `Pin 11` | Rear-Left IR Module  | Downward Rear-Left Edge Sensor |
| **Bluetooth (HC-05)** | **SoftwareSerial RX** | `Pin A4` | HC-05 TX | Telemetry Receiver |
| | **SoftwareSerial TX** | `Pin A5` | HC-05 RX (resistor divider) | Telemetry Transmitter |

---

## 🏗 Directory Structure & Code Architecture

```text
SentiRover/
├── SentiRover/
│   ├── SentiRover.ino         # Main entry point (Setup & Supervisory loop tick)
│   ├── Config.h               # Hardware pinouts, timing parameters, & macros
│   ├── TaskScheduler.h/.cpp   # Deterministic non-blocking cooperative task kernel
│   ├── IMotorController.h     # Abstract interface for motor driver abstraction
│   ├── MotorDriver.h/.cpp     # 4-pin L298N driver with PWM acceleration ramping
│   ├── EdgeSensorArray.h/.cpp # Debounced 4x IR cliff sensor array (2 Front, 2 Rear)
│   ├── DistanceScanner.h/.cpp # Non-blocking ultrasonic & servo sweep state machine
│   ├── BluetoothLink.h/.cpp   # Serial telecommand parser with heartbeat watchdog
│   └── RobotController.h/.cpp # Master FSM with 'W' mode toggle & safety interlocks
└── README.md                  # System technical documentation
```

---

## 🚀 Building and Flashing

### Using Arduino IDE
1. Open [SentiRover.ino](file:///d:/work/SentiRover/SentiRover/SentiRover.ino).
2. Select Board: **Arduino Uno** or **Arduino Nano** (ATmega328P).
3. Set Port to your connected USB serial port.
4. Click **Upload**.

---

## 📜 License
Released under the [MIT License](LICENSE). Engineered for robustness, scalability, and high-performance embedded robotics.
