# MCU Firmware Report

## 1. Overview
This document details the firmware implementation for the robotic arm project. The firmware is designed to run on an **Arduino Due** (implied by `SerialUSB` and SAM3X8E timer registers) and controls a 4-DOF robotic arm with a gripper. It handles low-level motor control, safety constraints, and communication with a host computer (ROS2 node).

## 2. Hardware Abstraction
The firmware interacts with the following hardware components:

*   **Actuators:**
    *   **Joints 1, 2, 3:** Stepper motors driven by external drivers (e.g., TB6600).
    *   **Joint 4 & Gripper:** Servo motors.
*   **Sensors:**
    *   **Reference Switches (Ref A, B, C):** Used for homing/calibration of the stepper joints.
*   **Controller:** Arduino Due (SAM3X8E ARM Cortex-M3).

### Pin Configuration (`config.h`)
*   **Steppers:** Pulse/Direction interface.
*   **Servos:** PWM interface.
*   **Switches:** Digital inputs with pull-ups.

## 3. Software Architecture

The codebase is organized into three main modules:

### 3.1. Main Application (`main.cpp`)
*   **Initialization:** Sets up serial ports, pins, and interrupts.
*   **Timer Interrupt:** Uses **Timer Counter 6 (TC6)** to trigger the `robot.run()` function every 20 microseconds. This ensures smooth stepper motor generation via the `AccelStepper` library.
*   **Main Loop:**
    *   `operate()`: Handles incoming serial commands.
    *   `ifspin()`: Periodically reports robot status/angles to the host (default 5Hz).
    *   Safety checks: Monitors error flags and triggers emergency braking if limits are breached.

### 3.2. Motor Control (`control.cpp` / `control.h`)
The `motorControl` class encapsulates all motion logic:
*   **Kinematics:** Converts between joint angles (degrees) and motor steps using `GEAR_RATIO`, `MICRO_STEP`, and `STEP_PER_REV`.
*   **Movement:** Supports relative (`move`) and absolute (`moveto`) positioning.
*   **Calibration (`refCalibrate`):** A blocking routine that moves joints towards limit switches to establish a zero reference.
*   **Safety:**
    *   **Soft Limits:** Prevents joints from exceeding defined min/max angles.
    *   **Collision Avoidance:** Specifically checks the sum of Joint 2 and Joint 3 angles to prevent self-collision (`JOINT_2_3_LIMIT`).

### 3.3. Communication (`serialCommand.cpp` / `serialCommand.h`)
The `serialCom` class manages the custom serial protocol used to talk to the ROS2 driver.

*   **Modes:**
    *   **Human Interface:** ASCII-based, human-readable logs and commands.
    *   **ROS2 Interface:** Binary packet-based for Machine-to-Machine (M2M) communication.

## 4. Communication Protocol

### 4.1. Packet Structure
Data is exchanged in fixed-size binary packets (struct `serialPackage` / `sendPackage`):

| Byte Offset | Field | Description |
| :--- | :--- | :--- |
| 0 | `startByte` | Header (0xAA for RX, 0xFE for TX) |
| 1 | `ID` | Command ID (RX) or Processing ID (TX) |
| 2 | `Info` | Bitmask (RX) or Status ID (TX) |
| 3-22 | `Arguments` | Array of 5 floats (Joint angles/params) |
| 23 | `checksum` | XOR checksum of the packet |

### 4.2. Command Set
| Command ID | Char | Description |
| :--- | :--- | :--- |
| `cmd_move` | 'M' | Relative move |
| `cmd_moveto` | 'A' | Absolute move |
| `cmd_position` | 'P' | Report position |
| `cmd_currentPos` | 'C' | Set current position (override) |
| `cmd_grip` | 'G' | Close gripper |
| `cmd_release` | 'R' | Open gripper |
| `cmd_moveref` | 'F' | Homing sequence |
| `cmd_humanInterface`| 'H' | Switch to text mode |
| `cmd_ros2Interface` | 'S' | Switch to binary mode |

## 5. Safety Features
1.  **Limit Switches:** Hardware interrupts (`refA_ISR`, etc.) trigger immediate braking.
2.  **Soft Limits:** Checked before every move in `safety_check()`.
3.  **Interference Check:** Prevents Joint 2 and 3 from colliding (Sum < 60 degrees - offset).
4.  **Watchdog/Timeout:** Calibration routine has a timeout to prevent infinite loops if a switch fails.
