# MCU Firmware Report
## 0. Workflow
nodes:
    vision: detect objects and stream objects data to node planner&kinetic
    planner&kinetic: subscribe objects coordinate. do inverse kinetic to move the arm to nearest object or grab the object or move it to corresponding zone. then stream the angles from to serial_driver node
    serial_driver: subscribe topic angles from node planner&kinetic send command via serial to MCU
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
The firmware is configured with the following pin assignments for the Arduino Due:

| Component | Signal | Pin | Description |
| :--- | :--- | :--- | :--- |
| **Joint 1** | DIR / PUL | 22 / 23 | Stepper Motor 1 |
| | REF | 30 | Limit Switch A |
| **Joint 2** | DIR / PUL | 24 / 25 | Stepper Motor 2 |
| | REF | 32 | Limit Switch B |
| **Joint 3** | DIR / PUL | 26 / 27 | Stepper Motor 3 |
| | REF | 34 | Limit Switch C |
| **Joint 4** | PWM | 28 | Servo Motor (Wrist) |
| **Gripper** | PWM | 29 | Servo Motor (Hand) |
| **System** | RefVolt | 40 | Reference High Signal |

## 3. Software Architecture

The codebase is organized into three main modules:

### 3.1. Main Application (`main.cpp`)
*   **Initialization:** Sets up serial ports, pins, and interrupts.
*   **Timer Interrupt:** Uses **Timer Counter 6 (TC6)** to trigger the `robot.run()` function every 20 microseconds. This ensures smooth stepper motor generation via the `AccelStepper` library.
*   **Main Loop:**
    *   `operate()`: Handles incoming serial commands.
    *   `ifspin()`: Periodically reports robot status/angles to the host (default 20Hz).
    *   Safety checks: Monitors error flags and triggers emergency braking if limits are breached.

### 3.2. Motor Control (`control.cpp` / `control.h`)
The `motorControl` class encapsulates all motion logic:
*   **Kinematics:** Converts between joint angles (degrees) and motor steps using `GEAR_RATIO`, `MICRO_STEP`, and `STEP_PER_REV`.
*   **Movement:** Supports relative (`move`) and absolute (`moveto`) positioning.
*   **Calibration (`refCalibrate`):** A blocking routine that establishes the zero reference for the stepper motors. It executes in three phases:
    1.  **Phase 1:** Simultaneously moves Joint 2 and Joint 3 towards their limit switches.
    2.  **Phase 2:** Moves Joint 1 towards its limit switch.
    3.  **Phase 3:** Moves all joints to the defined `HOME` positions.
    *   *Note:* The routine includes a timeout mechanism (`TIMEOUT_LIMIT`) to prevent infinite blocking if a switch fails.
*   **Safety:**
    *   **Soft Limits:** Prevents joints from exceeding defined min/max angles (e.g., Joint 1: -90° to +16°).
    *   **Collision Avoidance:** Specifically checks the sum of Joint 2 and Joint 3 angles. To prevent self-collision, the condition `Joint2 + Joint3 < 60° - INTERFERENCE_OFFSET` must be met.

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
| `cmd_abort` | 'X' | Emergency stop |

## 5. Safety Features
The firmware implements multiple layers of safety to protect the hardware:

1.  **Limit Switches:**
    *   Configured as hardware interrupts (`refA_ISR`, `refB_ISR`, `refC_ISR`).
    *   Triggering a switch during normal operation (non-calibration) causes immediate braking of the respective joint.
2.  **Soft Limits:**
    *   Checked before every move in `safety_check()`.
    *   Ensures target positions are within the allowed range (e.g., `joint1Min` to `joint1Max`).
3.  **Interference Check (Self-Collision):**
    *   Prevents Joint 2 and 3 from colliding with each other or the base.
    *   Logic: `Joint2_Angle + Joint3_Angle < (60° - 5°)`.
4.  **Watchdog/Timeout:**
    *   The calibration routine has a 10-second timeout (`TIMEOUT_LIMIT`). If switches are not triggered within this window, the operation aborts with `error_timeout`.
5.  **Error Reporting:**
    *   The system reports error states via the `errorFlag` enum:
        *   `error_none` (0)
        *   `error_invalid_axis` (1)
        *   `error_limitation_breaked` (2)
        *   `error_timeout` (3)
