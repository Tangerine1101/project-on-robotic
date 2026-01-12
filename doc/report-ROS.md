# ROS2 Package Report

## 1. Overview
The ROS2 software stack is contained within the `pkg` package. It provides a complete interface for controlling the robotic arm, including hardware drivers, vision processing, and high-level planning.

## 2. Package Structure
The package follows a standard ROS2 python package structure:

*   **Package Name:** `pkg`
*   **Dependencies:** `rclpy`, `std_msgs`, `sensor_msgs` (implied), `robot_interfaces`.
*   **Launch Files:** Located in `launch/`.
*   **Configuration:** Parameters defined in `config/params.yaml`.

## 3. Nodes

The system consists of three primary nodes, orchestrated by the launch system.

### 3.1. Serial Driver (`serial_driver`)
*   **Entry Point:** `pkg.serial_driver:main`
*   **Function:** Acts as a bridge between the ROS2 network and the Arduino firmware.
*   **Responsibilities:**
    *   Opens the serial port defined by `port` and `baudrate` parameters.
    *   Encodes ROS2 commands into the binary protocol expected by the MCU.
    *   Decodes status packets from the MCU and publishes joint states.
*   **Parameters:**
    *   `port`: Serial device path (e.g., `/dev/ttyACM0`).
    *   `baudrate`: Communication speed (default: 115200).
    *   `read_frequency`: Rate at which the driver polls the serial port.

### 3.2. Vision Node (`vision_node`)
*   **Entry Point:** `pkg.vision:main`
*   **Function:** Handles image processing tasks, likely for object detection and localization.
*   **Parameters:**
    *   `camera_offset_x`, `camera_offset_y`: Physical offset of the camera relative to the robot base.
    *   `pixels_per_cm`: Calibration factor for converting pixel coordinates to real-world units.

### 3.3. Planner Node (`planner_node`)
*   **Entry Point:** `pkg.planner_node:main`
*   **Function:** The "brain" of the operation. It coordinates the arm's movements based on task logic.
*   **Logic:**
    *   Defines classification zones for sorting objects (e.g., "Onion" vs "Garlic").
    *   Manages the pickup and drop-off sequences.
*   **Parameters:**
    *   `pickup_height_mm`: Z-height for grasping.
    *   `zone_onion_{x,y,z}`: Target coordinates for onions.
    *   `zone_garlic_{x,y,z}`: Target coordinates for garlic.
    *   `home_angles`: Default resting position.

## 4. Custom Interfaces (`robot_interfaces`)

The project defines custom ROS2 interfaces to standardize communication between nodes.

### 4.1. Action: `MoveArm`
Used for non-blocking control of the arm joints.
*   **Goal:**
    *   `float64[] targets`: Target angles for the joints.
    *   `int8 bitmask`: Mask to select which joints to move.
*   **Result:**
    *   `bool success`: Whether the movement completed successfully.
    *   `float64[] current_values`: Final joint positions.

### 4.2. Service: `GripCommand`
Used for simple, blocking gripper control.
*   **Request:**
    *   `string command`: Command type ("open", "close", "calibrate").
*   **Response:**
    *   `bool success`: Operation status.

## 5. Launch System

### `run.launch.py`
This is the main entry point for starting the robot. It performs the following:
1.  **Parameter Loading:** Locates `config/params.yaml` in the package share directory.
2.  **Node Startup:** Launches `serial_driver`, `vision_node`, and `planner_node` concurrently.
3.  **Configuration:** Passes the loaded parameters to all nodes.

### `novision.launch.py`
(Inferred) Likely a variant of the run launch file that excludes the `vision_node` for testing mechanical control without camera input.
