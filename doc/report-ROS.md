# ROS2 Package Report

## 1. Overview
The ROS2 software stack is contained within the `pkg` package. It provides a complete interface for controlling the robotic arm, including hardware drivers, vision processing, and high-level planning.

## 2. Package Structure
The package follows a standard ROS2 python package structure:

*   **Package Name:** `pkg`
*   **Dependencies:** `rclpy`, `std_msgs`, `sensor_msgs` (implied), `robot_interfaces`, `std_srvs`, `action_msgs`, `python3-numpy`.
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
    *   `port`: Serial device path (e.g., `/dev/ttyACM0`). Defines which USB port the Arduino is connected to.
    *   `baudrate`: Communication speed (default: 115200). Must match the `BAUDRATE` in the MCU's `config.h`.
    *   `tolerance`: Allowed deviation for joint angles (default: 1.5 degrees). Used to determine if a joint has reached its target.
    *   `timeout_sec`: Timeout for serial communication (default: 10.0 seconds).
    *   `read_frequency`: Rate at which the driver polls the serial port for updates (default: 20.0 Hz).

### 3.2. Vision Node (`vision_node`)
*   **Entry Point:** `pkg.vision:main`
*   **Function:** Handles image processing tasks, likely for object detection and localization.
*   **Parameters:**
    *   `camera_dev`: Video device index (default: 4). Use `v4l2-ctl --list-devices` to find the correct index.
    *   `x_0_cm`, `y_0_cm`: Physical offset of the camera frame relative to the robot base frame (in cm).
        *   `x_0_cm`: (+) up, (-) down.
        *   `y_0_cm`: (+) left, (-) right.
    *   `pixels_per_cm`: Calibration factor (pixels/cm) used to convert image coordinates to physical coordinates.
    *   `conf_threshold`: Minimum confidence score (0.0 to 1.0) for the YOLO model to accept a detection (default: 0.7).
    *   `stream_frequency`: Target frame rate for the video stream (default: 24 Hz).

### 3.3. Planner Node (`planner_node`)
*   **Entry Point:** `pkg.planner_node:main`
*   **Function:** The "brain" of the operation. It coordinates the arm's movements based on task logic.
*   **Logic:**
    *   Defines classification zones for sorting objects (e.g., "Onion" vs "Garlic").
    *   Manages the pickup and drop-off sequences.
*   **Parameters:**
    *   `pickup_height_mm`: The Z-height (in mm) at which the gripper should attempt to grasp an object.
    *   `zone_onion_{x,y}`: Target drop-off coordinates (in mm) for objects classified as "Onion".
    *   `zone_garlic_{x,y}`: Target drop-off coordinates (in mm) for objects classified as "Garlic".
    *   `zone_lemon_{x,y}`: Target drop-off coordinates (in mm) for objects classified as "Lemon".
    *   `zone_radius_mm`: Exclusion radius (default: 70.0 mm). Objects detected within this radius of a zone center are ignored to prevent re-processing already sorted items.

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

### 4.3. Message: `DetectedObject`
Represents a single object detected by the vision system.
*   `string name`: The class name of the object (e.g., "onion", "garlic").
*   `float64 x`: The X coordinate of the object in the robot's frame (mm).
*   `float64 y`: The Y coordinate of the object in the robot's frame (mm).
*   `float64 z`: The Z coordinate of the object in the robot's frame (mm).

### 4.4. Message: `ObjectList`
A container for multiple detected objects, typically published by the vision node.
*   `DetectedObject[] objects`: An array of `DetectedObject` messages.

### 4.5. Service: `MoveTo`
A service to request the robot to move its end-effector to a specific Cartesian coordinate.
*   **Request:**
    *   `float64 x`: Target X coordinate (mm).
    *   `float64 y`: Target Y coordinate (mm).
    *   `float64 z`: Target Z coordinate (mm).
*   **Response:**
    *   `bool success`: True if the movement was successful, False otherwise.

## 5. Launch System

### `run.launch.py`
This is the main entry point for starting the robot. It performs the following:
1.  **Parameter Loading:** Locates `config/params.yaml` in the package share directory.
2.  **Node Startup:** Launches `serial_driver`, `vision_node`, and `planner_node` concurrently.
3.  **Configuration:** Passes the loaded parameters to all nodes.

### `novision.launch.py`
(Inferred) Likely a variant of the run launch file that excludes the `vision_node` for testing mechanical control without camera input.

[driver-1] [ERROR] [1768369246.044121387] [serial_driver]: ⏰ Timed Out. Current: [0.0, 39.990875244140625, -9.985401153564453, 100.0, 100.0]
[driver-1] [INFO] [1768369246.050316013] [serial_driver]: ✅ Ack Received: done