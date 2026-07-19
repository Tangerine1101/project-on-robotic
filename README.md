
# C++ code (MCU firmware)
Build with [PlatformIO](https://platformio.org/) (`platformio.ini` is at the repo root):
```
pio run              # compile
pio run -t upload    # flash to the Arduino Due
```
If you prefer the Arduino IDE instead: copy all files from "src" and "include" folders into a folder named "main", rename main.cpp to main.ino and open it with Arduino IDE.\
note: you need the AccelStepper and Servo libraries and a C++17 compiler either way.

# Python app (PC side)
The PC side used to be a ROS2 stack (still present, unused, in `ros2/` — see below); it has
been replaced with a single Python app in `robot/`, runnable directly on Windows or Linux
with no container/ROS install needed.

```
cd robot
pip install -r requirements.txt
python main.py                 # full auto: vision + planner + serial + web dashboard
python main.py --no-vision     # no camera; drive the arm manually
python main.py --manual        # planner stays idle; drive via the dashboard or the CLI below
python main.py --no-dashboard  # run without the web dashboard
python tools/cli.py            # manual command-line control (moveto/goto/grip/calibrate/monitor/...)
python tools/test_camera.py    # camera + YOLO preview, no MCU needed
python camera_calibrate.py     # derive the camera->robot transform and write it to config.yaml
```
`tools/cli.py` also has a **`goto <x_mm> <y_mm> <z_mm>`** command that runs the
inverse kinematics (`kinematics.solve_ik_first`, mm in → joint degrees out) and
moves joints 1–4 to that Cartesian point (grip untouched); it prints
`unreachable` if no safe IK solution exists.

The camera→robot mapping is calibrated by **`camera_calibrate.py`**: it opens the
camera (YOLO runs only as a visual reference), lets you read pixel coordinates
off a live preview, then asks for 3 point correspondences — for each of A, B, C
the robot `X Y` (mm) and the matching pixel `u v` — plus a `z_flip` (is the
camera z-axis flipped vs the robot). It fits a 2D similarity transform and writes
`x_0_mm` / `y_0_mm` / `pixels_per_mm` / `rotate` / `z_flip` into the `vision:`
block of `config.yaml`, which `vision.py` applies to emit detections in **mm**
(the whole camera pipeline is mm now, matching the IK/planner).
The MCU and camera are auto-detected by USB ID (see `robot/devices.py`); override
`serial.port` / `vision.camera_index` in `robot/config.yaml` if auto-detect ever
picks the wrong device.

## Web dashboard
`main.py` serves a dashboard at **http://localhost:8000** (host/port in the
`dashboard:` section of `robot/config.yaml`; also reachable from other machines
on the LAN by default). Two tabs:

- **Dashboard** — live camera stream with YOLO detection boxes, plus a rolling
  chart of all five joint angles (from the MCU's 20 Hz status stream).
- **Manual control** — a toggle for auto mode (the pick-and-place sorting
  cycle) and a manual joint-move form. The fields **x y z w e** are absolute
  joint angles in degrees for **joint1 joint2 joint3 joint4 grip** respectively
  (not Cartesian coordinates); leave a field blank to keep that joint in place.
  Manual moves are rejected while auto mode is on. Note the grip (`e`) is
  clamped to 148° by the firmware until the real open/close angles are
  measured (doc/bug-report.md 1.4).

See `doc/bug-report.md` for known issues (some need real hardware to confirm) and
`TODO.md` for the current handover checklist.

## `ros2/` (deprecated)
The original ROS2 Humble + Docker implementation is kept for reference but is no
longer maintained or required — everything it did is now in `robot/`.

# list of commands and errors:
The MCU only speaks the binary "machine interface" protocol below (the old text-based
"HumanInterface" CLI mode has been removed from the firmware; use `robot/tools/cli.py`
for interactive/manual control instead).

|Define|CommandID| Function
|------|---------|----
|cmd_none            |~                       |
|cmd_move            |M                       |move joints that are set in the bitmask a **relative** angle
|cmd_moveto          |A                       |move joints that are set in the bitmask to the **Absolute** angle; joints 1-3 run **time-synchronized** (their speed/acceleration profiles are stretched so all commanded steppers arrive together; joint 4/grip are servos with no speed API and move immediately)
|cmd_position        |P                       |report joint angles (replies with a `P`/`D` packet carrying the current angles); like every MCU→PC packet it also carries the 3 limit-switch states (see `limitSwitches` below)
|cmd_currentPos      |C                       |set joints' current angle to the desired angle, without moving
|cmd_grip            |G                       |close grip
|cmd_release         |R                       |open grip
|cmd_moveref         |F                       |calibrate by moving until the arm reaches the limit switches (reference point) — see doc/bug-report.md 5.6 for a quirk in how this reports completion
|cmd_machineInterface|S                       |(reserved; the binary interface is now always active, this command is a no-op)
|cmd_abort           |X                       |abort process — **see doc/bug-report.md 5.1: not fully implemented, does not reliably stop motion**
|cmd_invalid         |&
|error_none         | 0
|error_invalid_axis|1||the motors or joints that called in the command are invalid
|error_limitation_breaked|2|| the robot arm's limitation breaked
|error_timeout  |3|| process timeout

A status packet may also carry `processingID == 'E'` (error report); `statusID` in that
case holds the error code above as an ASCII digit (e.g. `'2'` for `error_limitation_breaked`).

# M2M: package send and receive mechanism
## package that send from MCU(Arduino Due in this case) to PC is define as:
```cpp
struct __attribute__((packed)) sendPackage //package that will be send to PC
{
    uint8_t startByte; //check if the first byte is correct
    char processingID; //the processing command character, refer to characters of enum commands(~, M, A, P, C, G, R, F, S, X, &, E, etc)
    char statusID; //status of the command: P(processing), D(done), F(fail)
    float Arguments[maxArguments]; //arguments for each joint
    uint8_t limitSwitches; //bitmask of the 3 reference switches
    uint8_t checksum;

    sendPackage(){
        startByte = NODE_SENDBYTE;
        processingID = '~';
        statusID = '~';
        for (int i = 0; i < maxArguments; i++) {
            Arguments[i] = 0.0; 
        }
        limitSwitches = 0;
        checksum = 0;
    }
};
```
- therefor, a sending package from MCU to PC is a string of [{start byte}, {processID}, {statusID}, {array of angles}, {limitSwitches byte}, {checksum byte}]
- `limitSwitches` is a bitmask of the 3 reference/limit switches: **bit0 = A (joint 1), bit1 = B (joint 2), bit2 = C (joint 3)**. Raw polarity (matches the firmware's `INPUT_PULLUP` active-low wiring): **1 = open / not touched, 0 = arm sitting on that switch**. The PC side (`robot/serial_link.py` → `SerialLink.limit_switches`) decodes this into per-switch "touched?" booleans; `tools/cli.py` shows them in `position` and `monitor`, and the web dashboard shows A/B/C indicators.
## package that received by MCU that send from PC:
```cpp
struct __attribute__((packed)) serialPackage //remote command package
{
    uint8_t startByte; //check if the first byte is correct
    char commandID; //the command character
    uint8_t bitmask; //5 bits to indicate which joints to move
    float Arguments[maxArguments]; //arguments for each joint
    uint8_t checksum; 
};
```
- therefor, the PC sends a package to the MCU as a string of [{start byte}, {commandID}, {bitmask}, {array of angles}, {checksum byte}]
