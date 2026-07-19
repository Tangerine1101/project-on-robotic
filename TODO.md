# Handover TODO

Ordered by priority. Items reference findings in [`doc/bug-report.md`](doc/bug-report.md). Status updated 2026-07-12 after implementing item 1, a firmware machine-interface cleanup pass, and (later the same day) items 1b/1c below.

## 1b. [DONE 2026-07-12] Firmware: time-synchronized moveto for joints 1–3
`cmd_moveto` now stretches each commanded stepper's speed/acceleration profile so all commanded axes arrive at the same moment (`motorControl::movetoSync` in `src/control.cpp`; the slowest axis runs the nominal profile, the others are slowed to match — trapezoid shape preserved). Relative moves (`cmd_move`) and calibration reset to nominal profiles first (`resetProfile()`). Joint 4/grip are hobby servos with no speed API and stay unsynchronized. Verified on the real Due: a 30°/40°/10° three-axis moveto converged within one status tick (~150 ms measurement tail), where the old firmware would have finished the 10° axis ~1.4 s early; relative-move and calibration timing unchanged after.

## 1d. [DONE 2026-07-18] Limit-switch state in the MCU→PC packet
Every status packet now carries a `limitSwitches` byte (`sendPackage` grew 24→25
bytes; the PC and firmware must be flashed together). Bit0/1/2 = ref switches
A/B/C (joints 1/2/3), raw polarity (1 = open, 0 = at switch). `motorControl::limitSwitchMask()`
composes it; every `sendingPackage(...)` call passes it. PC side: `SerialLink.limit_switches`
(decoded to per-switch "touched?" booleans), shown in `tools/cli.py`'s `position`
and `monitor`, and as A/B/C indicators on the dashboard (`/api/status`). Verified
on the real Due: 25-byte packet frames cleanly (no checksum drops), joints still
decode correctly, and switch states track the actual GPIO (with nothing wired,
A/B read open and C read "at limit" — pin 34/refC is grounded or has a pressed
switch; worth a hardware check).

## 1e. [DONE 2026-07-18] Servo (joint4/grip) move fix
`moveto -d`/`-e` commanded the servos correctly in firmware (feedback tracked)
but the horn didn't physically move. Two root causes addressed: (1) the servos
were `attach()`ed only in the global `motorControl` constructor, which runs
**before** the Arduino core's `init()` (global ctors run pre-`main()`), so pulse
generation could fail to start — servos are now (re)attached in `init()`, which
`setup()` calls after core init; (2) the 50 kHz stepper ISR (`TC6`) shared the
default NVIC priority with the Servo library's pulse timer (`TC3`) and could
starve it — `TC6` is now demoted (`NVIC_SetPriority(TC6_IRQn, 8)`) so servo
edges preempt. Also added an experimental `DETACH_SERVO_AFTER_TASK` macro
(`config.h`, off by default) that detaches each servo `SERVO_SETTLE_MS` after a
move. Note: `moveto -e` is still pinned to 148° by the grip clamp (bug 1.4) —
that's expected, not a servo fault. **Physical motion still needs the owner to
eyeball** (no servo-position feedback exists; `servoAngle()` reports the
commanded pulse only).

## 1f. [DONE 2026-07-18] `goto` command + camera↔robot calibration
`tools/cli.py` gained a `goto <x_mm> <y_mm> <z_mm>` command that runs the inverse
kinematics (`kinematics.solve_ik_first`) and moves joints 1–4 to a Cartesian
point (grip untouched); prints `unreachable` when there's no safe IK solution.
New `robot/camera_calibrate.py` derives the camera→robot 2D transform from 3
typed point correspondences (robot `X Y` mm + pixel `u v`) plus a `z_flip`
confirmation, fitting a similarity (`r = a*p + b`) and writing
`x_0_mm/y_0_mm/pixels_per_mm/rotate/z_flip` into `config.yaml`'s `vision:` block.
The camera pipeline was normalized to **mm** end-to-end: `vision.py` now applies
the calibrated transform and emits detections in mm (`rotate`/`z_flip` are
actually consumed), and `planner.py` dropped its `*10` cm→mm conversions.
Config keys renamed `x_0_cm/y_0_cm/pixels_per_cm` → `x_0_mm/y_0_mm/pixels_per_mm`.

## 1c. [DONE 2026-07-12] Web dashboard
`robot/dashboard.py` + `robot/static/` (Flask, served by `main.py` on port 8000, see README): live camera stream with detection boxes, rolling 5-joint angle chart from the 20 Hz stream, auto-mode toggle, and a manual joint-move form (x y z w e = joint1..grip, degrees). Manual moves are blocked in auto mode. Verified end-to-end in a browser against the real Due + camera. Side fix: `SerialLink` now supports multiple packet listeners, so the dashboard's history recorder, `tools/cli.py monitor`, and `calibrate()`'s internal watcher no longer displace each other.

## 1. [DONE 2026-07-12] De-ROSification — replace ROS2 with plain Python, Windows-native

`robot/` now contains a single Python app that replaces the whole ROS2 stack — no container, no `colcon build`, no `ros2` CLI:
```
robot/
├── main.py           # entry point: python main.py [--no-vision] [--manual]
├── config.yaml       # COM/serial port, camera index, zones, offsets (ported from params.yaml)
├── requirements.txt
├── devices.py        # auto-detects the MCU (2341:003D) and camera (0C46:636B) by USB ID
├── serial_link.py    # MCU binary protocol, single-flight command queue
├── planner.py        # pick-and-place state machine (port of planner_node.py)
├── vision.py         # camera + YOLO thread (port of vision.py)
├── kinematics.py     # IK math, ported as-is
├── best.pt
└── tools/
    ├── cli.py           # interactive manual control (replaces the old HumanInterface + package_gen.py/package_receive.py)
    └── test_camera.py   # camera + YOLO preview, no MCU needed
```
Verified end-to-end against the real Due + camera attached to this dev machine (see `doc/bug-report.md` §5 for the hardware trace): device auto-detection, the binary serial protocol (moveto/grip/release/position/calibrate), and the full pick-and-place state cycle (dry-run with a fake detection, since no real onion/garlic/lemon was on hand) all ran correctly. `ros2/` is kept only as a deprecated reference (see README) — not deleted, since the owner may want to diff against it later, but it should not be developed further.

**Two additional bugs were found and fixed during this port** (see doc/bug-report.md 1.6 and 5.6):
- the double +90° offset on joint 4 — removed the extra add in the planner, kinematics.py keeps the one true offset.
- `refCalibrate()`'s multi-phase timeout behavior, which reports a misleading "failed" status ~14s before the firmware is actually done and able to accept new commands — this is very likely the root cause of the "robot sometimes runs a command late/twice" symptom reported from memory. Fixed on the Python side (waits for the firmware to truly go idle); the firmware itself still has the underlying quirk (see item 5 below).

Remaining loose end: `robot/devices.py`'s Windows camera auto-detection (WMI-based) is written but **not verified on real Windows hardware** — this dev environment is Linux-only. If it misbehaves, set `vision.camera_index` directly in `config.yaml` as a fallback (always works).

## 2. [DONE 2026-07-12] Firmware: HumanInterface removed, machine interface only
The text-based CLI mode (`commandHandle`/`readFrom`, bugs 1.2/1.3) has been deleted rather than patched — `cmd_ros2Interface` was renamed `cmd_machineInterface` (same byte `'S'`), and `robot/tools/cli.py` is now the CLI. Firmware compiles (`pio run`) and was flashed and tested against the real Due.

## 3. [OPEN — needs hardware] Fix the grip: `gripOpen` and `gripClose` are the same angle (bug-report 1.4)
Left as-is per the owner's decision (a wrong guess could strain the servo) — `config.h:26` now has a `TODO` comment on it. Whoever has the arm assembled needs to measure the real open/close angles and update both `gripOpen`/`gripClose` (this also fixes `gripMin`/`gripMax`, which currently lock the gripper to exactly 148° for any client, not just the shortcut grip/release commands — see bug-report 1.4).

## 4. [DONE 2026-07-12] Joint-4/grip position reporting fixed (bug-report 1.5)
`get_angles()` now reads `servoAngle(joint4)`/`servoAngle(grip)` directly instead of the never-updated callback fields (which were removed, along with the dead `moveServo()`). Verified on hardware — status packets now report real joint-4/grip angles.

## 5. [OPEN — firmware safety gaps, flagged not fixed] Motor-control review findings (bug-report §5)
Found during a focused review requested by the owner; **not changed** since they touch active motion-safety logic on hardware the owner considers stable, and need a judgment call:
- **5.1 — `cmd_abort` does not actually stop the motors.** No real e-stop exists in the firmware today. Highest-priority item here.
- **5.6 — `refCalibrate()` reports a misleading failure ~14s before it's actually done**, during which the MCU won't accept new commands. Worked around client-side (see item 1); the real fix belongs in `refCalibrate()` itself (stop after the first phase timeout instead of continuing through all three).
- 5.2 — per-axis limit violations (as opposed to the joint2/3 collision limit) hang silently instead of raising an error.
- 5.3 — `avoidCollision()` is dead code; the real collision guard is a separate inline duplicate in `run()`.

## 6. [OPEN — needs hardware] Resolve joint-4 double +90° offset (bug-report 1.6)
Fixed on the Python side already (see item 1) — `kinematics.py` keeps the one +90 shift, `planner.py` no longer adds a second one. Still needs confirmation on real hardware: command a known joint-4 angle and check what the arm actually does, since this was never tested with a physical arm attached.

## 7. Repo hygiene (bug-report §4)
- [DONE] `.gitignore` added; committed `__pycache__` removed; `package_gen.py`/`package_receive.py` deleted (superseded by `robot/tools/cli.py`).
- [OPEN] `best.pt` is still duplicated inside `ros2/` (`ros2/old/best.pt` and `ros2/src/robot_pkg/pkg/init/best.pt`) — left alone since `ros2/` is being kept only as a frozen reference, not developed further. `robot/best.pt` is the one copy that matters going forward.
- [OPEN, moot if `ros2/` is ever deleted] placeholder license in `ros2/src/robot_pkg/setup.py`, hardcoded host path in `ros2/.devcontainer/devcontainer.json`.

## 8. Write a Windows setup guide for the successor
Still open. Should cover: Python install, `pip install -r robot/requirements.txt`, finding the correct COM port (Device Manager) if auto-detect ever fails, and flashing the firmware (PlatformIO `pio run -t upload`, or Arduino IDE per the README's C++ section).
