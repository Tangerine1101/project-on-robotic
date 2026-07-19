# Project overview

Robot-arm pick-and-place project, two halves:

- **Firmware** (`src/`, `include/`): C++/PlatformIO for an Arduino Due. Drives 3 stepper joints + 2 servos (joint 4, gripper), talks over USB serial using a fixed-size binary packet protocol — the "machine interface" (see `README.md` for the packet layout and command table). Build/flash with `pio run` / `pio run -t upload` (`platformio.ini` at repo root).
- **PC side** (`robot/`): a plain Python app — `main.py` (entry point), `serial_link.py` (MCU protocol), `planner.py` (pick-and-place state machine), `vision.py` (camera + YOLO), `kinematics.py` (IK), `devices.py` (USB auto-detection), `dashboard.py` + `static/` (Flask web dashboard: camera stream, joint chart, manual control on port 8000), `tools/cli.py` (manual control). Runs natively on Windows or Linux, no container/ROS needed: `pip install -r robot/requirements.txt && python robot/main.py`.

## History: de-ROSification (done 2026-07-12)

This project used to run a ROS2 Humble stack in Docker (`ros2/src/robot_pkg/pkg/`) for the PC side. It was replaced with `robot/` because the project was handed to a successor who works **Windows-only — no Linux, no WSL, no Docker**. `ros2/` is kept only as a frozen reference — **do not develop it further**, and don't assume it reflects the current protocol/bugfixes (`robot/` has since fixed things `ros2/` still has, e.g. the joint-4 double +90° offset).

The MCU serial protocol and the IK math (`kinematics.py`, ported from `FWK_Degree.py` / `binh.m`) are unchanged from the ROS2 era — only the PC-side orchestration layer was replaced.

See [`doc/bug-report.md`](doc/bug-report.md) for known bugs (some need hardware to confirm, some already fixed — check each entry's status tag) and [`TODO.md`](TODO.md) for the current prioritized handover list.

## Conventions

- All documentation and code comments: **English**.
- Chat responses to the project owner: **Vietnamese** (this is the one exception — it applies to conversational replies only, not to anything committed to the repo).
- Existing code mixes Vietnamese and English comments/strings (e.g. `package_gen.py`, `testguide.md`) — when touching those files, prefer converting comments to English rather than adding more Vietnamese, but don't do a drive-by rewrite unrelated to the task at hand.
