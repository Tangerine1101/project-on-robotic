# Bug & Issue Report

Scan date: 2026-07-11, updated 2026-07-12. Covers the whole repo: MCU firmware (`src/`, `include/`), PC-side ROS2 stack (`ros2/`), and root-level Python helper scripts. Each entry lists the file/line, the problem, its impact, and a suggested fix. Entries marked **[HW]** need a physical robot to confirm/verify.

**Status update (2026-07-12):** the firmware's HumanInterface text mode was removed and the machine (binary) interface is now the only interface (renamed `cmd_ros2Interface` → `cmd_machineInterface`, same byte `'S'`). As part of that work: 1.2 and 1.3 are moot (the buggy text-command parser they applied to was deleted, not patched); 1.5 is fixed; 1.8 and 1.9 are resolved (dead function removed, `'E'` documented). 1.4 and 1.6 remain open — 1.6 is being fixed on the Python side during the de-ROSification rewrite (see §5 in `TODO.md`), not in firmware. See new §5 below for additional findings from a focused motor-control review.

---

## 1. Critical / functional bugs

### 1.1 `package_receive.py` crashes on the first byte, and would never sync even if fixed
- [package_receive.py:10](../package_receive.py#L10), [package_receive.py:64](../package_receive.py#L64), [package_receive.py:74](../package_receive.py#L74)
- `NODE_SENDBYTE = 0xFE` is an `int`. Line 64 calls `NODE_SENDBYTE.decode()` → `AttributeError`, crashing immediately on connect. Line 74 compares `byte == NODE_SENDBYTE`, i.e. `bytes == int`, which is always `False` even after the decode bug is fixed — the header-search loop would never find the start byte.
- The comment on lines 8-9 also says the start byte is ASCII `'@'` (0x40); the real value (matching `config.h`'s `NODE_SENDBYTE 0xFE`) is `0xFE`.
- Impact: this tool (the serial traffic monitor used throughout `testguide.md`) does not run at all.
- Fix: compare `byte == bytes([NODE_SENDBYTE])`, drop the `.decode()` call, fix the comment.

### 1.2 [MOOT — HumanInterface removed] Firmware human-interface commands can't match after `toLowerCase()`
- [src/serialCommand.cpp:8-69](../src/serialCommand.cpp#L8-L69) (`serialCom::commandHandle`)
- The incoming line is lowercased (`Command.toLowerCase()`, line 18) but then matched against mixed-case literals: `"currentPos "`, `"humanInterface"`, `"ros2Interface"`, `"testA"`, `"testB"`, `"testC"`. `String::startsWith` is case-sensitive, so none of these branches can ever be reached — they always fall through to `cmd_invalid`.
- README's command table documents `currentPos`, `humanInterface`, `ros2Interface` as working CLI commands; they are not.
- Fix: either compare against the lowercased literal (`"currentpos "`, `"humaninterface"`, `"ros2interface"`, `"testa"`, `"testb"`, `"testc"`), or stop lowercasing the whole command and only lowercase the leading keyword.

### 1.3 [MOOT — HumanInterface removed] `currentPos` argument offset is wrong even ignoring 1.2
- [src/serialCommand.cpp:34-36](../src/serialCommand.cpp#L34-L36)
- `Command.startsWith("currentPos ")` is 11 characters (including the trailing space), but `readFrom(10, Command)` starts parsing at offset 10 — one character early, inside the space/keyword boundary. Should be `readFrom(11, Command)`.

### 1.4 [OPEN] Grip never opens: `gripOpen == gripClose`
- [include/config.h:26-27](../include/config.h#L26-L27)
```cpp
inline const float gripOpen = 148.0;
inline const float gripClose = 148.0;
```
- `cmd_grip`/`cmd_release` (and the ROS `GripCommand` service) move the gripper servo to the exact same angle either way — the gripper physically never opens or closes. **[HW]**
- **Compounding effect found during motor-control review**: `gripMin`/`gripMax` (`config.h:89-90`) are defined as `gripOpen`/`gripClose` — since both are `148.0`, `gripMin == gripMax == 148.0`. The bounds check in `motorControl::moveto('e', angle)` ([control.cpp:152-153](../src/control.cpp#L152-L153)) is `if ((angle <= gripMax) && (angle >= gripMin)) grip.write(angle);` — with `gripMin == gripMax`, this only accepts `angle == 148.0` exactly. Any other grip angle sent via `moveto`/`currentPos` on axis `'e'` is silently rejected (no error flag set, no packet malformed — the write is just skipped). This means the gripper is not just stuck open==closed, it's **fully locked to 148°** for any binary-protocol client, not only through the `cmd_grip`/`cmd_release` shortcuts.
- Fix: give `gripOpen` a distinct angle from `gripClose` (pick a value that matches how the gripper is actually wired); this also fixes `gripMin`/`gripMax` since they derive from the same two constants.

### 1.5 [FIXED 2026-07-12] Joint-4/grip position report is permanently stuck at boot value
- [include/control.h:48-49](../include/control.h#L48-L49), [src/control.cpp:263-269](../src/control.cpp#L263-L269) (`get_angles`), [src/control.cpp:141-158](../src/control.cpp#L141-L158) (`moveto`)
- `angles[3]`/`angles[4]` (joint 4 and grip, reported to the PC via `sendingPackage`/`topicPrint`) are read from `joint4_callback`/`grip_callback`, two member fields initialized once to `REF_D`/`REF_E` and otherwise only written by `moveServo()` — which is never called (`main.cpp:81` calls it commented-out).
- Actual moves happen through `motorControl::moveto('d'/'e', angle)`, which calls `joint4.write(angle)`/`grip.write(angle)` **directly**, bypassing `joint4_callback`/`grip_callback` entirely.
- Net effect: every status packet sent to the PC reports joint 4 and the gripper as permanently at their boot/reference angle, regardless of where they actually are. `serial_driver.py`'s `execute_move_callback` already works around this by explicitly skipping joint index 3 in its tolerance check (`if i == 3: continue`, [serial_driver.py:121](../ros2/src/robot_pkg/pkg/serial_driver.py#L121)) — evidence this was noticed but only patched on the consumer side, not fixed at the source. **[HW]**
- Fix: either call `servoAngle(joint4)`/`servoAngle(grip)` directly in `get_angles()` (they already exist and are used in `safety_check`/`reportPosition`), or update `joint4_callback`/`grip_callback` inside `moveto()`.
- **Applied**: `get_angles()` now reads `servoAngle(joint4)`/`servoAngle(grip)` directly ([control.cpp:237-243](../src/control.cpp#L237-L243)); `joint4_callback`/`grip_callback`/`moveServo()` (dead — never called) were removed.

### 1.6 [OPEN — being fixed in the new Python planner, not firmware] Suspected double +90° offset on joint 4
- [ros2/src/robot_pkg/pkg/FWK_Degree.py:113-116](../ros2/src/robot_pkg/pkg/FWK_Degree.py#L113-L116), [ros2/src/robot_pkg/pkg/planner_node.py:283-289](../ros2/src/robot_pkg/pkg/planner_node.py#L283-L289) (`send_arm_goal`)
- `IK_fulls_1` already remaps `q4` into the servo's 0–180° frame (`temp = Q[:, 3] + 90`). `send_arm_goal` then adds another `+90.0` (`raw[3] += 90.0 # Servo fix`) to whatever `solve_ik` returned before sending it to the driver.
- If both offsets apply to the same value, joint-4 targets sent over serial can reach up to 270°, well outside the 0–180° servo range enforced by `joint4Max`/`joint4Min` in `config.h` — moves would silently clamp/no-op at the firmware boundary check in `moveto()` ([control.cpp:148-151](../src/control.cpp#L148-L151)).
- Needs a decision: keep the offset in the IK function *or* in the planner, not both. **[HW — confirm by comparing commanded vs. actual joint-4 angle]**

### 1.7 Docker build is broken as checked in
- [ros2/Dockerfile:46](../ros2/Dockerfile#L46)
- `COPY ./build/ros_entrypoint.sh /ros_entrypoint.sh` — there is no `build/` directory in the repo; the actual entrypoint script is at `ros2/ros2_entrypoint.sh` (note also the filename mismatch: `ros_entrypoint.sh` vs `ros2_entrypoint.sh`). `docker build` on this repo fails at this step.
- (Being superseded by the de-ROSification effort — see TODO.md item 1 — but noted in case anyone tries to build the image before that lands.)

### 1.8 [FIXED 2026-07-12] Dead/tautological command-caching logic
- was: `src/main.cpp` (`getCommandID`)
```cpp
if (cmdID == 0|| currentCommand != '~' || currentCommand == commands::cmd_moveref || getStateID() != 'P' || currentCommand != commands::cmd_abort)
    cmdID = currentCommand;
```
- The last clause `currentCommand != commands::cmd_abort` is almost always true (true whenever the current command isn't abort), and it's OR'd with everything else, so the whole condition reduces to "true unless `currentCommand == cmd_abort`". Combined with the `currentCommand != '~'` clause, in practice this function's caching never actually holds a stale value — it's effectively always `cmdID = currentCommand`, making the static-variable caching pointless.
- **Applied**: confirmed `getCommandID()` had no callers anywhere in the firmware; removed the function entirely.

### 1.9 [DOCUMENTED 2026-07-12] Error status byte not in the protocol's command map
- [src/main.cpp:79](../src/main.cpp#L79)
- `serialCLI.sendingPackage('E', errorFlag + '0', robot.angles);` uses `processingID = 'E'`, which was not one of the `commands` enum values (`~,M,A,P,C,G,R,F,S,X,&`, see [serialCommand.h:37-50](../include/serialCommand.h#L37-L50)) and is not in `package_receive.py`'s `COMMAND_MAP` or the README table. On the PC side this always prints as "Unknown (E)" instead of a recognizable error report.
- **Applied**: kept the `'E'` byte as-is (changing it would be a protocol break with no benefit) but documented it in `serialCommand.h` next to the `commands` enum and in the README protocol table; the new Python CLI's status decoder (`robot/tools/cli.py`) recognizes it.

---

## 2. Protocol / documentation mismatches

### 2.1 README's command table has a duplicate/wrong abort code
- [README.md:29-30](../README.md#L29-L30)
```
|cmd_ros2Interface   |S | ros2Interface| ...
|cmd_abort           |S | abort| ...
```
Both `ros2Interface` and `abort` are listed with CommandID `S`. The real abort code is `X` ([serialCommand.h:48](../include/serialCommand.h#L48)).

### 2.2 Test guide publishes the wrong message type
- [ros2/testguide.md:38](../ros2/testguide.md#L38) and [package_gen.py:49-67](../package_gen.py#L49-L67) (`generate_ros2_vision_command`)
- Both publish `std_msgs/msg/String` with a JSON-encoded payload to `/vision/detections`, but `planner_node.py` subscribes with type `robot_interfaces/msg/ObjectList` ([planner_node.py:76-77](../ros2/src/robot_pkg/pkg/planner_node.py#L76-L77)). A `String` publish on that topic fails at the `ros2 topic pub` type-check (or is silently ignored depending on how it's invoked) — it will not reach `vision_callback`.
- The correct form already exists further down in the same file: [testguide.md:93](../ros2/testguide.md#L93) (publishing `robot_interfaces/msg/ObjectList` directly). Section A should be corrected to match, or removed in favor of the working example.

---

## 3. Robustness issues (style/reliability, not necessarily broken today)

- **Blocking sleeps inside async ROS callbacks** — `planner_node.py`: [line 143](../ros2/src/robot_pkg/pkg/planner_node.py#L143) (`time.sleep(5)` in the calibration branch), and [lines 172](../ros2/src/robot_pkg/pkg/planner_node.py#L172), [223](../ros2/src/robot_pkg/pkg/planner_node.py#L223), [262](../ros2/src/robot_pkg/pkg/planner_node.py#L262), [300](../ros2/src/robot_pkg/pkg/planner_node.py#L300) (`time.sleep(self.sleep_zzz)`). These block the executor thread they run on; with a `MultiThreadedExecutor` this merely wastes a thread, but it defeats the purpose of `async`/`await` and will bite if the executor's thread pool is ever reduced.
- **Bare `except: pass` hides real errors** — [planner_node.py:314](../ros2/src/robot_pkg/pkg/planner_node.py#L314) (`solve_ik`) swallows *any* exception, including a `NameError` if the `FWK_Degree` import at [lines 17-23](../ros2/src/robot_pkg/pkg/planner_node.py#L17-L23) failed (that import failure itself only prints a message and continues running rather than stopping the node).
- **Ignored move result before a drop** — [planner_node.py:227-231](../ros2/src/robot_pkg/pkg/planner_node.py#L227-L231) (`MOVING_CENTER` state): `success = await self.send_arm_goal(...)` is computed but never checked; the state machine advances to `MOVING_DROP` even if the lift move failed/timed out.
- **Serial connect failure leaves the node running as a silent no-op** — [serial_driver.py:68-73](../ros2/src/robot_pkg/pkg/serial_driver.py#L68-L73) (`connect_serial`): on `serial.SerialException` it logs `fatal` but does not raise or exit; `self.serial_conn` stays `None` and every subsequent command silently does nothing (`send_binary_pkg` checks `if not self.serial_conn ... return`) rather than failing loudly.
- **Linux/GUI-only vision code** — `vision.py`: `cv2.imshow`/`cv2.waitKey` ([lines 112-114](../ros2/src/robot_pkg/pkg/vision.py#L112-L114)) require an X11 display and break headless/container runs without X forwarding; `cv2.CAP_V4L2` ([line 41](../ros2/src/robot_pkg/pkg/vision.py#L41)) is Linux-only (no-op/wrong on Windows, where `CAP_DSHOW` is the equivalent); the default `camera_dev: 4` ([params.yaml:12](../ros2/src/robot_pkg/config/params.yaml#L12)) is machine-specific and will not exist on a different PC.
- **Busy-wait with no timeout** — [package_receive.py:80-81](../package_receive.py#L80-L81): `while ser.in_waiting < remaining_size: pass` spins a CPU core at 100% with no timeout; a dropped/short packet hangs the tool forever.

---

## 4. Repo hygiene

- **Committed build artifacts**: `ros2/src/robot_pkg/pkg/__pycache__/*.pyc` is checked into git; there is no `.gitignore` anywhere in the repo.
- **Duplicated binary weights**: `best.pt` (YOLO model) exists in two places — [ros2/old/best.pt](../ros2/old/best.pt) and [ros2/src/robot_pkg/pkg/init/best.pt](../ros2/src/robot_pkg/pkg/init/best.pt).
- **Dead/superseded IK implementation**: [ros2/old/FWK_Degree_old.py](../ros2/old/FWK_Degree_old.py) uses different link lengths (`l1..l6 = 170,200,220,45,50,105`) than the current `FWK_Degree.py` and `binh.m` (`180,200,220,50,15,100`) — confirms it's an earlier, now-replaced calibration. Not imported anywhere; safe to remove once confirmed unused, or keep only if it documents useful history.
- **Placeholder license**: [ros2/src/robot_pkg/setup.py:30](../ros2/src/robot_pkg/setup.py#L30) — `license='TODO: License declaration'`.
- **Hardcoded absolute host path**: [ros2/.devcontainer/devcontainer.json:24](../ros2/.devcontainer/devcontainer.json#L24) — `--volume=/home/arch/Documents/PlatformIO/Projects/project_on_robotic:/mnt` only works on the original author's machine.
- **Dead function `angleTopic`**: was in `src/control.cpp`, never called from `main.cpp`/`serialCommand.cpp`; also printed `NODE_SENDBYTE` as a decimal int via `ComPort.print`, not as the raw framing byte the rest of the protocol uses. **Removed 2026-07-12** as part of the HumanInterface cleanup (same pass that fixed 1.5 — `get_angles()` needed touching anyway).

---

## 5. Motor-control review (2026-07-12, `control.cpp`/`control.h`)

Focused review requested by the owner, who considers the MCU firmware reliable overall. Scope: `motorControl::run()`, `refCalibrate()`, limit/collision macros in `control.h`, and the servo/stepper move paths. No behavior changes were made from this section except the `setpos()` fix noted below (in scope because it reintroduced a text byte into what is now a pure-binary interface) — the rest are flagged for the owner/successor to decide on, since they touch active safety logic on a robot the owner considers stable.

### 5.1 [CRITICAL — flagged, not changed] `cmd_abort` does not actually stop the motors
- [src/main.cpp](../src/main.cpp) `operate()`, the `cmd_abort` branch is empty: `else if (cmd == cmd_abort){ }`. No `joint*.stop()` call, nothing.
- The only place the `abort`/`interrupt` concept has any effect is `refCalibrate(bool interrupt)` ([control.cpp:173](../src/control.cpp#L173)) — and there, passing `interrupt = false` does **not** stop the steppers either: it only short-circuits the *wait loops* (`while (... && interrupt ...)`), so the function immediately falls through to `joint1.setCurrentPosition(angleToSteps(REF_A))` etc. — as if the reference switches had just been reached — and then immediately issues `joint1.moveTo(HOME_A)` (and 2/3) to drive the arm to the home position. Meanwhile the moves issued at the top of `refCalibrate` (`joint2.move(...)`, `joint3.move(...)`, `joint1.move(...)`) are still in flight and continue to be serviced by the timer ISR (`TC6_Handler` → `robot.run()`) regardless of `interrupt`, since `run()`'s calibration branch only checks `calibrating`/limit switches, not the `interrupt` flag.
- Net effect: sending `X` (abort) while calibrating does not stop the arm. It (a) makes the firmware believe calibration succeeded and snap its internal position tracking to the reference angles even though the arm may be nowhere near them, then (b) immediately commands a new move to the home position from that now-incorrect assumed position. This can produce unexpected motion right when the operator is trying to stop the arm. **[HW — do not test by actually invoking abort mid-calibration until this is triaged]**
- Outside of calibration, there is no abort/e-stop path in the firmware at all — `cmd_abort` reaching `operate()` in the normal (non-calibrating) state does nothing.
- Recommendation for the owner: implement a real abort — at minimum `joint1.stop(); joint2.stop(); joint3.stop();` (AccelStepper's `stop()` decelerates within the configured acceleration, it doesn't teleport) and set `calibrating = false` unconditionally when `cmd_abort` is received, regardless of what phase `refCalibrate` is in.

### 5.2 Per-axis limit violations hang silently instead of erroring, unlike the collision limit
- [src/control.cpp:40-79](../src/control.cpp#L40) (`run()`)
- When the *collision* guard trips (`!JOINT_2_3_LIMIT_targetpos`), the firmware sets `errorFlag = error_limitation_breaked`, stops all 3 joints, and returns — this error does get reported to the PC via `topicPrint()`/`sendingPackage`.
- But when a single joint's own range check fails (`JOINT1_LIMITATION`/`JOINT2_LIMITATION`/`JOINT3_LIMITATION` is false — e.g. a `moveto`/`move` command targets an angle beyond that joint's configured min/max), `run()` simply skips calling `.run()` for that joint. No error flag is set. `AccelStepper::distanceToGo()` stays non-zero forever (the target was accepted by `moveTo()`/`move()`, it's just never stepped toward), so `ifRun()` reports "still running" indefinitely and `getStateID()` never returns `'D'`. From the PC's point of view, an out-of-range command looks identical to a command that's still in progress — it will eventually time out client-side (the old ROS `serial_driver.py` had a 15s `timeout_sec`) rather than fail fast with a clear error.
- Recommendation: set `errorFlag = error_limitation_breaked` (or a new dedicated code) when any of `JOINT1_LIMITATION`/`JOINT2_LIMITATION`/`JOINT3_LIMITATION` is false, matching the collision-guard behavior, so out-of-range commands fail fast and visibly instead of silently hanging.

### 5.3 `avoidCollision()` is dead code — the real collision guard is a separate, inline duplicate
- [control.h:53](../include/control.h#L53) declares it, [control.cpp:268-281](../src/control.cpp#L268-L281) defines it, but it is never called anywhere. The actual collision protection used at runtime is the inline `JOINT_2_3_LIMIT_currentpos`/`JOINT_2_3_LIMIT_targetpos` macro checks directly inside `run()`. Two independent implementations of the same joint2/joint3 interference guard exist; only one is live. Low risk today (dead code doesn't run), but a future edit to one without the other would silently reintroduce a gap. Recommend deleting `avoidCollision()`, or wiring it in and deleting the inline duplicate — not done here since it's a judgment call outside this review's scope.

### 5.4 [FIXED 2026-07-12] Stray human-readable text in `setpos()`'s error path
- was: `src/control.cpp`, `setpos()` default case: `ComPort.println("[ERROR] Invalid Axis");`
- With HumanInterface removed, the serial line is now expected to carry *only* framed binary packets both directions. This one leftover `println` would inject a raw ASCII line into that stream on an invalid-axis `currentPos` command, which could desync a strict binary reader depending on timing.
- **Applied**: replaced with `errorFlag = error_invalid_axis;`, matching the pattern already used by `move()`/`moveto()`'s default cases.

### 5.5 Blocking `delay(15)` per servo axis inside the command path
- [control.cpp:148-155](../src/control.cpp#L148-L155) (`moveto`, axes `'d'`/`'e'`)
- Each servo move blocks for 15 ms inside `operate()`. Stepper motion isn't affected (driven by the independent `TC6` timer ISR), but a single incoming packet that addresses both joint 4 and grip (bitmask covering `'d'` and `'e'`) blocks the main loop — and therefore new serial command processing — for up to ~30 ms. Not observed to cause a failure in this review, but worth knowing if command latency ever becomes a concern; the 15 ms figure is not derived from anything documented (no comment explains why 15 ms specifically).

### 5.6 [CONFIRMED ON HARDWARE 2026-07-12] `refCalibrate()` reports premature failure while still running — likely root cause of "commands sometimes repeat/get delayed"
- [src/control.cpp:173-234](../src/control.cpp#L173-L234) (`refCalibrate`)
- `refCalibrate()` has 3 sequential phases (joint2/3 homing, joint1 homing, move-to-home), each with its own `TIMEOUT_LIMIT` (10s, `config.h`) wait loop. **Critically, when a phase times out it sends its own `sendingPackage('F','F', ref)` (fail) packet and then falls through to the *next* phase anyway** — it does not return or abort. The function only returns after all 3 phases have each either succeeded or timed out, and `main.cpp`'s `loop()` does not read any new serial command while `operate()` is blocked inside this call (there is no re-entrant serial polling during calibration).
- **Verified against the real MCU** (Due attached, no limit switches wired, so every phase times out): raw packet trace of a single `calibrate` call —
  ```
  t=0.01s   F / P   (start)
  t=10.03s  F / F   (phase 1 timeout -- looks like a final failure, but isn't)
  t=20.05s  F / F   (phase 2 timeout -- also looks final)
  t=24.49s  F / D   (refCalibrate's own unconditional final packet)
  t=24.49s  F / F   (very next tick: errorFlag is still set from the phase-2 timeout and
                      getStateID() reports it before loop()'s own error-report block clears it)
  ```
  A client that treats the *first* `processingID=='F'` packet with `statusID` in `{D,F}` as the command's answer (a reasonable-looking heuristic, and what this rewrite's `serial_link.py` originally did, matching the same pattern used for grip/release/position which genuinely are atomic) gets a false "failed" result at **t≈10s**, while the MCU is still executing calibration and **will not process any new command** for another ~14s. If that client then reacts to the "failure" (e.g. retries, sends a different command), the new command byte is written to the serial port but sits unread in the Arduino's UART buffer until the original `refCalibrate()` call finally returns — from the outside this looks exactly like "the robot ran a stale/duplicate command a while later," matching the owner's reported symptom.
- **Fix applied on the Python side** (`robot/serial_link.py`'s `calibrate()`): rather than resolving on the first `'F'`-tagged terminal packet, it now tracks the last `'F'`-tagged status seen and only resolves once the stream transitions back to idle (`processingID == '~'`) — which structurally can only happen after `refCalibrate()` has actually returned control to `loop()`. Verified on hardware: `calibrate()` now correctly blocks for the full ~24.5s in this no-switches scenario and returns `False` (matching the real outcome), and a `position` query sent immediately afterward gets an instant, correct reply — confirming no stale command was left queued.
- **Not fixed in firmware** (out of scope for this pass, flagged for the owner like 5.1-5.3): the cleaner fix is in `refCalibrate()` itself — return/abort immediately on the first phase timeout instead of continuing through all 3 phases, and/or don't emit a mid-function packet that looks identical to the true final one. Any client that talks to this firmware (not just this Python rewrite) needs to know about this quirk until it's fixed at the source.

### 5.7 [FIXED 2026-07-12] `SerialLink` packet listener was a single slot — `calibrate()` silently displaced `monitor`
- `robot/serial_link.py`
- Found while adding the web dashboard (which needs a permanent listener for its joint-history chart): `set_packet_listener()` held exactly one callback, and `calibrate()` temporarily *replaced* it with its own watcher — so running `tools/cli.py monitor` during a calibration silently dropped the monitor output, and the dashboard recorder would have been displaced the same way.
- Fixed by replacing the single slot with `add_packet_listener()`/`remove_packet_listener()` (a listener list); `calibrate()` and `cli.py` now add/remove instead of swap. Verified on hardware: a monitor listener kept receiving the real `F/P → F/F… → F/D` calibration packets while `calibrate()`'s own watcher ran concurrently, and the 20 Hz stream resumed normally afterwards.

### 5.8 [FIXED 2026-07-18] Servo (joint 4 / grip) commanded but never physically moves
- `src/control.cpp` (`motorControl` constructor, `init()`, `moveto`), `src/main.cpp` (`setupTimer6`)
- Symptom (owner-reported): `moveto -d <deg>` doesn't move the joint-4 servo. Confirmed on hardware that the **command path is fine** — the firmware accepts the command and `servoAngle()` feedback tracks the commanded value (30→29.9, 150→149.9, 90→89.9). But that feedback is **open-loop**: `servoAngle()` returns `readMicroseconds()` (the last commanded pulse), not a real measured position, so a matching feedback value (and the PC's "move done" check) proves only that the command was issued, never that the horn moved. `moveto -e` is separately pinned at 148° by the grip clamp (bug 1.4) — expected, not a fault.
- Two root causes fixed:
  1. **Servos attached before core init.** `attach()` was called only in the global `motorControl robot` constructor, which runs during C++ static init — *before* the Arduino core's `init()` (SAM core: global constructors run from the reset handler, before `main()` calls `init()`). Pulse generation set up that early can fail to actually run. Servos are now (re)attached in `init()`, which `setup()` calls after the core is up.
  2. **50 kHz stepper ISR starving the servo timer.** The stepper callback runs `robot.run()` from `TC6_Handler` every 20 µs (`CALLBACK_TIME`) at the default NVIC priority 0 — the same priority as the Servo library's pulse-timing interrupt (SAM3X: first servos use TC1 ch0 → `TC3_IRQn`). With no priority difference the servo interrupt can't preempt, so a busy stepper ISR delays/omits servo pulse edges. `setupTimer6()` now calls `NVIC_SetPriority(TC6_IRQn, 8)` to demote the stepper ISR below the servo's; a few µs of stepper jitter is harmless.
- Also added an experimental, off-by-default `DETACH_SERVO_AFTER_TASK` macro (`config.h`): when enabled, `moveto` detaches each servo `SERVO_SETTLE_MS` after the write, so idle servos stop drawing holding current and stop emitting refresh pulses (which also removes their ISR load between moves).
- **Still needs the owner to physically confirm** the horn now tracks `moveto -d`: there is no servo-position feedback for automated verification. Firmware compiles and was flashed to the real Due.

---

## 6. Integration round (2026-07-18 → 2026-07-26)

Found while wiring up the full pipeline on real hardware (homography calibration,
the rewritten planner, the redesigned dashboard). All fixes are **PC-side**
(`robot/`); the underlying firmware causes noted below are flagged, not changed.

### 6.1 [FIXED 2026-07-26 — PC side] Base joint overshoots / "buzzes" and the move stalls
- Symptom (owner-reported, with a log): during the rectangular pick-place cycle,
  when the arm reached over the drop box the base joint (joint 1) crept past target
  and the motors hummed without settling, until the move timed out.
- Root cause: the new multi-segment path issues the next `moveTo` as soon as the
  previous move is within `move_tolerance_deg` (2°), i.e. **while joints are still
  moving fast**. Firmware `movetoSync` ([control.cpp](../src/control.cpp)) scales a
  short axis' acceleration down (`A' = A/s²`); its own comment notes it **assumes
  every axis starts from rest**. When joint 1 goes from the long axis in one segment
  to the short axis in the next while still moving ~8°/s, the lowered acceleration
  can't brake it in time → it overshoots ~16° and crawls back, exceeding the 15 s
  move timeout. Confirmed directly from the residual log (joint `a` diverging while
  `b`/`c` sat at zero).
- **Fix applied** (`SerialLink._wait_for_targets`): a move now completes only when
  each commanded joint is within tolerance **and has stopped** (per-sample change ≤
  `settle_eps_deg`, default 0.3°), polling slower than the 20 Hz stream so motion
  isn't aliased away. Every segment therefore starts from rest, satisfying
  `movetoSync`'s assumption.
- **Underlying firmware cause (flagged, not changed):** `movetoSync`'s rest
  assumption. A firmware-side hardening would be to not scale a near-stationary
  axis' acceleration below nominal, so a still-moving short axis can still brake.

### 6.2 [FIXED 2026-07-26 — PC side] Robot calibrates twice on entering auto
- Symptom: entering auto calibrated, then the arm calibrated a second time.
- Root cause: `serial.calibrate_timeout_sec` was **20 s**, under `refCalibrate()`'s
  ~31 s worst case (3 phases × 10 s + ~1 s — the same multi-phase behavior as §5.6).
  A slow-but-succeeding calibrate raised `TimeoutError` while the MCU was still
  calibrating; `Planner._calibrating` caught it and retried → a second full
  calibrate.
- **Fix applied:** raised `calibrate_timeout_sec` → **35 s** (config.yaml) so a slow
  success isn't declared failed, plus a `Planner._need_calibrate` guard so the pick
  loop can never re-enter `CALIBRATING` — calibrates exactly once per auto entry
  (re-armed on each Manual→Auto). Related to §5.6; the real firmware fix is still to
  make `refCalibrate()` stop after the first phase timeout.

### 6.3 [FIXED 2026-07-25 — PC side] Detected position shifts when the camera resolution changes
- Symptom: a homography calibrated at one resolution put objects in the wrong place
  at another (e.g. 1920×1080 → 640×480).
- Root cause: the homography/ROI are expressed in calibration-resolution pixels; the
  live pipeline fed raw current-resolution pixels straight in.
- **Fix applied** (`vision.py::set_frame_size`): rescale the homography + ROI to the
  current frame using the camera's actual behavior — square pixels, centre-crop of
  the longer axis when the aspect ratio differs, then a uniform scale — not an
  independent per-axis stretch. Verified numerically against real captures.

### 6.4 Debug logging added
- `planner.py`: every state transition (`state: A -> B`), each move's target + IK
  solution + result (`reached in Xs` / `TIMEOUT … residual=[…]`), and a throttled
  heartbeat (`[hb] task=… pos=…`). `serial_link.py`: a ~1 Hz in-move residual line
  so a stuck/overshooting joint is visible while the planner thread is blocked in a
  move. These are what made §6.1 diagnosable.

---

## Notes for the successor

Most of the above were found by static analysis alone. §5 (motor-control review) additionally involved compiling and flashing the firmware to the physical Due and exercising the real binary protocol (safe: no motors/switches are wired to it) — see `TODO.md` for current status. Items marked **[HW]** (1.4, 1.6) still need real motor/switch hardware to confirm. Two items are the highest priority:
- **5.1 (no real abort)** — a safety gap, not just a code-quality one.
- **5.6 (`refCalibrate` reports failure ~14s before it's actually done)** — this is the most likely explanation for the "robot sometimes runs a command late/twice" symptom the owner described from memory. It's fixed on the Python client side (`robot/serial_link.py`) but the firmware itself still has the underlying behavior; any other client talking to this MCU needs the same workaround until `refCalibrate()` is changed to stop after the first phase timeout.
