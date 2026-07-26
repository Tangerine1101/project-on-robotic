"""Pick-and-place state machine.

Ported from ros2/src/robot_pkg/pkg/planner_node.py's brain_loop. Runs as a plain
sequential loop in its own thread instead of an async ROS callback, which makes
the old blocking time.sleep() calls actually correct (there's no executor to
starve) rather than a code smell.

Fixes folded in during the port (see doc/bug-report.md):
- 1.6: removed the extra `+90` on joint 4 (send_arm_goal used to add +90 on top
  of the +90 kinematics.solve_ik already applies). config.yaml's home_angles
  and every hardcoded joint-4 target below are written in the final servo
  frame directly.
- The cycle is a continuous rectangular pick-and-place: approach above the object
  -> lower to pick -> lift -> traverse to the box -> lower to the box's non-zero
  height -> release -> lift clear, then straight on to the next object. It only
  returns home (idle) when the table is empty, and prioritises the object nearest
  its own sorting box. Every move checks its result and falls back to MOVING_HOME
  on failure instead of dropping blindly.
- No bare `except: pass` around the IK call -- exceptions are logged.
"""
import logging
import math
import threading
import time

from kinematics import solve_ik_first

logger = logging.getLogger("planner")


class State:
    WAIT_DEPS = 0
    CALIBRATING = 1
    MOVING_HOME = 2
    SCANNING = 3
    MOVING_ABOVE_PICK = 4  # over the object at lift height, before descending to pick
    MOVING_PICK = 5        # down onto the object (z=pickup_height_mm)
    GRIPPING = 6
    MOVING_LIFT = 7        # straight up above the pick point (same x,y, z=lift_height_mm)
    MOVING_TRAVERSE = 8    # sideways at lift height, above the drop box
    MOVING_DROP = 9        # down into the box (z=drop_height_mm, box rim is non-zero)
    RELEASING = 10
    MOVING_RETREAT = 11    # up clear of the box after release, before the next object
    MANUAL = 99


STATE_NAMES = {
    State.WAIT_DEPS: "WAIT_DEPS",
    State.CALIBRATING: "CALIBRATING",
    State.MOVING_HOME: "MOVING_HOME",
    State.SCANNING: "SCANNING",
    State.MOVING_ABOVE_PICK: "MOVING_ABOVE_PICK",
    State.MOVING_PICK: "MOVING_PICK",
    State.GRIPPING: "GRIPPING",
    State.MOVING_LIFT: "MOVING_LIFT",
    State.MOVING_TRAVERSE: "MOVING_TRAVERSE",
    State.MOVING_DROP: "MOVING_DROP",
    State.RELEASING: "RELEASING",
    State.MOVING_RETREAT: "MOVING_RETREAT",
    State.MANUAL: "MANUAL",
}

VALID_NAMES = ("onion", "garlic", "lemon")


class Planner:
    def __init__(self, link, vision, config):
        self.link = link
        self.vision = vision  # None when running with --no-vision
        cfg = config["planner"]
        self.wait_vision = bool(cfg.get("wait_vision", True)) and vision is not None
        self.home_angles = list(cfg["home_angles"])
        self.sleep_time = float(cfg.get("sleep_time", 0.5))
        self.pickup_height_mm = float(cfg.get("pickup_height_mm", 0.0))
        # Safe height for the lift + sideways traverse (up/over/down path), and the
        # height to descend to when releasing into the drop box (its rim sits above
        # the table, so this is non-zero). Both in mm; see config.yaml planner block.
        self.lift_height_mm = float(cfg.get("lift_height_mm", 120.0))
        self.drop_height_mm = float(cfg.get("drop_height_mm", 40.0))
        self.zones = cfg.get("zones", {})
        self.zone_radius_mm = float(cfg.get("zone_radius_mm", 150.0))

        self.state = State.WAIT_DEPS
        self.current_target = None
        # True once we've parked at home with nothing to pick, so SCANNING can idle
        # there instead of re-homing every tick. Cleared the moment a target appears.
        self._homed_idle = False
        # Calibrate exactly once per auto entry: set on construction (startup enters
        # auto) and on every Manual->Auto switch, cleared once a calibrate succeeds.
        self._need_calibrate = True
        self._last_hb = 0.0  # last heartbeat log time
        self._stop_evt = threading.Event()

    @property
    def state_name(self):
        return STATE_NAMES.get(self.state, str(self.state))

    def _set_state(self, new):
        """Single choke point for state changes so every transition is logged."""
        if new != self.state:
            logger.info("state: %s -> %s", self.state_name, STATE_NAMES.get(new, str(new)))
        self.state = new

    def manual_mode(self):
        self._set_state(State.MANUAL)

    def auto_mode(self):
        if self.state == State.MANUAL:
            self._need_calibrate = True  # re-entering auto always recalibrates once
            self._set_state(State.WAIT_DEPS)

    def stop(self):
        self._stop_evt.set()

    def run_forever(self):
        while not self._stop_evt.is_set():
            try:
                self._step()
                self._heartbeat()
            except Exception:
                logger.exception("planner step failed in state %s", self.state)
            time.sleep(0.1)

    def _heartbeat(self, period=1.0):
        """Throttled 'current task + arm position' line for terminal debugging.
        Quiet while parked idle at home so an unattended arm doesn't spam."""
        if self.state == State.MANUAL:
            return
        if self.state == State.SCANNING and self._homed_idle:
            return
        now = time.monotonic()
        if now - self._last_hb < period:
            return
        self._last_hb = now
        pos = ", ".join(f"{a:.1f}" for a in self.link.current_joints)
        logger.info("[hb] task=%s pos=[%s]", self.state_name, pos)

    # ------------------------------------------------------------------

    def _step(self):
        if self.state == State.MANUAL:
            return
        handler = {
            State.WAIT_DEPS: self._wait_deps,
            State.CALIBRATING: self._calibrating,
            State.MOVING_HOME: self._moving_home,
            State.SCANNING: self._scanning,
            State.MOVING_ABOVE_PICK: self._moving_above_pick,
            State.MOVING_PICK: self._moving_pick,
            State.GRIPPING: self._gripping,
            State.MOVING_LIFT: self._moving_lift,
            State.MOVING_TRAVERSE: self._moving_traverse,
            State.MOVING_DROP: self._moving_drop,
            State.RELEASING: self._releasing,
            State.MOVING_RETREAT: self._moving_retreat,
        }[self.state]
        handler()

    def _wait_deps(self):
        # Auto entry: calibrate once (independent of vision), else go straight to
        # scanning. SCANNING already tolerates an empty/never-ready camera.
        self._set_state(State.CALIBRATING if self._need_calibrate else State.SCANNING)

    def _calibrating(self):
        if not self._need_calibrate:  # guard: never calibrate twice per auto entry
            self._set_state(State.MOVING_HOME)
            return
        logger.info("Calibrating (once per auto entry)...")
        try:
            ok = self.link.calibrate()
        except TimeoutError:
            ok = False
        if ok:
            logger.info("Calibrated.")
            self._need_calibrate = False
            self._set_state(State.MOVING_HOME)
        else:
            logger.error("Calibration FAILED (timeout/fail ack); retrying...")

    def _moving_home(self):
        # Park at home and idle there until an object shows up. Opening the gripper
        # here also recovers a still-held object after a mid-cycle failure.
        logger.info("Moving home...")
        if not self.link.move_to(self.home_angles):
            logger.error("Home move failed/timed out.")
            return
        self.link.grip_open()
        self._homed_idle = True
        self._set_state(State.SCANNING)

    def _scanning(self):
        if not self.vision:
            return  # --no-vision mode: nothing auto-triggers a pick

        candidates = []
        for obj in self.vision.get_detections():
            if obj["name"].lower() not in VALID_NAMES:
                continue
            in_zone, dist, zone_name = self._check_zone_radius(obj["x"], obj["y"])
            if in_zone:
                logger.info("Ignoring %s already in %s (%.0fmm)", obj["name"], zone_name, dist)
                continue
            candidates.append(obj)

        if not candidates:
            # Nothing to do: return home once, then idle there quietly.
            if not self._homed_idle:
                self._set_state(State.MOVING_HOME)
            return

        target = self._nearest_to_box(candidates)
        logger.info("Target: %s at (%.0f, %.0f)", target["name"], target["x"], target["y"])
        self.current_target = target
        self._homed_idle = False
        self._set_state(State.MOVING_ABOVE_PICK)

    def _nearest_to_box(self, candidates):
        """Pick the object closest to its own destination box (fall back to distance
        from the robot origin if that type has no configured zone)."""
        def dist_to_box(o):
            zone = self.zones.get(o["name"].lower())
            if zone is None:
                return math.hypot(o["x"], o["y"])
            return math.hypot(o["x"] - zone["x"], o["y"] - zone["y"])
        return min(candidates, key=dist_to_box)

    def _moving_above_pick(self):
        # Approach from directly above the object at the safe lift height.
        x, y = self.current_target["x"], self.current_target["y"]
        logger.info("Moving above object...")
        if self._move_to_xyz(x, y, self.lift_height_mm, "Approach"):
            self._set_state(State.MOVING_PICK)
        else:
            self._set_state(State.MOVING_HOME)

    def _moving_pick(self):
        # Lower straight down onto the object.
        x, y = self.current_target["x"], self.current_target["y"]
        logger.info("Lowering to pick...")
        if self._move_to_xyz(x, y, self.pickup_height_mm, "Pick"):
            self._set_state(State.GRIPPING)
        else:
            self._set_state(State.MOVING_HOME)

    def _gripping(self):
        logger.info("Gripping...")
        self.link.grip_close()
        time.sleep(self.sleep_time)
        self._set_state(State.MOVING_LIFT)

    def _drop_zone(self):
        """Drop-box (x, y) for the currently held object, from config zones."""
        name = (self.current_target["name"].lower() if self.current_target else "")
        return self.zones.get(name, {"x": 150.0, "y": 150.0})

    def _move_to_xyz(self, x, y, z, what):
        """IK + move to a Cartesian point; True on success. On unreachable/failed
        move, log and let the caller fall back to MOVING_HOME (drops the attempt).
        Logs the target, the IK joint solution, and on timeout the per-joint
        residual -- the key probe for the 'buzzing over the box' trajectory bug."""
        angles = solve_ik_first(x, y, z)
        if angles is None:
            logger.warning("%s unreachable at (%.0f, %.0f, %.0f).", what, x, y, z)
            return False
        ik = ", ".join(f"{a:.1f}" for a in angles)
        logger.info("%s: target=(%.0f, %.0f, %.0f) ik=[%s]", what, x, y, z, ik)
        t0 = time.monotonic()
        ok = self.link.move_to(angles)
        dt = time.monotonic() - t0
        if ok:
            logger.info("%s: reached in %.1fs", what, dt)
            return True
        # Timed out: show where each commanded joint got stuck (target vs current).
        cur = self.link.current_joints
        residual = ", ".join(f"{abs(angles[i] - cur[i]):.1f}" for i in range(len(angles)))
        logger.error(
            "%s: TIMEOUT after %.1fs target=[%s] pos=[%s] residual(J1-4)=[%s]",
            what, dt, ik, ", ".join(f"{a:.1f}" for a in cur[:len(angles)]), residual)
        return False

    def _moving_lift(self):
        # Straight up above the pick point (same x,y) so the object clears the table.
        logger.info("Lifting straight up...")
        x, y = self.current_target["x"], self.current_target["y"]
        if self._move_to_xyz(x, y, self.lift_height_mm, "Lift"):
            self._set_state(State.MOVING_TRAVERSE)
        else:
            self._set_state(State.MOVING_HOME)

    def _moving_traverse(self):
        # Move sideways at the safe lift height, over the drop box.
        zone = self._drop_zone()
        logger.info("Traversing to box %s at lift height...", zone)
        if self._move_to_xyz(zone["x"], zone["y"], self.lift_height_mm, "Traverse"):
            self._set_state(State.MOVING_DROP)
        else:
            self._set_state(State.MOVING_HOME)

    def _moving_drop(self):
        # Lower into the box (its rim sits above the table -> non-zero z).
        zone = self._drop_zone()
        logger.info("Lowering into box %s...", zone)
        if self._move_to_xyz(zone["x"], zone["y"], self.drop_height_mm, "Drop"):
            self._set_state(State.RELEASING)
        else:
            self._set_state(State.MOVING_HOME)

    def _releasing(self):
        logger.info("Releasing...")
        self.link.grip_open()
        time.sleep(self.sleep_time)
        self._set_state(State.MOVING_RETREAT)

    def _moving_retreat(self):
        # Lift clear of the box, then go straight to the next object (no homing).
        zone = self._drop_zone()
        logger.info("Lifting clear of box...")
        if self._move_to_xyz(zone["x"], zone["y"], self.lift_height_mm, "Retreat"):
            self._set_state(State.SCANNING)
        else:
            self._set_state(State.MOVING_HOME)

    def _check_zone_radius(self, x_mm, y_mm):
        for name, zone in self.zones.items():
            dist = math.hypot(x_mm - zone["x"], y_mm - zone["y"])
            if dist <= self.zone_radius_mm:
                return True, dist, name
        return False, 0.0, ""
