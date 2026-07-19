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
- MOVING_CENTER now checks the lift move's result instead of ignoring it.
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
    PRE_TASK_OPEN = 3
    SCANNING = 4
    MOVING_PICK = 5
    GRIPPING = 6
    MOVING_CENTER = 7
    MOVING_DROP = 8
    RELEASING = 9
    MANUAL = 99


STATE_NAMES = {
    State.WAIT_DEPS: "WAIT_DEPS",
    State.CALIBRATING: "CALIBRATING",
    State.MOVING_HOME: "MOVING_HOME",
    State.PRE_TASK_OPEN: "PRE_TASK_OPEN",
    State.SCANNING: "SCANNING",
    State.MOVING_PICK: "MOVING_PICK",
    State.GRIPPING: "GRIPPING",
    State.MOVING_CENTER: "MOVING_CENTER",
    State.MOVING_DROP: "MOVING_DROP",
    State.RELEASING: "RELEASING",
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
        self.zones = cfg.get("zones", {})
        self.zone_radius_mm = float(cfg.get("zone_radius_mm", 150.0))

        self.state = State.WAIT_DEPS
        self.current_target = None
        self.target_angles = None
        self._stop_evt = threading.Event()

    @property
    def state_name(self):
        return STATE_NAMES.get(self.state, str(self.state))

    def manual_mode(self):
        self.state = State.MANUAL

    def auto_mode(self):
        if self.state == State.MANUAL:
            self.state = State.WAIT_DEPS

    def stop(self):
        self._stop_evt.set()

    def run_forever(self):
        while not self._stop_evt.is_set():
            try:
                self._step()
            except Exception:
                logger.exception("planner step failed in state %s", self.state)
            time.sleep(0.1)

    # ------------------------------------------------------------------

    def _step(self):
        if self.state == State.MANUAL:
            return
        handler = {
            State.WAIT_DEPS: self._wait_deps,
            State.CALIBRATING: self._calibrating,
            State.MOVING_HOME: self._moving_home,
            State.PRE_TASK_OPEN: self._pre_task_open,
            State.SCANNING: self._scanning,
            State.MOVING_PICK: self._moving_pick,
            State.GRIPPING: self._gripping,
            State.MOVING_CENTER: self._moving_center,
            State.MOVING_DROP: self._moving_drop,
            State.RELEASING: self._releasing,
        }[self.state]
        handler()

    def _wait_deps(self):
        if self.wait_vision and not self.vision.has_recent_detection():
            logger.warning("Waiting for vision...")
            return
        logger.info("Ready. Requesting calibration.")
        self.state = State.CALIBRATING

    def _calibrating(self):
        logger.info("Calibrating...")
        try:
            ok = self.link.calibrate()
        except TimeoutError:
            ok = False
        if ok:
            logger.info("Calibrated.")
            self.state = State.MOVING_HOME
        else:
            logger.error("Calibration failed! Retrying...")

    def _moving_home(self):
        logger.info("Moving home...")
        if self.link.move_to(self.home_angles):
            self.state = State.PRE_TASK_OPEN
        else:
            logger.error("Home move failed/timed out.")

    def _pre_task_open(self):
        logger.info("Ensuring open...")
        self.link.grip_open()
        time.sleep(self.sleep_time)
        if self.vision:
            self.vision.clear_detections()
        self.state = State.SCANNING

    def _scanning(self):
        if not self.vision:
            return  # --no-vision mode: nothing auto-triggers a pick
        detections = self.vision.get_detections()
        if not detections:
            logger.info("Scanning...")
            return

        candidates = []
        for obj in detections:
            if obj["name"].lower() not in VALID_NAMES:
                continue
            in_zone, dist, zone_name = self._check_zone_radius(obj["x"], obj["y"])
            if in_zone:
                logger.info("Ignoring %s in %s (%.0fmm)", obj["name"], zone_name, dist)
                continue
            candidates.append(obj)

        if not candidates:
            logger.info("Objects filtered out.")
            return

        closest = min(candidates, key=lambda o: math.hypot(o["x"], o["y"]))
        logger.info("Target: %s", closest["name"])
        angles = solve_ik_first(closest["x"], closest["y"], self.pickup_height_mm)
        if angles is None:
            logger.warning("Unreachable.")
            self.vision.clear_detections()
            return
        self.current_target = closest
        self.target_angles = angles
        self.state = State.MOVING_PICK

    def _moving_pick(self):
        logger.info("Picking...")
        ok = self.link.move_to(self.target_angles)
        self.state = State.GRIPPING if ok else State.MOVING_HOME

    def _gripping(self):
        logger.info("Gripping...")
        self.link.grip_close()
        time.sleep(self.sleep_time)
        self.state = State.MOVING_CENTER

    def _moving_center(self):
        logger.info("Lifting to center...")
        center_angles = [0.0, 0.0, 0.0, self.home_angles[3]]
        ok = self.link.move_to(center_angles)
        if ok:
            self.state = State.MOVING_DROP
        else:
            logger.error("Center lift failed; returning home instead of dropping.")
            self.state = State.MOVING_HOME

    def _moving_drop(self):
        name = (self.current_target["name"].lower() if self.current_target else "")
        zone = self.zones.get(name, {"x": 150.0, "y": 150.0})
        logger.info("Dropping %s at zone %s...", name, zone)
        angles = solve_ik_first(zone["x"], zone["y"], 100.0)
        if angles is not None:
            self.link.move_to(angles)
        self.state = State.RELEASING

    def _releasing(self):
        self.link.grip_open()
        time.sleep(self.sleep_time)
        logger.info("Done.")
        self.state = State.MOVING_HOME

    def _check_zone_radius(self, x_mm, y_mm):
        for name, zone in self.zones.items():
            dist = math.hypot(x_mm - zone["x"], y_mm - zone["y"])
            if dist <= self.zone_radius_mm:
                return True, dist, name
        return False, 0.0, ""
