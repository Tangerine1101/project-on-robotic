"""Binary serial protocol to the MCU (machine interface only, see README.md).

Ported from ros2/src/robot_pkg/pkg/serial_driver.py, with the root cause of the
"robot sometimes repeats a command" bug fixed. Root cause (confirmed by tracing
main.cpp/control.cpp): the old serial_driver.py's read_serial_data() set
latest_ack="done" on ANY packet with statusID=='D', regardless of which command
that packet's processingID belonged to. Since the MCU streams status at 20Hz and
almost always reports 'D' (idle) when nothing is moving, a grip/release/calibrate
call sent while the arm was idle would see the very next idle status tick and
report false success immediately -- often before the command had even reached
the MCU. That looked like "the robot ran an old/duplicate command" from the
outside.

Fix here: for commands the firmware acks explicitly and atomically (grip 'G',
release 'R', calibrate 'F', position 'P' -- see main.cpp/control.cpp, these call
sendingPackage() directly, not through the racy 20Hz topicPrint/currentCommand
path), we only accept a 'D'/'F' packet whose processingID matches the exact
command we just sent.

For move/moveto ('M'/'A') the firmware has NO reliable per-command ack at all:
main.cpp's operate() only updates `currentCommand` inside the same call that
also resets it to '~' the instant the move finishes, so by the time the next
20Hz topicPrint sample is taken, the command byte has already been overwritten
back to '~' -- a moveto command's completion is structurally never reported as
(processingID==<cmd>, statusID=='D'). So for moves we fall back to the same
approach the ROS driver used: poll the streamed current_joints until they're
within tolerance of the commanded target. See doc/bug-report.md section 5 for
the full firmware trace this is based on.

Only one command may be in flight at a time (self._cmd_lock) -- this is the
single-flight queue the old ROS code lacked.
"""
import logging
import struct
import threading
import time

import serial

logger = logging.getLogger("serial_link")

START_BYTE = 0xAA   # host -> MCU
SEND_BYTE = 0xFE    # MCU -> host
MAX_ARGS = 5

OUT_FMT = "<BcB5fB"   # serialPackage: start, commandID, bitmask, 5 floats, checksum
IN_FMT = "<Bcc5fBB"   # sendPackage:   start, processingID, statusID, 5 floats, limitSwitches, checksum
IN_SIZE = struct.calcsize(IN_FMT)

# Reference/limit-switch bitmask (the extra byte in every packet). Raw polarity
# from the firmware: bit set (1) = switch open / not touched, bit clear (0) =
# arm sitting on that switch. See include/serialCommand.h.
LIMIT_SW_BITS = (0x01, 0x02, 0x04)  # A(joint1), B(joint2), C(joint3)

# Axis order used by the bitmask (bit i => axis i): a,b,c,d,e = joint1,2,3,4,grip
AXES = "abcde"

# Commands that get an immediate, atomic ack from the firmware (safe to match
# on processingID). Move/moveto ('M'/'A') are deliberately NOT in this set --
# see module docstring.
_ACKED_COMMANDS = {"G", "R", "F", "P"}

COMMAND_MAP = {
    "~": "None/Idle",
    "M": "Move (Relative)",
    "A": "MoveTo (Absolute)",
    "P": "Position Report",
    "C": "Set Current Pos",
    "G": "Grip (Close)",
    "R": "Release (Open)",
    "F": "MoveRef (Calibrate)",
    "S": "Machine Interface",
    "X": "Abort",
    "E": "Error Report",
    "&": "Invalid",
}
STATUS_MAP = {
    "P": "Processing",
    "D": "Done",
    "F": "Fail",
    "~": "Idle",
}


def _xor_checksum(data):
    checksum = 0
    for b in data:
        checksum ^= b
    return checksum


def _pad_args(args):
    padded = (list(args) + [0.0] * MAX_ARGS)[:MAX_ARGS]
    return [float(a) for a in padded]


class SerialLink:
    def __init__(self, port, baudrate=115200, config=None):
        config = config or {}
        self.move_tolerance = config.get("move_tolerance_deg", 2.0)
        self.move_timeout = config.get("move_timeout_sec", 15.0)
        self.calibrate_timeout = config.get("calibrate_timeout_sec", 20.0)
        self.grip_timeout = config.get("grip_timeout_sec", 5.0)
        # A move is "done" only once the joints are within tolerance AND have
        # stopped (per-sample change <= this). Returning while a joint is still
        # moving fast makes the *next* movetoSync overshoot -- see _wait_for_targets.
        self.settle_eps = float(config.get("settle_eps_deg", 0.3))

        self._ser = serial.Serial(port, baudrate, timeout=0.05)
        self._rx_buf = bytearray()
        self._state_lock = threading.Lock()
        self.current_joints = [0.0] * MAX_ARGS
        self.last_status = ("~", "~")
        # Per-switch "touched?" booleans (A, B, C) decoded from every packet's
        # limitSwitches byte. True = arm is on that reference/limit switch.
        self.limit_switches = (False, False, False)

        self._cmd_lock = threading.RLock()  # single-flight: one command in the air at a time
        self._ack_cmd = None
        self._ack_event = threading.Event()
        self._ack_result = None

        self._listeners = []
        self._listeners_lock = threading.Lock()

        self._stop = threading.Event()
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True, name="serial-rx")
        self._rx_thread.start()

    def close(self):
        self._stop.set()
        self._rx_thread.join(timeout=2)
        try:
            self._ser.close()
        except Exception:
            pass

    def add_packet_listener(self, callback):
        """callback(proc_char, status_char, args_list) -- called for every valid
        packet received. Multiple listeners may be registered at once (e.g. the
        dashboard's history recorder alongside calibrate()'s watcher)."""
        with self._listeners_lock:
            if callback not in self._listeners:
                self._listeners.append(callback)

    def remove_packet_listener(self, callback):
        with self._listeners_lock:
            if callback in self._listeners:
                self._listeners.remove(callback)

    # ------------------------------------------------------------------ RX

    def _rx_loop(self):
        while not self._stop.is_set():
            try:
                data = self._ser.read(4096)
            except serial.SerialException:
                logger.exception("serial read failed")
                time.sleep(0.5)
                continue
            if data:
                self._rx_buf.extend(data)
            while len(self._rx_buf) >= IN_SIZE:
                if self._rx_buf[0] != SEND_BYTE:
                    del self._rx_buf[0]
                    continue
                pkt = bytes(self._rx_buf[:IN_SIZE])
                if _xor_checksum(pkt[:-1]) != pkt[-1]:
                    del self._rx_buf[0]
                    continue
                del self._rx_buf[:IN_SIZE]
                self._handle_packet(pkt)

    def _handle_packet(self, pkt):
        _, proc_b, stat_b, *rest = struct.unpack(IN_FMT, pkt)
        args = list(rest[:MAX_ARGS])
        limit_byte = rest[MAX_ARGS]  # rest[MAX_ARGS + 1] is the checksum
        proc = proc_b.decode("ascii", errors="replace")
        stat = stat_b.decode("ascii", errors="replace")
        # raw polarity -> "touched?" (bit clear = on the switch)
        switches = tuple(not (limit_byte & b) for b in LIMIT_SW_BITS)

        with self._state_lock:
            self.current_joints = args
            self.last_status = (proc, stat)
            self.limit_switches = switches

        with self._listeners_lock:
            listeners = list(self._listeners)
        for listener in listeners:
            try:
                listener(proc, stat, args)
            except Exception:
                logger.exception("packet listener raised")

        if self._ack_cmd is not None and proc == self._ack_cmd and stat in ("D", "F"):
            self._ack_result = stat == "D"
            self._ack_cmd = None
            self._ack_event.set()

    # ------------------------------------------------------------------ TX

    def _send(self, cmd_char, args, bitmask=0x1F):
        payload = struct.pack(
            "<BcB5f", START_BYTE, cmd_char.encode("ascii"), bitmask, *_pad_args(args)
        )
        checksum = _xor_checksum(payload)
        self._ser.write(payload + bytes([checksum]))

    def _send_and_wait_ack(self, cmd_char, args, bitmask, timeout):
        assert cmd_char in _ACKED_COMMANDS, f"{cmd_char} has no reliable firmware ack"
        with self._cmd_lock:
            self._ack_event.clear()
            self._ack_cmd = cmd_char
            self._ack_result = None
            self._send(cmd_char, args, bitmask)
            got = self._ack_event.wait(timeout)
            self._ack_cmd = None
            if not got:
                raise TimeoutError(f"no ack for '{cmd_char}' within {timeout}s")
            return self._ack_result

    @staticmethod
    def _reached(current, prev, targets, bitmask, tolerance, settle_eps):
        """A move is complete only when every commanded joint is within `tolerance`
        of its target AND has stopped moving (changed <= `settle_eps` since the
        previous sample `prev`). The at-rest requirement is what prevents the next
        movetoSync from overshooting: the firmware scales a short axis' acceleration
        down, so if that axis is still moving fast when the next segment is issued it
        cannot brake in time and shoots well past target. `prev` is None on the first
        sample (treated as still moving)."""
        if prev is None:
            return False
        for i in range(MAX_ARGS):
            if not (bitmask & (1 << i)):
                continue
            if abs(targets[i] - current[i]) > tolerance:
                return False
            if abs(current[i] - prev[i]) > settle_eps:
                return False
        return True

    def _wait_for_targets(self, targets, bitmask, tolerance, timeout):
        deadline = time.monotonic() + timeout
        next_log = time.monotonic() + 1.0  # throttle the live-progress line to ~1Hz
        prev = None
        while time.monotonic() < deadline:
            with self._state_lock:
                current = list(self.current_joints)
            if self._reached(current, prev, targets, bitmask, tolerance, self.settle_eps):
                return True
            # Live view while the planner thread is blocked here (the arm is moving
            # or stuck): shows per-axis residual so a frozen/overshooting joint --
            # e.g. the base creeping past target over the box -- is visible.
            now = time.monotonic()
            if now >= next_log:
                next_log = now + 1.0
                axes = [i for i in range(MAX_ARGS) if bitmask & (1 << i)]
                resid = "  ".join(f"{AXES[i]}:{targets[i] - current[i]:+.1f}" for i in axes)
                logger.info("move: waiting  residual[%s]", resid)
            prev = current
            # Poll slower than the 20Hz (50ms) status stream so `prev` is always a
            # genuinely earlier sample -> real motion is detected, not aliased away.
            time.sleep(0.08)
        return False

    # ------------------------------------------------------------- high level

    def move_to(self, angles, bitmask=0x0F, tolerance=None, timeout=None):
        """Absolute move. bitmask defaults to joints 1-4 only (0x0F), excluding
        the grip axis ('e') -- arm moves should never touch the gripper as a
        side effect (see doc/bug-report.md 1.4: once gripOpen/gripClose get
        distinct values, sending bitmask 0x1F here would move the gripper to
        0 deg on every arm move)."""
        tolerance = self.move_tolerance if tolerance is None else tolerance
        timeout = self.move_timeout if timeout is None else timeout
        targets = _pad_args(angles)
        with self._cmd_lock:
            self._send("A", angles, bitmask)
            return self._wait_for_targets(targets, bitmask, tolerance, timeout)

    def move_rel(self, deltas, bitmask=0x0F, tolerance=None, timeout=None):
        tolerance = self.move_tolerance if tolerance is None else tolerance
        timeout = self.move_timeout if timeout is None else timeout
        with self._cmd_lock:
            with self._state_lock:
                base = list(self.current_joints)
            deltas_p = _pad_args(deltas)
            targets = [
                base[i] + deltas_p[i] if (bitmask & (1 << i)) else base[i]
                for i in range(MAX_ARGS)
            ]
            self._send("M", deltas, bitmask)
            return self._wait_for_targets(targets, bitmask, tolerance, timeout)

    def grip_close(self, timeout=None):
        timeout = self.grip_timeout if timeout is None else timeout
        return self._send_and_wait_ack("G", [], 0x1F, timeout)

    def grip_open(self, timeout=None):
        timeout = self.grip_timeout if timeout is None else timeout
        return self._send_and_wait_ack("R", [], 0x1F, timeout)

    def calibrate(self, timeout=None):
        """Calibration is multi-phase on the firmware (control.cpp refCalibrate):
        each phase that times out waiting for a limit switch sends its own
        'F'/'F' (fail) packet and then CONTINUES to the next phase anyway --
        the MCU's main loop stays blocked inside refCalibrate() the whole
        time and won't read any new command until it truly returns, up to
        three TIMEOUT_LIMIT (10s) phases later. Matching on the first 'F'
        ack (as _send_and_wait_ack does for grip/release/position, which ARE
        atomic) would report failure after ~10s while the arm is still busy
        for up to ~20s more -- a client that reacts to that (e.g. retries)
        sends a byte that just sits unread in the MCU's serial buffer until
        the real end. See doc/bug-report.md 5.6.

        So instead we wait for the stream to go idle again (processingID
        reverts to '~') after having seen at least one 'F'-tagged packet --
        that only happens once refCalibrate() has actually returned -- and
        report the *last* F-tagged status seen before that as the result.
        """
        timeout = self.calibrate_timeout if timeout is None else timeout
        state = {"last_status": None, "done": False}
        done_evt = threading.Event()

        def watcher(proc, stat, args):
            if proc == "F":
                state["last_status"] = stat
            elif proc == "~" and state["last_status"] is not None and not state["done"]:
                state["done"] = True
                done_evt.set()

        with self._cmd_lock:
            self.add_packet_listener(watcher)
            try:
                self._send("F", [], 0x1F)
                got = done_evt.wait(timeout)
            finally:
                self.remove_packet_listener(watcher)
        if not got:
            raise TimeoutError(f"calibrate did not settle within {timeout}s")
        return state["last_status"] == "D"

    def query_position(self, timeout=2.0):
        self._send_and_wait_ack("P", [], 0x1F, timeout)
        with self._state_lock:
            return list(self.current_joints)

    def set_current_pos(self, angles, bitmask=0x0F):
        """Fire-and-forget: firmware doesn't ack cmd_currentPos."""
        with self._cmd_lock:
            self._send("C", angles, bitmask)

    def abort(self):
        """Fire-and-forget, bypasses the single-flight lock so it's sent
        immediately even if another command is waiting for its ack.
        NOTE: the firmware does not fully implement this yet -- see
        doc/bug-report.md section 5.1. This call does not guarantee the arm
        actually stops moving."""
        self._send("X", [], 0x1F)
