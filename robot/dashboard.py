"""Web dashboard: camera stream, joint-angle history chart, manual control.

Served by Flask on a background thread (see main.py). Endpoints:

    GET  /                    the single-page UI (static/index.html)
    GET  /video               MJPEG stream of the vision worker's annotated frames
    GET  /api/status          mode, planner state, current joints, grip angle,
                              detected objects, link status, limit-switch states
    GET  /api/history?since=  joint-angle samples recorded from the 20Hz stream
    POST /api/mode            {"auto": true|false} -- toggle planner auto/manual
    POST /api/move            {"x","y","z","w","e": deg} -- joint-angle move,
                              fields map to joints 1,2,3,4,grip; only in manual mode
    POST /api/goto            {"x","y","z": mm} -- Cartesian move via inverse
                              kinematics (joints 1-4 only); only in manual mode
    POST /api/grip            close the gripper; only in manual mode
    POST /api/release         open the gripper; only in manual mode
    POST /api/calibrate       run the reference/home calibration; only in manual mode

Manual commands (move/goto/grip/release/calibrate) are rejected with 409 unless
the planner is in MANUAL mode -- in AUTO the planner owns the arm.

The history recorder registers a SerialLink packet listener, so it keeps
recording during calibrate()/monitor without conflicting with their listeners.
"""
import collections
import logging
import threading
import time
from pathlib import Path

from flask import Flask, Response, jsonify, request

from kinematics import solve_ik_first

logger = logging.getLogger("dashboard")

# /api/move field -> (bitmask bit, joint label). Order matters: index i is
# the protocol's Arguments[i] slot (axes a..e).
MOVE_FIELDS = ("x", "y", "z", "w", "e")

HISTORY_LEN = 2400  # ~2 minutes at the firmware's 20Hz status rate


def create_dashboard(link, vision, planner):
    """Build the Flask app. `vision` may be None (--no-vision)."""
    app = Flask(
        __name__,
        static_folder=str(Path(__file__).resolve().parent / "static"),
        static_url_path="",  # index.html references /app.js, /style.css directly
    )

    history = collections.deque(maxlen=HISTORY_LEN)  # (epoch_sec, [5 joint angles])
    history_lock = threading.Lock()

    def record_packet(proc, stat, args):
        with history_lock:
            history.append((time.time(), list(args)))

    link.add_packet_listener(record_packet)

    @app.route("/")
    def index():
        return app.send_static_file("index.html")

    @app.route("/video")
    def video():
        if vision is None:
            return Response("vision disabled", status=503)

        def frames():
            last = None
            while True:
                jpeg = vision.get_frame_jpeg()
                if jpeg is not None and jpeg is not last:
                    last = jpeg
                    yield (b"--frame\r\nContent-Type: image/jpeg\r\n"
                           b"Content-Length: " + str(len(jpeg)).encode() + b"\r\n\r\n"
                           + jpeg + b"\r\n")
                time.sleep(0.04)

        return Response(frames(), mimetype="multipart/x-mixed-replace; boundary=frame")

    @app.route("/api/status")
    def api_status():
        joints = list(link.current_joints)
        detections = []
        if vision is not None:
            detections = [
                {"name": d["name"], "x": d["x"], "y": d["y"]}
                for d in vision.get_detections()
            ]
        return jsonify({
            "mode": "manual" if planner.state_name == "MANUAL" else "auto",
            "state": planner.state_name,
            "joints": joints,
            "grip_angle": joints[4] if len(joints) > 4 else None,
            "detections": detections,
            "link_status": list(link.last_status),
            "limit_switches": list(link.limit_switches),  # [A, B, C] touched?
            "vision": vision is not None,
        })

    @app.route("/api/history")
    def api_history():
        since = request.args.get("since", type=float, default=0.0)
        with history_lock:
            samples = [(t, j) for (t, j) in history if t > since]
        return jsonify({
            "t": [s[0] for s in samples],
            "joints": [s[1] for s in samples],
        })

    @app.route("/api/mode", methods=["POST"])
    def api_mode():
        body = request.get_json(silent=True) or {}
        if "auto" not in body:
            return jsonify({"error": "missing 'auto' field"}), 400
        if body["auto"]:
            planner.auto_mode()
        else:
            planner.manual_mode()
        logger.info("mode switched to %s", "auto" if body["auto"] else "manual")
        return jsonify({"mode": "auto" if body["auto"] else "manual"})

    @app.route("/api/move", methods=["POST"])
    def api_move():
        if planner.state_name != "MANUAL":
            return jsonify({"error": "manual moves only allowed in manual mode"}), 409
        body = request.get_json(silent=True) or {}
        angles = [0.0] * 5
        bitmask = 0
        for i, field in enumerate(MOVE_FIELDS):
            if body.get(field) is None or body[field] == "":
                continue
            try:
                angles[i] = float(body[field])
            except (TypeError, ValueError):
                return jsonify({"error": f"field '{field}' is not a number"}), 400
            bitmask |= 1 << i
        if bitmask == 0:
            return jsonify({"error": "no joint angles given"}), 400
        logger.info("manual move: angles=%s bitmask=0x%02X", angles, bitmask)
        ok = link.move_to(angles, bitmask=bitmask)
        return jsonify({"ok": bool(ok), "joints": list(link.current_joints)})

    def require_manual():
        """Return a (json, status) 409 tuple if not in MANUAL mode, else None."""
        if planner.state_name != "MANUAL":
            return jsonify({"error": "only allowed in manual mode"}), 409
        return None

    @app.route("/api/goto", methods=["POST"])
    def api_goto():
        blocked = require_manual()
        if blocked:
            return blocked
        body = request.get_json(silent=True) or {}
        try:
            x, y, z = (float(body[k]) for k in ("x", "y", "z"))
        except (KeyError, TypeError, ValueError):
            return jsonify({"error": "need numeric x, y, z (mm)"}), 400
        angles = solve_ik_first(x, y, z)
        if angles is None:
            return jsonify({"error": "unreachable / outside safe range"}), 422
        logger.info("manual goto: (%.1f, %.1f, %.1f) -> %s", x, y, z, angles)
        ok = link.move_to(angles, bitmask=0x0F)  # joints 1-4 only, never the grip axis
        return jsonify({"ok": bool(ok), "ik": angles, "joints": list(link.current_joints)})

    def _run_command(label, fn):
        """Run a blocking link command in manual mode, turning TimeoutError / serial
        errors into a JSON error instead of a 500 so the UI never wedges."""
        blocked = require_manual()
        if blocked:
            return blocked
        try:
            ok = fn()
        except TimeoutError as e:
            logger.warning("%s timed out: %s", label, e)
            return jsonify({"ok": False, "error": f"{label} timed out"}), 200
        except Exception:
            logger.exception("%s failed", label)
            return jsonify({"ok": False, "error": f"{label} failed"}), 200
        return jsonify({"ok": bool(ok)})

    @app.route("/api/grip", methods=["POST"])
    def api_grip():
        return _run_command("grip", link.grip_close)

    @app.route("/api/release", methods=["POST"])
    def api_release():
        return _run_command("release", link.grip_open)

    @app.route("/api/calibrate", methods=["POST"])
    def api_calibrate():
        return _run_command("calibrate", link.calibrate)

    return app


def start_dashboard(link, vision, planner, host, port):
    """Run the dashboard server on a daemon thread; returns the thread."""
    app = create_dashboard(link, vision, planner)

    def serve():
        # threaded=True: /video holds a connection open per client; use_reloader
        # must be off outside the main thread.
        app.run(host=host, port=port, threaded=True, use_reloader=False, debug=False)

    thread = threading.Thread(target=serve, daemon=True, name="dashboard")
    thread.start()
    logger.info("dashboard on http://%s:%d", host, port)
    return thread
