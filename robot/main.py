#!/usr/bin/env python3
"""Robot arm controller -- replaces the ROS2 stack (ros2/src/robot_pkg).

Usage:
    python main.py                 # full auto: vision + planner + serial + web dashboard
    python main.py --no-vision     # no camera; drive the arm manually via tools/cli.py
    python main.py --manual        # planner stays idle; drive via tools/cli.py or the dashboard
    python main.py --no-dashboard  # don't start the web dashboard

The dashboard (camera stream, joint chart, manual control) is served on
http://<host>:<port> from the `dashboard:` section of config.yaml.

See README.md for setup and TODO.md for what's still open.
"""
import argparse
import logging
import signal
import sys
import threading
import time
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
import devices
from planner import Planner
from serial_link import SerialLink
from vision import VisionWorker


def load_config(path):
    with open(path) as f:
        return yaml.safe_load(f)


def resolve_port(cfg):
    port = cfg["serial"].get("port")
    if not port or port == "auto":
        return devices.find_mcu_port()
    return port


def resolve_camera_index(cfg):
    idx = cfg["vision"].get("camera_index")
    if idx is None or idx == "auto":
        return devices.find_camera_index()
    return int(idx)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=str(Path(__file__).resolve().parent / "config.yaml"))
    ap.add_argument("--no-vision", action="store_true", help="run without the camera/YOLO pipeline")
    ap.add_argument("--manual", action="store_true", help="start idle; drive the arm with tools/cli.py or the dashboard")
    ap.add_argument("--no-dashboard", action="store_true", help="don't start the web dashboard")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(name)-12s %(levelname)-7s %(message)s",
        datefmt="%H:%M:%S",  # time-only: readable when watching the live terminal
    )
    log = logging.getLogger("main")

    cfg = load_config(args.config)

    port = resolve_port(cfg)
    log.info("MCU on %s", port)
    link = SerialLink(port, baudrate=cfg["serial"]["baudrate"], config=cfg["serial"])

    vision = None
    if not args.no_vision:
        cam_index = resolve_camera_index(cfg)
        log.info("Camera index %s", cam_index)
        model_path = Path(__file__).resolve().parent / cfg["vision"]["model_path"]
        vision = VisionWorker(cam_index, model_path, cfg["vision"])
        vision.start()
    else:
        cfg["planner"]["wait_vision"] = False

    planner = Planner(link, vision, cfg)
    if args.manual:
        planner.manual_mode()
        log.info("Starting in MANUAL mode -- use tools/cli.py or the dashboard to drive the arm.")

    dash_cfg = cfg.get("dashboard", {})
    if not args.no_dashboard and dash_cfg.get("enabled", True):
        from dashboard import start_dashboard  # deferred: flask only needed here
        start_dashboard(
            link, vision, planner,
            host=dash_cfg.get("host", "0.0.0.0"),
            port=int(dash_cfg.get("port", 8000)),
        )

    stop_evt = threading.Event()

    def handle_sigint(signum, frame):
        log.info("Shutting down...")
        stop_evt.set()

    signal.signal(signal.SIGINT, handle_sigint)

    planner_thread = threading.Thread(target=planner.run_forever, daemon=True, name="planner")
    planner_thread.start()

    try:
        while not stop_evt.is_set():
            time.sleep(0.2)
    finally:
        planner.stop()
        if vision:
            vision.stop()
        link.close()


if __name__ == "__main__":
    main()
