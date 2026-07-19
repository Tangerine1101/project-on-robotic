#!/usr/bin/env python3
"""Camera<->robot calibration helper.

Opens the camera, runs the YOLO model (as a visual reference only), and asks the
user for 3 point correspondences: for each of A, B, C, the robot coordinate
(X, Y in mm) and the matching pixel (u, v) read off the preview window. From
those it fits the 2D camera->robot transform and writes it to config.yaml's
`vision:` block, where vision.py picks it up.

Transform model (see vision.py.pixel_to_robot): a 2D similarity with an optional
z-flip (handedness) reflection ::

    p = u + i*v          (or u - i*v when z_flip)
    r = a*p + b          r = X_mm + i*Y_mm,  unknown complex a, b

    rotate        = degrees(angle(a))
    pixels_per_mm = 1 / abs(a)
    x_0_mm, y_0_mm = b.real, b.imag

Usage:
    python camera_calibrate.py                 # detect camera, prompt, write
    python camera_calibrate.py --no-write      # compute and print only
"""
import argparse
import cmath
import math
import re
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
import devices

CONFIG_PATH = Path(__file__).resolve().parent / "config.yaml"
POINT_NAMES = ("A", "B", "C")


# --------------------------------------------------------------------- transform

def fit_transform(pixels, robots, z_flip):
    """Least-squares fit r = a*p + b over the correspondences.

    pixels/robots: lists of (u, v) / (x_mm, y_mm). Returns (a, b, max_err, mean_err).
    """
    p = np.array([_to_complex(u, v, z_flip) for (u, v) in pixels])
    r = np.array([complex(x, y) for (x, y) in robots])
    M = np.column_stack([p, np.ones_like(p)])
    (a, b), *_ = np.linalg.lstsq(M, r, rcond=None)
    resid = np.abs(a * p + b - r)
    return a, b, float(resid.max()), float(resid.mean())


def _to_complex(u, v, z_flip):
    return complex(u, -v) if z_flip else complex(u, v)


def decompose(a, b):
    return {
        "rotate": math.degrees(cmath.phase(a)),
        "pixels_per_mm": 1.0 / abs(a),
        "x_0_mm": b.real,
        "y_0_mm": b.imag,
    }


# ------------------------------------------------------------------------- camera

def open_camera(cfg):
    idx = cfg["vision"].get("camera_index")
    idx = devices.find_camera_index() if (idx is None or idx == "auto") else int(idx)
    api = cv2.CAP_DSHOW if sys.platform == "win32" else cv2.CAP_V4L2
    cap = cv2.VideoCapture(idx, api)
    if not cap.isOpened():
        cap = cv2.VideoCapture(idx)
    if not cap.isOpened():
        raise RuntimeError(f"Camera index {idx} could not be opened")
    w, h = cfg["vision"].get("frame_width"), cfg["vision"].get("frame_height")
    if w and h:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(w))
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(h))
    print(f"Camera resolution: {cap.get(cv2.CAP_PROP_FRAME_WIDTH):g}x{cap.get(cv2.CAP_PROP_FRAME_HEIGHT):g}")
    return cap


def preview_and_read_pixels(cap, model):
    """Show a live annotated window with a cursor-pixel overlay so the user can
    read off pixel coordinates. Click logs the pixel to the console. Press 'q'
    (or ESC) to close the window and continue to the text prompts."""
    cursor = {"u": 0, "v": 0}

    def on_mouse(event, x, y, flags, param):
        cursor["u"], cursor["v"] = x, y
        if event == cv2.EVENT_LBUTTONDOWN:
            print(f"  clicked pixel: {x} {y}")

    win = "Calibration -- hover to read pixels, click to log, 'q' to continue"
    cv2.namedWindow(win)
    cv2.setMouseCallback(win, on_mouse)
    print("Preview open: hover the mouse to read (u, v); press 'q' when done.")
    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        try:
            for r in model(frame, verbose=False):
                for box in r.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        except Exception:
            pass
        u, v = cursor["u"], cursor["v"]
        cv2.drawMarker(frame, (u, v), (0, 0, 255), cv2.MARKER_CROSS, 16, 1)
        cv2.putText(frame, f"pixel: {u} {v}", (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.imshow(win, frame)
        if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
            break
    cv2.destroyWindow(win)


# -------------------------------------------------------------------------- input

def _prompt_pair(prompt):
    while True:
        raw = input(prompt).replace(",", " ").split()
        try:
            a, b = (float(x) for x in raw)
            return a, b
        except ValueError:
            print("  please enter two numbers, e.g. '120 -45'")


def _prompt_bool(prompt):
    while True:
        raw = input(prompt).strip().lower()
        if raw in ("y", "yes", "true", "1"):
            return True
        if raw in ("n", "no", "false", "0"):
            return False
        print("  please answer y/n")


def collect_correspondences():
    z_flip = _prompt_bool("Is the camera z-axis flipped vs the robot? (y/n): ")
    pixels, robots = [], []
    for name in POINT_NAMES:
        print(f"-- Point {name} --")
        robots.append(_prompt_pair(f"  robot   X_mm Y_mm for {name}: "))
        pixels.append(_prompt_pair(f"  camera  u v (pixels) for {name}: "))
    return pixels, robots, z_flip


# --------------------------------------------------------------------- yaml write

def update_vision_config(text, values):
    """Replace/insert keys inside the top-level `vision:` block, preserving
    comments and the rest of the file. `values` maps key -> formatted string."""
    lines = text.splitlines(keepends=True)
    start = next((i for i, ln in enumerate(lines) if re.match(r"^vision:\s*(#.*)?$", ln)), None)
    if start is None:
        raise RuntimeError("no top-level 'vision:' block found in config.yaml")
    end = len(lines)
    for i in range(start + 1, len(lines)):
        if lines[i].strip() and not lines[i][0].isspace():  # next top-level key
            end = i
            break

    remaining = dict(values)
    for i in range(start + 1, end):
        m = re.match(r"^(\s*)([A-Za-z0-9_]+):(\s*)([^#\n]*)(#.*)?(\r?\n?)$", lines[i])
        if not m:
            continue
        key = m.group(2)
        if key in remaining:
            indent, sep = m.group(1), m.group(3) or " "
            comment = m.group(5) or ""
            comment = ("  " + comment) if comment else ""
            nl = m.group(6) or "\n"
            lines[i] = indent + key + ":" + sep + remaining.pop(key) + comment + nl

    # Insert any keys that weren't already present, right after the header.
    if remaining:
        insert = [f"  {k}: {v}\n" for k, v in remaining.items()]
        lines[start + 1:start + 1] = insert
    return "".join(lines)


def format_values(params, z_flip):
    return {
        "x_0_mm": f"{params['x_0_mm']:.3f}",
        "y_0_mm": f"{params['y_0_mm']:.3f}",
        "pixels_per_mm": f"{params['pixels_per_mm']:.4f}",
        "rotate": f"{params['rotate']:.3f}",
        "z_flip": "true" if z_flip else "false",
    }


# --------------------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--no-write", action="store_true", help="compute and print only; don't touch config.yaml")
    args = ap.parse_args()

    cfg = yaml.safe_load(CONFIG_PATH.read_text())

    cap = open_camera(cfg)
    try:
        from ultralytics import YOLO  # deferred: heavy import
        model_path = Path(__file__).resolve().parent / cfg["vision"]["model_path"]
        print(f"Loading model {model_path} ...")
        model = YOLO(str(model_path))
        preview_and_read_pixels(cap, model)
    finally:
        cap.release()
        cv2.destroyAllWindows()

    pixels, robots, z_flip = collect_correspondences()
    a, b, max_err, mean_err = fit_transform(pixels, robots, z_flip)
    params = decompose(a, b)

    print("\n=== Calibration result ===")
    print(f"  rotate        : {params['rotate']:.3f} deg")
    print(f"  z_flip        : {z_flip}")
    print(f"  pixels_per_mm : {params['pixels_per_mm']:.4f}")
    print(f"  x_0_mm        : {params['x_0_mm']:.3f}")
    print(f"  y_0_mm        : {params['y_0_mm']:.3f}")
    print(f"  fit residual  : max {max_err:.2f} mm, mean {mean_err:.2f} mm")
    if max_err > 5.0:
        print("  WARNING: residual is large -- check the typed points / z_flip.")

    values = format_values(params, z_flip)
    print("\nvision: block update:")
    for k, v in values.items():
        print(f"  {k}: {v}")

    if args.no_write:
        print("\n--no-write: config.yaml left unchanged.")
        return
    if not _prompt_bool("\nWrite these into config.yaml? (y/n): "):
        print("Not written.")
        return
    CONFIG_PATH.write_text(update_vision_config(CONFIG_PATH.read_text(), values))
    print(f"Wrote calibration to {CONFIG_PATH}")


if __name__ == "__main__":
    main()
