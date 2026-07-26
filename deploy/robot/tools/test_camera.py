#!/usr/bin/env python3
"""Standalone camera + YOLO preview -- no robot/serial connection needed.

Shows each detection's position in both the camera pixel frame ("(u,v)px"
center) and the calibrated robot frame ("(x,y)mm"), plus the workspace ROI
outline (yellow), using the same homography + ROI vision.py uses at runtime --
handy for visually sanity-checking a calibration (from camera_calibrate.py)
before trusting it to drive real pick-and-place moves. Detections inside the
ROI are drawn green, those outside (which the live pipeline drops) are dimmed.

Usage:
    python tools/test_camera.py                # auto-detect camera by USB ID
    python tools/test_camera.py --index 2       # or specify an index directly
"""
import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import cv2
import yaml

import devices
from vision import VisionWorker

CONFIG_PATH = Path(__file__).resolve().parent.parent / "config.yaml"
GREEN = (0, 255, 0)
ORANGE = (0, 165, 255)
YELLOW = (0, 255, 255)
GRAY = (150, 150, 150)


def main():
    vision_cfg = yaml.safe_load(CONFIG_PATH.read_text()).get("vision", {})
    # Reuse VisionWorker's pixel->robot transform (same calibration vision.py uses
    # at runtime) without starting its camera/model thread.
    transform = VisionWorker(camera_index=None, model_path=None, config=vision_cfg)

    ap = argparse.ArgumentParser()
    ap.add_argument("--index", type=int, default=None)
    ap.add_argument("--model", default=str(Path(__file__).resolve().parent.parent / "best.pt"))
    ap.add_argument("--conf", type=float, default=0.7)
    ap.add_argument("--width", type=int, default=vision_cfg.get("frame_width"))
    ap.add_argument("--height", type=int, default=vision_cfg.get("frame_height"))
    ap.add_argument("--snapshot", metavar="PATH", default=None,
                    help="grab ONE frame, print each detection's pixel+robot coords, "
                         "save the raw frame to PATH, and exit (for offline calibration diagnosis)")
    args = ap.parse_args()

    index = args.index if args.index is not None else devices.find_camera_index()
    print(f"Opening camera index {index}")
    api = cv2.CAP_DSHOW if sys.platform == "win32" else cv2.CAP_V4L2
    cap = cv2.VideoCapture(index, api)
    if not cap.isOpened():
        cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        print("Could not open camera")
        sys.exit(1)
    if args.width and args.height:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    print(f"Resolution: {cap.get(cv2.CAP_PROP_FRAME_WIDTH):g}x{cap.get(cv2.CAP_PROP_FRAME_HEIGHT):g}")

    model = None
    try:
        from ultralytics import YOLO

        model = YOLO(args.model)
    except Exception as e:
        print(f"YOLO model not loaded ({e}); showing raw camera feed only")

    if args.snapshot:
        # Grab a few frames so auto-exposure/focus settle, then keep the last.
        frame = None
        for _ in range(10):
            ret, f = cap.read()
            if ret:
                frame = f
        cap.release()
        if frame is None:
            print("Could not capture a frame")
            sys.exit(1)
        raw = frame.copy()
        print(f"Captured {frame.shape[1]}x{frame.shape[0]} frame")
        transform.set_frame_size(frame.shape[1], frame.shape[0])
        if model is not None:
            for r in model(frame, conf=args.conf, verbose=False):
                for box in r.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    name = model.names[int(box.cls[0])]
                    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                    x_mm, y_mm = transform.pixel_to_robot(cx, cy)
                    tag = "in-ROI" if transform._in_roi(x_mm, y_mm) else "OUT"
                    print(f"  {name:8s} pixel=({cx:4d},{cy:4d})  ->  robot=({x_mm:7.1f},{y_mm:7.1f}) mm  [{tag}]")
        cv2.imwrite(args.snapshot, raw)
        print(f"Saved raw frame to {args.snapshot}")
        return

    print("Press 'q' to quit.")
    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        transform.set_frame_size(frame.shape[1], frame.shape[0])
        if model is not None:
            cv2.polylines(frame, [transform._roi_px], True, YELLOW, 2)  # workspace ROI
            results = model(frame, conf=args.conf, verbose=False)
            for r in results:
                for box in r.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    name = model.names[int(box.cls[0])]
                    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                    x_mm, y_mm = transform.pixel_to_robot(cx, cy)
                    in_roi = transform._in_roi(x_mm, y_mm)
                    box_color = GREEN if in_roi else GRAY
                    mm_color = ORANGE if in_roi else GRAY

                    cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, 2)
                    cv2.drawMarker(frame, (cx, cy), box_color, cv2.MARKER_CROSS, 10, 2)
                    cv2.putText(frame, f"{name} ({cx},{cy})px", (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 2)
                    label_mm = f"({x_mm:.1f},{y_mm:.1f})mm" + ("" if in_roi else " OUT")
                    cv2.putText(frame, label_mm, (x1, y2 + 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, mm_color, 2)

            origin_x, origin_y = transform._origin_pixel()
            cv2.drawMarker(frame, (origin_x, origin_y), ORANGE, cv2.MARKER_TILTED_CROSS, 16, 2)
            cv2.putText(frame, "robot (0,0)", (origin_x + 10, origin_y + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, ORANGE, 2)
        cv2.imshow("Camera Test", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
