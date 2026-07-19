#!/usr/bin/env python3
"""Standalone camera + YOLO preview -- no robot/serial connection needed.

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

CONFIG_PATH = Path(__file__).resolve().parent.parent / "config.yaml"


def main():
    vision_cfg = yaml.safe_load(CONFIG_PATH.read_text()).get("vision", {})

    ap = argparse.ArgumentParser()
    ap.add_argument("--index", type=int, default=None)
    ap.add_argument("--model", default=str(Path(__file__).resolve().parent.parent / "best.pt"))
    ap.add_argument("--conf", type=float, default=0.7)
    ap.add_argument("--width", type=int, default=vision_cfg.get("frame_width"))
    ap.add_argument("--height", type=int, default=vision_cfg.get("frame_height"))
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

    print("Press 'q' to quit.")
    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        if model is not None:
            results = model(frame, conf=args.conf, verbose=False)
            for r in results:
                for box in r.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    name = model.names[int(box.cls[0])]
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, name, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.imshow("Camera Test", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
