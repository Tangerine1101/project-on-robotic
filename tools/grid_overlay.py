"""Draw a labeled pixel-coordinate grid over an image, to help estimate
bounding-box pixel coordinates by eye (used for manual dataset labeling since
locate-anything.cpp isn't usable here - see project owner discussion).

Usage:
    python tools/grid_overlay.py <image_path> [--spacing 40] [--out out.jpg]
"""
import argparse
from pathlib import Path

import cv2
import numpy as np


def draw_grid(img, spacing=40):
    out = img.copy()
    h, w = out.shape[:2]
    for x in range(0, w, spacing):
        color = (0, 200, 255) if x % (spacing * 5) == 0 else (60, 60, 60)
        cv2.line(out, (x, 0), (x, h), color, 1)
        if x % (spacing * 5) == 0:
            cv2.putText(out, str(x), (x + 2, 14), cv2.FONT_HERSHEY_SIMPLEX,
                        0.4, (0, 0, 255), 1, cv2.LINE_AA)
    for y in range(0, h, spacing):
        color = (0, 200, 255) if y % (spacing * 5) == 0 else (60, 60, 60)
        cv2.line(out, (0, y), (w, y), color, 1)
        if y % (spacing * 5) == 0:
            cv2.putText(out, str(y), (2, y + 12), cv2.FONT_HERSHEY_SIMPLEX,
                        0.4, (0, 0, 255), 1, cv2.LINE_AA)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("image")
    ap.add_argument("--spacing", type=int, default=40)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    img = cv2.imread(args.image)
    if img is None:
        raise SystemExit(f"cannot read {args.image}")
    gridded = draw_grid(img, args.spacing)
    out_path = args.out or str(Path(args.image).with_suffix(".grid.jpg"))
    cv2.imwrite(out_path, gridded)
    print(out_path)


if __name__ == "__main__":
    main()
