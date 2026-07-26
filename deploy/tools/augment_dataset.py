"""Photometric augmentation: brightness up/down + slight blur. Boxes don't
move for any of these (no geometry changes), so each augmented image just
gets a copy of its source image's YOLO label file.

Usage:
    python tools/augment_dataset.py --variants undistorted distorted
"""
import argparse
import shutil
from pathlib import Path

import cv2
import numpy as np

REPO = Path(__file__).resolve().parent.parent
DATASET = REPO / "dataset"


def adjust_brightness(img, delta):
    return cv2.convertScaleAbs(img, alpha=1.0, beta=delta)


def slight_blur(img):
    return cv2.GaussianBlur(img, (5, 5), sigmaX=1.2)


AUGMENTATIONS = {
    "bright": lambda img: adjust_brightness(img, 35),
    "dark": lambda img: adjust_brightness(img, -35),
    "blur": slight_blur,
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--variants", nargs="+", default=["undistorted", "distorted"])
    args = ap.parse_args()

    for variant in args.variants:
        vdir = DATASET / "processed" / variant
        photos = sorted(vdir.glob("my_photo-*.jpg"),
                         key=lambda p: int(p.stem.split("-")[1]))
        n_written = 0
        for photo in photos:
            label_path = photo.with_suffix(".txt")
            if not label_path.exists():
                print(f"skip {photo.name}: no label file")
                continue
            img = cv2.imread(str(photo))
            for suffix, fn in AUGMENTATIONS.items():
                out_img = fn(img)
                out_name = f"{photo.stem}_{suffix}"
                cv2.imwrite(str(vdir / f"{out_name}.jpg"), out_img)
                shutil.copyfile(label_path, vdir / f"{out_name}.txt")
                n_written += 1
        print(f"{variant}: augmented {len(photos)} photos -> +{n_written} images")


if __name__ == "__main__":
    main()
