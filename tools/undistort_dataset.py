"""Normalize the 24 training photos to the 640x480 format the camera will run
at in deployment.

For each dataset/my_photo-N.jpg (1280x720) this writes two 640x480 variants
(object positions differ slightly between them, since undistort shifts pixels
- each gets labeled independently later, not by copying the other's boxes):
  - dataset/processed/undistorted/my_photo-N.jpg : undistort -> center-crop
    16:9 -> 4:3 -> resize to 640x480
  - dataset/processed/distorted/my_photo-N.jpg   : center-crop -> resize only
    (kept as-is as extra data diversity, per project owner's request)

Usage:
    python tools/undistort_dataset.py
"""
import json
from pathlib import Path

import cv2
import numpy as np
import yaml

REPO = Path(__file__).resolve().parent.parent
DATASET = REPO / "dataset"
CALIB_DIR = DATASET / "calib"
OUT_UNDIST = DATASET / "processed" / "undistorted"
OUT_DIST = DATASET / "processed" / "distorted"

TARGET_W, TARGET_H = 640, 480


def load_intrinsics():
    data = yaml.safe_load((CALIB_DIR / "camera_intrinsics.yaml").read_text())
    K = np.array(data["camera_matrix"], dtype=np.float64)
    dist = np.array(data["dist_coeffs"], dtype=np.float64)
    w, h = data["image_width"], data["image_height"]
    return K, dist, w, h


def center_crop_16x9_to_4x3(img):
    h, w = img.shape[:2]
    target_w = int(round(h * 4 / 3))
    x0 = (w - target_w) // 2
    return img[:, x0:x0 + target_w], x0


def main():
    K, dist, w, h = load_intrinsics()
    new_K, _ = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), 0.0, (w, h))

    OUT_UNDIST.mkdir(parents=True, exist_ok=True)
    OUT_DIST.mkdir(parents=True, exist_ok=True)

    photos = sorted(
        DATASET.glob("my_photo-*.jpg"),
        key=lambda p: int(p.stem.split("-")[1]),
    )
    if not photos:
        raise SystemExit(f"no my_photo-*.jpg found in {DATASET}")

    crop_x0 = None
    for path in photos:
        img = cv2.imread(str(path))

        und = cv2.undistort(img, K, dist, None, new_K)
        und_crop, crop_x0 = center_crop_16x9_to_4x3(und)
        und_final = cv2.resize(und_crop, (TARGET_W, TARGET_H), interpolation=cv2.INTER_AREA)
        cv2.imwrite(str(OUT_UNDIST / path.name), und_final)

        dist_crop, _ = center_crop_16x9_to_4x3(img)
        dist_final = cv2.resize(dist_crop, (TARGET_W, TARGET_H), interpolation=cv2.INTER_AREA)
        cv2.imwrite(str(OUT_DIST / path.name), dist_final)

    scale = TARGET_W / (h * 4 / 3)
    transform_meta = {
        "source_size": [w, h],
        "crop_x0": crop_x0,
        "crop_size": [int(round(h * 4 / 3)), h],
        "resize_to": [TARGET_W, TARGET_H],
        "scale": scale,
        "note": (
            "pixel(x,y) in a processed 640x480 image maps back to the raw "
            "1280x720 frame as raw_x = x/scale + crop_x0, raw_y = y/scale "
            "(undistorted variant additionally needs the inverse of "
            "cv2.undistort with camera_intrinsics.yaml's K/dist applied "
            "first). Kept for reuse when wiring this into robot/vision.py."
        ),
    }
    (DATASET / "processed" / "transform.json").write_text(json.dumps(transform_meta, indent=2))

    print(f"wrote {len(photos)} undistorted -> {OUT_UNDIST}")
    print(f"wrote {len(photos)} distorted   -> {OUT_DIST}")
    print(f"wrote {DATASET / 'processed' / 'transform.json'}")


if __name__ == "__main__":
    main()
