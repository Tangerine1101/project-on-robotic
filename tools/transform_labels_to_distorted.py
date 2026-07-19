"""Derive bounding boxes for the distorted/ image variant from the manually
labeled undistorted/ variant, by exactly inverting the crop/scale/undistort
chain from tools/undistort_dataset.py - instead of re-labeling by hand.

Object positions differ by up to ~9px (640x480 scale) between the two
variants (measured from camera_intrinsics.yaml's k1), non-negligible for
~60px objects, so we can't just copy boxes as-is; but since the geometric
transform is fully known, we can map box corners through it precisely.

Usage:
    python tools/transform_labels_to_distorted.py
Reads:  dataset/processed/undistorted/my_photo-N.json (labelme format)
Writes: dataset/processed/distorted/my_photo-N.json   (labelme format)
"""
import json
from pathlib import Path

import cv2
import numpy as np
import yaml

REPO = Path(__file__).resolve().parent.parent
DATASET = REPO / "dataset"
CALIB_DIR = DATASET / "calib"
UNDIST_DIR = DATASET / "processed" / "undistorted"
DIST_DIR = DATASET / "processed" / "distorted"

TARGET_W, TARGET_H = 640, 480


def load_intrinsics():
    data = yaml.safe_load((CALIB_DIR / "camera_intrinsics.yaml").read_text())
    K = np.array(data["camera_matrix"], dtype=np.float64)
    dist = np.array(data["dist_coeffs"], dtype=np.float64)
    w, h = data["image_width"], data["image_height"]
    return K, dist, w, h


def undistorted_px_to_distorted_px(pts_uv, K, dist, new_K):
    """Inverse of what cv2.undistort resamples: given pixel coords in the
    undistorted/new_K image, find where they came from in the original
    distorted image (K, dist)."""
    pts_uv = np.asarray(pts_uv, dtype=np.float64).reshape(-1, 1, 2)
    fx, fy, cx, cy = new_K[0, 0], new_K[1, 1], new_K[0, 2], new_K[1, 2]
    xn = (pts_uv[:, 0, 0] - cx) / fx
    yn = (pts_uv[:, 0, 1] - cy) / fy
    obj_pts = np.stack([xn, yn, np.ones_like(xn)], axis=1).reshape(-1, 1, 3)
    rvec = np.zeros(3)
    tvec = np.zeros(3)
    img_pts, _ = cv2.projectPoints(obj_pts, rvec, tvec, K, dist)
    return img_pts.reshape(-1, 2)


def box_to_points(x1, y1, x2, y2, n_per_edge=3):
    """Sample points along the box perimeter (not just corners) so the
    axis-aligned bbox of the transformed points stays accurate even though
    distortion can bow the box's edges slightly."""
    ts = np.linspace(0, 1, n_per_edge)
    pts = []
    for t in ts:
        pts.append((x1 + t * (x2 - x1), y1))  # top edge
        pts.append((x1 + t * (x2 - x1), y2))  # bottom edge
        pts.append((x1, y1 + t * (y2 - y1)))  # left edge
        pts.append((x2, y1 + t * (y2 - y1)))  # right edge
    return np.array(pts)


def main():
    K, dist, raw_w, raw_h = load_intrinsics()
    new_K, _ = cv2.getOptimalNewCameraMatrix(K, dist, (raw_w, raw_h), 0.0, (raw_w, raw_h))

    crop_w = int(round(raw_h * 4 / 3))
    crop_x0 = (raw_w - crop_w) // 2
    scale = TARGET_W / crop_w

    DIST_DIR.mkdir(parents=True, exist_ok=True)
    json_files = sorted(UNDIST_DIR.glob("my_photo-*.json"),
                         key=lambda p: int(p.stem.split("-")[1]))
    if not json_files:
        raise SystemExit(f"no labelme json found in {UNDIST_DIR}")

    n_boxes_total = 0
    n_clipped = 0
    for jpath in json_files:
        d = json.loads(jpath.read_text())
        out_shapes = []
        for shape in d["shapes"]:
            pts = np.array(shape["points"], dtype=np.float64)
            x1, y1 = pts.min(axis=0)
            x2, y2 = pts.max(axis=0)

            # 640x480 undistorted-crop coords -> raw undistorted 1280x720 coords
            pts_640 = box_to_points(x1, y1, x2, y2)
            pts_raw_undist = pts_640 / scale
            pts_raw_undist[:, 0] += crop_x0

            # raw undistorted -> raw distorted (inverse of cv2.undistort)
            pts_raw_dist = undistorted_px_to_distorted_px(pts_raw_undist, K, dist, new_K)

            # raw distorted 1280x720 -> 640x480 distorted-crop coords
            pts_out = pts_raw_dist.copy()
            pts_out[:, 0] -= crop_x0
            pts_out *= scale

            nx1, ny1 = pts_out.min(axis=0)
            nx2, ny2 = pts_out.max(axis=0)
            clipped = (nx1 < 0 or ny1 < 0 or nx2 > TARGET_W or ny2 > TARGET_H)
            n_clipped += int(clipped)
            nx1, ny1 = max(nx1, 0), max(ny1, 0)
            nx2, ny2 = min(nx2, TARGET_W), min(ny2, TARGET_H)

            out_shapes.append({
                **shape,
                "points": [[float(nx1), float(ny1)], [float(nx2), float(ny1)],
                           [float(nx2), float(ny2)], [float(nx1), float(ny2)]],
            })
            n_boxes_total += 1

        out = {**d, "shapes": out_shapes, "imagePath": jpath.stem + ".jpg"}
        (DIST_DIR / jpath.name).write_text(json.dumps(out, indent=2))

    print(f"wrote {len(json_files)} distorted-variant label jsons "
          f"({n_boxes_total} boxes, {n_clipped} clipped at frame edge) -> {DIST_DIR}")


if __name__ == "__main__":
    main()
