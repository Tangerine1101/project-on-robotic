#!/usr/bin/env python3
"""Camera<->robot calibration from a checkerboard.

Produces the pixel->robot mapping the live pipeline uses, as a **homography**
(3x3, handles the perspective of a tilted table), plus an **ROI** (the board
rectangle grown by `roi_extend_mm`, oriented to the board's tilt). Results are
written to a standalone calibration file (default robot/camera_calibration.yaml)
that vision.py loads; config.yaml only points at it.

Homography-only mode: no separate lens-distortion step and no per-frame
undistort -- the homography from ~cols*rows board corners absorbs the planar
perspective, which is enough for a low-distortion camera whose workspace stays
within the board+margin ROI. (The calibration file leaves camera_matrix/
dist_coeffs unset; a future multi-view intrinsic mode can fill them in.)

Two-pass workflow (the board must NOT move between passes):

  Pass 1 -- place the printed board (tools/make_checkerboard.py) flat in the
    workspace and run:
        python camera_calibrate.py
    It detects the board and saves dataset/calib/checkerboard_detected.jpg with
    three corners labelled BL / BR / TL (an L shape -- non-collinear, so they fix
    the board->robot frame including handedness). Touch the robot tip to those
    three physical corners, read the robot X,Y (mm), and fill robot_points_mm in
    robot/checkerboard.json.

  Pass 2 -- with the board still in place, run it again. It fits board->robot
    from your 3 points, maps every corner into the robot frame, fits the
    pixel->robot homography, builds the ROI, and writes the calibration file.

Usage:
    python camera_calibrate.py                 # live camera, write calibration
    python camera_calibrate.py --no-write       # compute + print, don't write
    python camera_calibrate.py --image shot.jpg # calibrate from a saved frame
"""
import argparse
import cmath
import json
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
import devices

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
CONFIG_PATH = HERE / "config.yaml"
CHECKERBOARD_JSON = HERE / "checkerboard.json"
DETECT_IMG = REPO / "dataset" / "calib" / "checkerboard_detected.jpg"
OVERLAY_IMG = REPO / "dataset" / "calib" / "checkerboard_calibrated.jpg"

GREEN = (0, 255, 0)
ORANGE = (0, 165, 255)
YELLOW = (0, 255, 255)
RED = (0, 0, 255)


# --------------------------------------------------- board-mm -> robot-mm fit

def _to_complex(x, y, z_flip):
    return complex(x, -y) if z_flip else complex(x, y)


def fit_similarity(src_xy, dst_xy, z_flip):
    """Least-squares 2D similarity r = a*p + b mapping src points to dst.
    Returns (a, b, max_err, mean_err). Reused for board-mm -> robot-mm; z_flip
    handles a handedness (mirror) difference between the two frames."""
    p = np.array([_to_complex(x, y, z_flip) for (x, y) in src_xy])
    r = np.array([complex(x, y) for (x, y) in dst_xy])
    M = np.column_stack([p, np.ones_like(p)])
    (a, b), *_ = np.linalg.lstsq(M, r, rcond=None)
    resid = np.abs(a * p + b - r)
    return a, b, float(resid.max()), float(resid.mean())


def apply_similarity(a, b, z_flip, pts_xy):
    out = []
    for (x, y) in pts_xy:
        r = a * _to_complex(x, y, z_flip) + b
        out.append((r.real, r.imag))
    return out


# --------------------------------------------------------- board detection

def detect_board(gray, cols, rows):
    """Return (ok, corners Nx2 float32) for a (cols x rows) inner-corner grid."""
    flags = (cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE
             | cv2.CALIB_CB_ACCURACY)
    ok, corners = cv2.findChessboardCornersSB(gray, (cols, rows), flags=flags)
    if not ok:
        ok, corners = cv2.findChessboardCorners(gray, (cols, rows), None)
        if ok:
            crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), crit)
    if not ok:
        return False, None
    return True, corners.reshape(-1, 2).astype(np.float32)


def board_geometry(cols, rows, square):
    """board-mm of every inner corner (row-major, matching findChessboardCorners)."""
    return np.array([[(k % cols) * square, (k // cols) * square]
                     for k in range(cols * rows)], dtype=np.float64)


# The 3 anchors are three of the four extreme inner corners forming an L
# (bottom_left, bottom_right, top_left) -- NON-collinear, so they fully fix the
# board->robot frame including handedness. (center + two diagonal corners would
# be collinear -> handedness unresolvable -> a mirrored calibration could pass
# with a small reprojection error.) Corners are also sharper physical targets to
# touch with the robot tip than the board's unmarked geometric center.
ANCHOR_KEYS = ("bottom_left", "bottom_right", "top_left")


def pick_extreme_corners(corners_xy, cols, rows):
    """Classify the 4 extreme inner corners by image position; return a dict of
    flat indices {tl, tr, bl, br}. Assumes a roughly upright board."""
    n = cols * rows
    extremes = [0, cols - 1, n - cols, n - 1]
    return {
        "tl": min(extremes, key=lambda k: corners_xy[k][0] + corners_xy[k][1]),
        "br": max(extremes, key=lambda k: corners_xy[k][0] + corners_xy[k][1]),
        "tr": max(extremes, key=lambda k: corners_xy[k][0] - corners_xy[k][1]),
        "bl": min(extremes, key=lambda k: corners_xy[k][0] - corners_xy[k][1]),
    }


def save_detect_image(frame, corners_xy, cols, rows, anchor_idx):
    disp = frame.copy()
    cv2.drawChessboardCorners(disp, (cols, rows),
                              corners_xy.reshape(-1, 1, 2), True)
    labels = {"bottom_left": "BL", "bottom_right": "BR", "top_left": "TL"}
    for key in ANCHOR_KEYS:
        pt = corners_xy[anchor_idx[{"bottom_left": "bl", "bottom_right": "br", "top_left": "tl"}[key]]]
        p = (int(round(pt[0])), int(round(pt[1])))
        cv2.drawMarker(disp, p, RED, cv2.MARKER_CROSS, 24, 3)
        cv2.putText(disp, labels[key], (p[0] + 12, p[1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, RED, 2, cv2.LINE_AA)
    DETECT_IMG.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(DETECT_IMG), disp)


# ------------------------------------------------------------------ camera

def open_camera(cfg, index):
    idx = index
    if idx is None:
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


def capture_frame(cfg, index):
    """Live preview; press 'c' to grab the current frame, 'q'/ESC to abort."""
    cap = open_camera(cfg, index)
    win = "Calibration -- position the whole board in view, press 'c' to capture ('q' aborts)"
    cv2.namedWindow(win)
    print("Preview open: press 'c' to capture the board, 'q' to abort.")
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                continue
            disp = frame.copy()
            cv2.putText(disp, "press 'c' to capture, 'q' to abort", (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, GREEN, 2)
            cv2.imshow(win, disp)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("c"):
                return frame
            if key in (ord("q"), 27):
                return None
    finally:
        cap.release()
        cv2.destroyAllWindows()


# ----------------------------------------------------------------- outputs

def build_roi(cols, rows, square, roi_extend, a, b, z_flip, H_inv):
    """ROI = board bounding rect grown by roi_extend mm on every side. Returns
    (roi_mm 4x2, roi_px 4x2)."""
    w = (cols - 1) * square
    h = (rows - 1) * square
    e = roi_extend
    roi_board = [(-e, -e), (w + e, -e), (w + e, h + e), (-e, h + e)]
    roi_mm = np.array(apply_similarity(a, b, z_flip, roi_board), dtype=np.float64)
    roi_px = cv2.perspectiveTransform(
        roi_mm.reshape(1, -1, 2).astype(np.float64), H_inv).reshape(-1, 2)
    return roi_mm, roi_px


def save_overlay_image(frame, corners_xy, robots_mm, H_inv, roi_px):
    disp = frame.copy()
    cv2.polylines(disp, [roi_px.round().astype(np.int32)], True, YELLOW, 2)
    reproj = cv2.perspectiveTransform(
        robots_mm.reshape(1, -1, 2), H_inv).reshape(-1, 2)
    for (dx, dy), (rx, ry) in zip(corners_xy, reproj):
        cv2.circle(disp, (int(dx), int(dy)), 3, GREEN, -1)      # detected
        cv2.drawMarker(disp, (int(rx), int(ry)), RED, cv2.MARKER_TILTED_CROSS, 6, 1)  # reprojected
    cv2.imwrite(str(OVERLAY_IMG), disp)


def write_calibration(path, frame, H, roi_mm, roi_px, err_mm, err_px, anchors_robot):
    h_img, w_img = frame.shape[:2]
    data = {
        "image_width": int(w_img),
        "image_height": int(h_img),
        "homography_px_to_mm": H.tolist(),
        "roi_polygon_px": roi_px.tolist(),
        "roi_polygon_mm": roi_mm.tolist(),
        "reprojection_error_mm": round(err_mm, 4),
        "reprojection_error_px": round(err_px, 4),
        "board_pose_mm": anchors_robot,
    }
    header = ("# Generated by camera_calibrate.py from a checkerboard view.\n"
              "# Do not edit by hand -- re-run camera_calibrate.py to regenerate.\n")
    path.write_text(header + yaml.safe_dump(data, sort_keys=False))


# -------------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--index", type=int, default=None, help="camera index override")
    ap.add_argument("--image", default=None, help="calibrate from a saved frame instead of live camera")
    ap.add_argument("--no-write", action="store_true", help="compute + print only; don't write calibration file")
    args = ap.parse_args()

    cfg = yaml.safe_load(CONFIG_PATH.read_text())
    vision_cfg = cfg["vision"]
    board = json.loads(CHECKERBOARD_JSON.read_text())
    cols, rows = int(board["pattern_cols"]), int(board["pattern_rows"])
    square = float(board["square_mm"])
    roi_extend = float(vision_cfg.get("roi_extend_mm", 50.0))
    calib_path = HERE / vision_cfg.get("calibration_file", "camera_calibration.yaml")

    # ---- get a frame ----
    if args.image:
        frame = cv2.imread(args.image)
        if frame is None:
            print(f"Could not read {args.image}")
            sys.exit(1)
    else:
        frame = capture_frame(cfg, args.index)
        if frame is None:
            print("Aborted.")
            sys.exit(1)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ok, corners_xy = detect_board(gray, cols, rows)
    if not ok:
        print(f"Checkerboard ({cols}x{rows} inner corners) not found. "
              "Make sure the whole board is visible, flat, and well lit, then retry.")
        sys.exit(1)
    print(f"Detected {cols}x{rows} = {cols * rows} inner corners.")

    board_mm = board_geometry(cols, rows, square)
    ext = pick_extreme_corners(corners_xy, cols, rows)
    anchor_idx = {"bottom_left": ext["bl"], "bottom_right": ext["br"], "top_left": ext["tl"]}
    save_detect_image(frame, corners_xy, cols, rows, ext)
    print(f"Saved labelled detection image: {DETECT_IMG}")

    rp = board["robot_points_mm"]
    filled = all(k in rp and rp[k][0] is not None and rp[k][1] is not None for k in ANCHOR_KEYS)
    if not filled:
        print("\n--- PASS 1 ---")
        print(f"Open {DETECT_IMG.name}: touch the robot tip to the 3 labelled corners and")
        print("fill robot_points_mm in robot/checkerboard.json (mm), then re-run:")
        print("  bottom_left  = the RED 'BL' corner")
        print("  bottom_right = the RED 'BR' corner")
        print("  top_left     = the RED 'TL' corner")
        print("Keep the board EXACTLY in place between now and the next run.")
        return

    # ---- PASS 2: board->robot from the 3 non-collinear anchor corners ----
    src = [tuple(board_mm[anchor_idx[k]]) for k in ANCHOR_KEYS]
    dst = [tuple(rp[k]) for k in ANCHOR_KEYS]
    best = None
    for z_flip in (False, True):
        a, b, mx, mn = fit_similarity(src, dst, z_flip)
        if best is None or mn < best[3]:
            best = (a, b, z_flip, mn, mx)
    a, b, z_flip, mn, mx = best
    scale = abs(a)
    print(f"\nboard->robot: z_flip={z_flip}  scale={scale:.4f} (expect ~1.0)  "
          f"rot={cmath.phase(a) * 180 / cmath.pi:.2f}deg  anchor resid mean={mn:.2f}mm max={mx:.2f}mm")
    if not 0.9 < scale < 1.1:
        print("  WARNING: scale is far from 1.0 -- check square_mm and the 3 measured robot points.")

    # ---- map every corner to robot mm, fit pixel->robot homography ----
    robots_mm = np.array(apply_similarity(a, b, z_flip, board_mm), dtype=np.float64)
    H, _ = cv2.findHomography(corners_xy.astype(np.float64), robots_mm, method=0)
    if H is None:
        print("Homography fit failed.")
        sys.exit(1)
    H_inv = np.linalg.inv(H)

    pred_mm = cv2.perspectiveTransform(
        corners_xy.reshape(1, -1, 2).astype(np.float64), H).reshape(-1, 2)
    err_mm = float(np.linalg.norm(pred_mm - robots_mm, axis=1).mean())
    pred_px = cv2.perspectiveTransform(
        robots_mm.reshape(1, -1, 2), H_inv).reshape(-1, 2)
    err_px = float(np.linalg.norm(pred_px - corners_xy, axis=1).mean())
    print(f"homography pixel->robot: reprojection error mean {err_mm:.2f}mm / {err_px:.2f}px")

    roi_mm, roi_px = build_roi(cols, rows, square, roi_extend, a, b, z_flip, H_inv)
    save_overlay_image(frame, corners_xy, robots_mm, H_inv, roi_px)
    print(f"Saved overlay verification image: {OVERLAY_IMG}")

    anchors_robot = {k: list(rp[k]) for k in ANCHOR_KEYS}
    if args.no_write:
        print("\n--no-write: calibration file left unchanged.")
        return
    write_calibration(calib_path, frame, H, roi_mm, roi_px, err_mm, err_px, anchors_robot)
    print(f"\nWrote calibration to {calib_path}")


if __name__ == "__main__":
    main()
