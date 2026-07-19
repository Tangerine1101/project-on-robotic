"""Camera<->robot calibration from a single static photo, instead of the
live-camera click-and-type flow in robot/camera_calibrate.py.

Uses dataset/calib.jpg (1280x720, same resolution as the current live
`vision:` config -- no undistort/crop applied, matching what vision.py
actually sees today) + the known robot-frame positions of the 4 reference
objects in it, documented in dataset/calib.md:

    lemon:  (260, -50)  and (305, -165)     # two lemons, ambiguous by class alone
    onion:  (305, -50)
    garlic: (305, -265)

(Z ("sag") from calib.md is ignored -- vision.py always emits z=0.0.)

robot/best.pt detects exactly one Garlic and one Onion box (unambiguous) but
two Lemon boxes, so we don't know a priori which detected lemon pixel is
which calib.md lemon. Resolved by brute-forcing both lemon<->coordinate
assignments times both z_flip values (4 combinations), fitting the same 2D
similarity transform robot/camera_calibrate.py uses, and keeping whichever
combination has the lowest residual -- a wrong assignment or wrong handedness
cannot be fit well by a pure similarity transform, so the correct combination
stands out clearly (see plan notes: ~19mm vs 67-107mm in a dry run).

Usage:
    python tools/calibrate_from_photo.py                # compute and write
    python tools/calibrate_from_photo.py --no-write      # compute and print only
"""
import argparse
import itertools
import sys
from pathlib import Path

import cv2

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "robot"))
from camera_calibrate import CONFIG_PATH, decompose, fit_transform, format_values, update_vision_config  # noqa: E402

CALIB_JPG = REPO / "dataset" / "calib.jpg"
BEST_PT = REPO / "robot" / "best.pt"
OUT_JPG = REPO / "dataset" / "calib" / "calib_correspondences.jpg"

# Robot-frame XY (mm), from dataset/calib.md; Z ("sag") dropped, see docstring.
GARLIC_ROBOT = (305, -265)
ONION_ROBOT = (305, -50)
LEMON_ROBOT_CANDIDATES = [(260, -50), (305, -165)]


def detect_objects(conf=0.25):
    from ultralytics import YOLO

    model = YOLO(str(BEST_PT))
    img = cv2.imread(str(CALIB_JPG))
    if img is None:
        raise RuntimeError(f"could not read {CALIB_JPG}")
    r = model(img, conf=conf, verbose=False)[0]

    by_class = {"Garlic": [], "Lemon": [], "Onion": []}
    for box in r.boxes:
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        name = model.names[int(box.cls[0])]
        by_class.setdefault(name, []).append(((x1 + x2) / 2, (y1 + y2) / 2))

    counts = {k: len(v) for k, v in by_class.items()}
    if counts != {"Garlic": 1, "Lemon": 2, "Onion": 1}:
        raise RuntimeError(
            f"expected exactly 1 Garlic, 2 Lemon, 1 Onion in {CALIB_JPG}, got {counts} "
            "-- inspect the detections manually before trusting this script's correspondence logic"
        )
    return img, by_class["Garlic"][0], by_class["Onion"][0], by_class["Lemon"]


def resolve_lemon_assignment(garlic_px, onion_px, lemon_px):
    """Try both lemon<->coordinate pairings x both z_flip values; return the
    (pixels, robots, z_flip, a, b, max_err, mean_err) combo with lowest mean_err,
    plus a report of all 4 combos for transparency."""
    fixed_px = [garlic_px, onion_px]
    fixed_robot = [GARLIC_ROBOT, ONION_ROBOT]

    results = []
    for lemon_perm in itertools.permutations(LEMON_ROBOT_CANDIDATES):
        pixels = fixed_px + list(lemon_px)
        robots = fixed_robot + list(lemon_perm)
        for z_flip in (False, True):
            a, b, max_err, mean_err = fit_transform(pixels, robots, z_flip)
            results.append((pixels, robots, z_flip, a, b, max_err, mean_err, lemon_perm))

    results.sort(key=lambda t: t[6])
    return results


def draw_correspondences(img, pixels, robots):
    out = img.copy()
    for (u, v), (x, y) in zip(pixels, robots):
        u, v = int(round(u)), int(round(v))
        cv2.drawMarker(out, (u, v), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
        cv2.putText(out, f"({x:.0f},{y:.0f})mm", (u + 10, v - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)
    OUT_JPG.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(OUT_JPG), out)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--no-write", action="store_true", help="compute and print only; don't touch config.yaml")
    ap.add_argument("--conf", type=float, default=0.25)
    args = ap.parse_args()

    img, garlic_px, onion_px, lemon_px = detect_objects(conf=args.conf)
    print(f"detected: garlic={garlic_px}  onion={onion_px}  lemon={lemon_px}")

    results = resolve_lemon_assignment(garlic_px, onion_px, lemon_px)
    print("\n=== all 4 (lemon assignment x z_flip) combinations ===")
    for pixels, robots, z_flip, a, b, max_err, mean_err, lemon_perm in results:
        params = decompose(a, b)
        print(f"  lemon->{lemon_perm}  z_flip={z_flip!s:5s}  "
              f"rotate={params['rotate']:7.2f}deg  px_per_mm={params['pixels_per_mm']:.4f}  "
              f"resid max={max_err:6.2f}mm mean={mean_err:6.2f}mm")

    pixels, robots, z_flip, a, b, max_err, mean_err, lemon_perm = results[0]
    print(f"\n=== winning combination: lemon->{lemon_perm}, z_flip={z_flip} "
          f"(mean resid {mean_err:.2f}mm, next best {results[1][6]:.2f}mm) ===")
    print("final pixel <-> robot correspondences:")
    for (u, v), (x, y) in zip(pixels, robots):
        print(f"  pixel ({u:7.1f}, {v:7.1f})  ->  robot ({x:6.1f}, {y:6.1f}) mm")

    draw_correspondences(img, pixels, robots)
    print(f"\nwrote verification image: {OUT_JPG}")

    params = decompose(a, b)
    print("\n=== Calibration result ===")
    print(f"  rotate        : {params['rotate']:.3f} deg")
    print(f"  z_flip        : {z_flip}")
    print(f"  pixels_per_mm : {params['pixels_per_mm']:.4f}")
    print(f"  x_0_mm        : {params['x_0_mm']:.3f}")
    print(f"  y_0_mm        : {params['y_0_mm']:.3f}")
    print(f"  fit residual  : max {max_err:.2f} mm, mean {mean_err:.2f} mm")
    print("  (residual is higher than a live 3-click calibration would give: calib.md's own")
    print("   hand-measurements carry 5-15mm tolerance, YOLO bbox centers aren't exact object")
    print("   centers for irregular objects like garlic, calib.jpg isn't undistorted yet, and")
    print("   only 4 points are available.)")

    values = format_values(params, z_flip)
    print("\nvision: block update:")
    for k, v in values.items():
        print(f"  {k}: {v}")

    if args.no_write:
        print("\n--no-write: config.yaml left unchanged.")
        return
    CONFIG_PATH.write_text(update_vision_config(CONFIG_PATH.read_text(), values))
    print(f"\nWrote calibration to {CONFIG_PATH}")


if __name__ == "__main__":
    main()
