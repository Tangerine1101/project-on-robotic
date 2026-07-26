#!/usr/bin/env python3
"""Build a YOLO detection dataset from labelme/X-AnyLabeling JSON labels.

Reads a folder of frames where only some frames carry a `<name>.json` label
(unlabeled frames are intentionally skipped), converts the rectangle shapes to
YOLO txt, splits train/val, and thickens the *train* split with photometric
augmentations (brighter / darker / light blur) whose boxes are unchanged.

Class ids follow best.pt's order (0=Garlic, 1=Lemon, 2=Onion) so a finetune
transfers cleanly.

Usage:
    python tools/build_finetune_dataset.py \
        --src dataset/finetune_2/my_video-1 \
        --out dataset/finetune_2/yolo
"""
import argparse
import json
import random
import shutil
from pathlib import Path

import cv2
import numpy as np

# Match best.pt: YOLO('best.pt').names == {0:'Garlic',1:'Lemon',2:'Onion'}
CLASS_ID = {"garlic": 0, "lemon": 1, "onion": 2}
CLASS_NAMES = {0: "Garlic", 1: "Lemon", 2: "Onion"}


def json_to_yolo_lines(jpath):
    """Convert one labelme json to YOLO label lines. Returns (lines, W, H)."""
    d = json.loads(Path(jpath).read_text())
    W, H = d["imageWidth"], d["imageHeight"]
    lines = []
    for s in d.get("shapes", []):
        cls = CLASS_ID.get(s["label"].strip().lower())
        if cls is None:
            print(f"  ! unknown label {s['label']!r} in {jpath.name} -- skipped")
            continue
        pts = np.array(s["points"], dtype=float)  # 2 or 4 corners
        xmin, ymin = pts[:, 0].min(), pts[:, 1].min()
        xmax, ymax = pts[:, 0].max(), pts[:, 1].max()
        cx = (xmin + xmax) / 2 / W
        cy = (ymin + ymax) / 2 / H
        w = (xmax - xmin) / W
        h = (ymax - ymin) / H
        # clamp to the image
        cx, cy = min(max(cx, 0), 1), min(max(cy, 0), 1)
        w, h = min(max(w, 0), 1), min(max(h, 0), 1)
        lines.append(f"{cls} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}")
    return lines, W, H


# Photometric augmentations: pixels change, boxes do not. Keyed by filename suffix.
def _bright(img, gain):
    return cv2.convertScaleAbs(img, alpha=gain, beta=0)


AUGS = {
    "bright": lambda im: _bright(im, 1.30),   # +30% brighter
    "dark":   lambda im: _bright(im, 0.70),   # -30% darker
    "blur":   lambda im: cv2.GaussianBlur(im, (5, 5), 0),  # light blur
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True, help="folder of frames + labelme json")
    ap.add_argument("--out", required=True, help="output YOLO dataset folder")
    ap.add_argument("--val-frac", type=float, default=0.2)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--no-aug", action="store_true", help="skip train augmentation")
    args = ap.parse_args()

    src = Path(args.src).resolve()
    out = Path(args.out).resolve()
    jsons = sorted(src.glob("*.json"))
    if not jsons:
        raise SystemExit(f"no .json labels found in {src}")

    # pair each json with its jpg (unlabeled frames are skipped by design)
    pairs = []
    for j in jsons:
        img = j.with_suffix(".jpg")
        if not img.exists():
            print(f"  ! no jpg for {j.name} -- skipped")
            continue
        pairs.append((img, j))

    random.seed(args.seed)
    random.shuffle(pairs)
    n_val = max(1, round(len(pairs) * args.val_frac))
    val_pairs, train_pairs = pairs[:n_val], pairs[n_val:]

    for sub in ("images/train", "images/val", "labels/train", "labels/val"):
        d = out / sub
        if d.exists():
            shutil.rmtree(d)
        d.mkdir(parents=True, exist_ok=True)

    def emit(img_path, lines, split, stem):
        (out / f"labels/{split}/{stem}.txt").write_text("\n".join(lines) + ("\n" if lines else ""))

    # --- val: originals only, no augmentation ---
    for img, j in val_pairs:
        lines, _, _ = json_to_yolo_lines(j)
        shutil.copy(img, out / f"images/val/{img.stem}.jpg")
        emit(img, lines, "val", img.stem)

    # --- train: originals + photometric augmentations ---
    n_train_imgs = 0
    for img, j in train_pairs:
        lines, _, _ = json_to_yolo_lines(j)
        shutil.copy(img, out / f"images/train/{img.stem}.jpg")
        emit(img, lines, "train", img.stem)
        n_train_imgs += 1
        if args.no_aug:
            continue
        bgr = cv2.imread(str(img))
        for suffix, fn in AUGS.items():
            stem = f"{img.stem}_{suffix}"
            cv2.imwrite(str(out / f"images/train/{stem}.jpg"), fn(bgr))
            emit(img, lines, "train", stem)   # same boxes
            n_train_imgs += 1

    data_yaml = out / "data.yaml"
    names_block = "\n".join(f"  {i}: {n}" for i, n in CLASS_NAMES.items())
    data_yaml.write_text(
        f"path: {out}\n"
        f"train: images/train\n"
        f"val: images/val\n"
        f"names:\n{names_block}\n"
    )

    print(f"labeled frames used : {len(pairs)}  (train {len(train_pairs)} / val {len(val_pairs)})")
    print(f"train images written: {n_train_imgs}  (orig + {0 if args.no_aug else len(AUGS)}x aug)")
    print(f"val   images written: {len(val_pairs)}  (originals only)")
    print(f"data.yaml           : {data_yaml}")


if __name__ == "__main__":
    main()
