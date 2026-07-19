"""Assemble the final YOLO dataset: merge undistorted/ + distorted/ (each
with original + brightness/dark/blur augmented images) into
dataset/yolo/images/{train,val} + labels/{train,val}, and write data.yaml.

Split is done by source photo number (1-24), not by individual file, so all
variants/augmentations of the same photo land in the same split - otherwise
a near-duplicate of a val image (e.g. its blurred twin) could leak into
train and inflate validation metrics.

Usage:
    python tools/build_yolo_dataset.py --val-fraction 0.2 --seed 0
"""
import argparse
import random
import shutil
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
DATASET = REPO / "dataset"
VARIANTS = ["undistorted", "distorted"]


def get_class_names():
    from ultralytics import YOLO
    m = YOLO(str(REPO / "robot" / "best.pt"))
    return [m.names[i] for i in sorted(m.names)]


def photo_number(path):
    # "my_photo-13_bright.jpg" / "my_photo-13.jpg" -> 13
    stem = path.stem
    num_part = stem.split("-")[1].split("_")[0]
    return int(num_part)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--val-fraction", type=float, default=0.2)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    out_root = DATASET / "yolo"
    for split in ("train", "val"):
        (out_root / "images" / split).mkdir(parents=True, exist_ok=True)
        (out_root / "labels" / split).mkdir(parents=True, exist_ok=True)

    photo_numbers = list(range(1, 25))
    rng = random.Random(args.seed)
    rng.shuffle(photo_numbers)
    n_val = max(1, round(len(photo_numbers) * args.val_fraction))
    val_numbers = set(photo_numbers[:n_val])
    train_numbers = set(photo_numbers[n_val:])
    print(f"train photo numbers ({len(train_numbers)}): {sorted(train_numbers)}")
    print(f"val photo numbers ({len(val_numbers)}): {sorted(val_numbers)}")

    counts = {"train": 0, "val": 0}
    for variant in VARIANTS:
        vdir = DATASET / "processed" / variant
        for img_path in sorted(vdir.glob("my_photo-*.jpg")):
            label_path = img_path.with_suffix(".txt")
            if not label_path.exists():
                continue
            n = photo_number(img_path)
            split = "val" if n in val_numbers else "train"
            out_name = f"{variant}_{img_path.stem}"
            shutil.copyfile(img_path, out_root / "images" / split / f"{out_name}.jpg")
            shutil.copyfile(label_path, out_root / "labels" / split / f"{out_name}.txt")
            counts[split] += 1

    names = get_class_names()
    data_yaml = out_root / "data.yaml"
    data_yaml.write_text(
        f"path: {out_root}\n"
        f"train: images/train\n"
        f"val: images/val\n"
        f"names:\n" + "".join(f"  {i}: {n}\n" for i, n in enumerate(names))
    )
    print(f"wrote {counts['train']} train / {counts['val']} val images -> {out_root}")
    print(f"wrote {data_yaml}")


if __name__ == "__main__":
    main()
