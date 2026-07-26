"""Convert labelme-format rectangle JSON (dataset/processed/<variant>/*.json)
into YOLO txt labels, using robot/best.pt's own class order so finetuning
stays id-compatible with the deployed model. Also renders annotated preview
images into dataset/label/ for review.

Usage:
    python tools/labelme_to_yolo.py --variants undistorted distorted
"""
import argparse
import json
from pathlib import Path

import cv2

REPO = Path(__file__).resolve().parent.parent
DATASET = REPO / "dataset"

COLORS = {"garlic": (0, 200, 255), "lemon": (0, 220, 0), "onion": (180, 0, 220)}


def get_class_map():
    from ultralytics import YOLO
    m = YOLO(str(REPO / "robot" / "best.pt"))
    return {name.lower(): idx for idx, name in m.names.items()}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--variants", nargs="+", default=["undistorted", "distorted"])
    ap.add_argument("--preview-dir", default=str(DATASET / "label"))
    args = ap.parse_args()

    class_map = get_class_map()
    print("class map:", class_map)

    preview_dir = Path(args.preview_dir)
    preview_dir.mkdir(parents=True, exist_ok=True)

    for variant in args.variants:
        vdir = DATASET / "processed" / variant
        json_files = sorted(vdir.glob("my_photo-*.json"),
                             key=lambda p: int(p.stem.split("-")[1]))
        unknown_labels = set()
        for jpath in json_files:
            d = json.loads(jpath.read_text())
            img_path = vdir / (jpath.stem + ".jpg")
            img = cv2.imread(str(img_path))
            h, w = img.shape[:2]

            lines = []
            preview = img.copy()
            for shape in d["shapes"]:
                label = shape["label"].lower()
                if label not in class_map:
                    unknown_labels.add((jpath.name, label))
                    continue
                pts = shape["points"]
                xs = [p[0] for p in pts]
                ys = [p[1] for p in pts]
                x1, x2 = min(xs), max(xs)
                y1, y2 = min(ys), max(ys)
                cls = class_map[label]
                cx, cy = (x1 + x2) / 2 / w, (y1 + y2) / 2 / h
                bw, bh = (x2 - x1) / w, (y2 - y1) / h
                lines.append(f"{cls} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}")

                color = COLORS.get(label, (255, 255, 255))
                cv2.rectangle(preview, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
                cv2.putText(preview, label, (int(x1), max(int(y1) - 5, 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

            (vdir / (jpath.stem + ".txt")).write_text("\n".join(lines) + "\n")
            cv2.imwrite(str(preview_dir / f"{variant}_{jpath.stem}.jpg"), preview)

        if unknown_labels:
            print(f"WARNING unknown labels in {variant}: {sorted(unknown_labels)}")
        print(f"{variant}: wrote {len(json_files)} YOLO label files + previews")


if __name__ == "__main__":
    main()
