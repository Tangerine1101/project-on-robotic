"""Convert manually-estimated pixel bboxes (JSON) into YOLO txt labels, and
render annotated preview images for owner review.

Class ids are read from robot/best.pt's own model.names so finetuning stays
compatible with the existing model (0=Garlic, 1=Lemon, 2=Onion currently).

Usage:
    python tools/write_yolo_label.py dataset/manual_labels_gate.json \
        --variants undistorted distorted --preview-dir dataset/label
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
    ap.add_argument("labels_json")
    ap.add_argument("--variants", nargs="+", default=["undistorted", "distorted"])
    ap.add_argument("--preview-dir", default=str(DATASET / "label"))
    args = ap.parse_args()

    class_map = get_class_map()
    print("class map:", class_map)

    labels = json.loads(Path(args.labels_json).read_text())
    labels.pop("_note", None)

    preview_dir = Path(args.preview_dir)
    preview_dir.mkdir(parents=True, exist_ok=True)

    for variant in args.variants:
        img_dir = DATASET / "processed" / variant
        label_dir = DATASET / "processed" / variant  # yolo txt sits beside the image
        for name, boxes in labels.items():
            img_path = img_dir / name
            if not img_path.exists():
                print(f"skip missing {img_path}")
                continue
            img = cv2.imread(str(img_path))
            h, w = img.shape[:2]

            lines = []
            preview = img.copy()
            for b in boxes:
                label = b["label"].lower()
                x1, y1, x2, y2 = b["box"]
                cls = class_map[label]
                cx = (x1 + x2) / 2 / w
                cy = (y1 + y2) / 2 / h
                bw = (x2 - x1) / w
                bh = (y2 - y1) / h
                lines.append(f"{cls} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}")

                color = COLORS.get(label, (255, 255, 255))
                cv2.rectangle(preview, (x1, y1), (x2, y2), color, 2)
                cv2.putText(preview, label, (x1, max(y1 - 5, 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

            txt_path = (label_dir / name).with_suffix(".txt")
            txt_path.write_text("\n".join(lines) + "\n")

            preview_path = preview_dir / f"{variant}_{name}"
            cv2.imwrite(str(preview_path), preview)

        print(f"{variant}: wrote {len(labels)} label files + previews")


if __name__ == "__main__":
    main()
