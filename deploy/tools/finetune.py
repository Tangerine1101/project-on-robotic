"""Finetune robot/best.pt (YOLO11s) on dataset/yolo at 640x480, then evaluate
both the new and original weights on the same held-out val split so results
are directly comparable.

Usage:
    python tools/finetune.py --epochs 80
"""
import argparse
from pathlib import Path

from ultralytics import YOLO

REPO = Path(__file__).resolve().parent.parent
DATA_YAML = REPO / "dataset" / "yolo" / "data.yaml"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--epochs", type=int, default=80)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--batch", type=int, default=8)
    ap.add_argument("--device", default="0")
    ap.add_argument("--patience", type=int, default=20)
    args = ap.parse_args()

    print("=== finetuning from robot/best.pt ===")
    model = YOLO(str(REPO / "robot" / "best.pt"))
    model.train(
        data=str(DATA_YAML),
        imgsz=args.imgsz,
        epochs=args.epochs,
        batch=args.batch,
        device=args.device,
        patience=args.patience,
        project=str(REPO / "dataset" / "yolo" / "runs"),
        name="finetune",
        exist_ok=True,
    )

    best_weights = REPO / "dataset" / "yolo" / "runs" / "finetune" / "weights" / "best.pt"
    print(f"\n=== evaluating NEW weights ({best_weights}) on val split ===")
    new_model = YOLO(str(best_weights))
    new_metrics = new_model.val(data=str(DATA_YAML), imgsz=args.imgsz, device=args.device)

    print("\n=== evaluating ORIGINAL robot/best.pt on the SAME val split ===")
    old_model = YOLO(str(REPO / "robot" / "best.pt"))
    old_metrics = old_model.val(data=str(DATA_YAML), imgsz=args.imgsz, device=args.device)

    def report(name, m):
        print(f"{name}: mAP50={m.box.map50:.4f}  mAP50-95={m.box.map:.4f}  "
              f"precision={m.box.mp:.4f}  recall={m.box.mr:.4f}")

    print("\n=== SUMMARY (same val split) ===")
    report("NEW (finetuned)", new_metrics)
    report("OLD (robot/best.pt)", old_metrics)
    print(f"\nnew weights saved at: {best_weights}")
    print("NOT copied over robot/best.pt - review metrics above before swapping it in.")


if __name__ == "__main__":
    main()
