#!/usr/bin/env python3
"""Train YOLOv8 on URC object detection dataset."""

from __future__ import annotations

import argparse

import torch
from ultralytics import YOLO


def default_device() -> str:
    if torch.backends.mps.is_available():
        return "mps"
    if torch.cuda.is_available():
        return "cuda"
    return "cpu"


def main() -> None:
    parser = argparse.ArgumentParser(description="Train YOLOv8 for URC object detection")
    parser.add_argument("--model", type=str, default="yolov8n.pt", help="Base model weights")
    parser.add_argument("--epochs", type=int, default=150)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--batch", type=int, default=16, help="Batch size (M4 MPS / Jetson)")
    parser.add_argument("--data", type=str, default="data.yaml", help="Dataset YAML")
    parser.add_argument("--device", type=str, default=None, help="cuda | mps | cpu")
    parser.add_argument("--patience", type=int, default=30, help="Early stopping patience")
    parser.add_argument("--project", type=str, default="../models", help="Training output root")
    parser.add_argument("--name", type=str, default="urc_objects", help="Run name")
    args = parser.parse_args()

    device = args.device or default_device()
    print(f"Using device: {device}")

    model = YOLO(args.model)
    results = model.train(
        data=args.data,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=device,
        patience=args.patience,
        project=args.project,
        name=args.name,
        # Augmentation tuned for outdoor rocky desert terrain
        hsv_h=0.02,
        hsv_s=0.5,
        hsv_v=0.4,
        degrees=15.0,
        translate=0.1,
        scale=0.5,
        flipud=0.0,
        fliplr=0.5,
        mosaic=1.0,
        mixup=0.1,
        copy_paste=0.1,
    )
    print(f"Best model: {results.save_dir}/weights/best.pt")


if __name__ == "__main__":
    main()
