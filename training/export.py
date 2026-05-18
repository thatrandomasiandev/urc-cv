#!/usr/bin/env python3
"""Export trained YOLOv8 weights to TensorRT engine for Jetson Orin Nano."""

from __future__ import annotations

import argparse
from pathlib import Path

from ultralytics import YOLO


def main() -> None:
    parser = argparse.ArgumentParser(description="Export YOLOv8 to TensorRT engine")
    parser.add_argument("--model", type=str, required=True, help="Path to best.pt")
    parser.add_argument("--imgsz", type=int, default=640)
    args = parser.parse_args()

    model = YOLO(args.model)
    out = model.export(
        format="engine",
        imgsz=args.imgsz,
        device=0,
        half=True,
        workspace=4,
    )
    engine_path = Path(out) if out else Path(args.model).with_suffix(".engine")
    print(f"TensorRT engine: {engine_path.resolve()}")


if __name__ == "__main__":
    main()
