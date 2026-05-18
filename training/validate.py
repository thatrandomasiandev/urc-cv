#!/usr/bin/env python3
"""Validate a trained YOLOv8 model on the val split."""

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


def load_class_names(data_yaml: str) -> list[str]:
    from ultralytics.utils import yaml_load

    cfg = yaml_load(data_yaml)
    names = cfg.get("names", [])
    if isinstance(names, dict):
        return [names[i] for i in sorted(names)]
    return list(names)


def main() -> None:
    parser = argparse.ArgumentParser(description="Validate YOLOv8 weights")
    parser.add_argument("--model", type=str, required=True, help="Path to .pt weights")
    parser.add_argument("--data", type=str, default="data.yaml", help="Dataset YAML")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--device", type=str, default=None, help="cuda | mps | cpu")
    args = parser.parse_args()

    device = args.device or default_device()
    names = load_class_names(args.data)

    model = YOLO(args.model)
    metrics = model.val(data=args.data, imgsz=args.imgsz, device=device, split="val")

    box = metrics.box
    print("\n=== Overall ===")
    print(f"mAP50:     {box.map50:.4f}")
    print(f"mAP50-95:  {box.map:.4f}")
    print(f"Precision: {box.mp:.4f}")
    print(f"Recall:    {box.mr:.4f}")

    ap50 = getattr(box, "ap50", None)
    maps = getattr(box, "maps", None)
    p_per_class = getattr(box, "p", None)
    r_per_class = getattr(box, "r", None)

    print("\n=== Per class ===")
    for i, name in enumerate(names):
        map50 = ap50[i] if ap50 is not None and i < len(ap50) else float("nan")
        map5095 = maps[i] if maps is not None and i < len(maps) else float("nan")
        prec = p_per_class[i] if p_per_class is not None and i < len(p_per_class) else float("nan")
        rec = r_per_class[i] if r_per_class is not None and i < len(r_per_class) else float("nan")
        print(
            f"{name:16s}  "
            f"mAP50={map50:.4f}  mAP50-95={map5095:.4f}  "
            f"P={prec:.4f}  R={rec:.4f}"
        )


if __name__ == "__main__":
    main()
