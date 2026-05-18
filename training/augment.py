#!/usr/bin/env python3
"""Offline dataset augmentation for YOLO-format images and labels."""

from __future__ import annotations

import argparse
import random
import shutil
from pathlib import Path

import cv2
import numpy as np

Label = tuple[int, float, float, float, float]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Augment YOLO dataset offline")
    parser.add_argument("--input", type=str, required=True, help="Input root (images + labels)")
    parser.add_argument("--output", type=str, required=True, help="Output root")
    parser.add_argument(
        "--factor",
        type=int,
        default=4,
        help="Augmented copies per source image (default 4)",
    )
    parser.add_argument("--seed", type=int, default=42)
    return parser.parse_args()


def resolve_dirs(root: Path) -> tuple[Path, Path]:
    """Resolve image and label directories for common YOLO layouts."""
    nested_images = root / "images"
    nested_labels = root / "labels"
    if nested_images.is_dir() and nested_labels.is_dir():
        return nested_images, nested_labels

    # e.g. datasets/urc_objects/images/train → labels/train
    if root.parent.name == "images":
        split_labels = root.parent.parent / "labels" / root.name
        if split_labels.is_dir():
            return root, split_labels

    return root, root


def list_images(images_dir: Path) -> list[Path]:
    exts = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
    return sorted(p for p in images_dir.iterdir() if p.suffix.lower() in exts)


def label_path_for(image_path: Path, images_dir: Path, labels_dir: Path) -> Path:
    rel = image_path.relative_to(images_dir)
    return labels_dir / rel.with_suffix(".txt")


def load_labels(path: Path) -> list[Label]:
    if not path.is_file():
        return []
    labels: list[Label] = []
    for line in path.read_text().strip().splitlines():
        parts = line.split()
        if len(parts) < 5:
            continue
        cls = int(parts[0])
        xc, yc, w, h = map(float, parts[1:5])
        labels.append((cls, xc, yc, w, h))
    return labels


def save_labels(path: Path, labels: list[Label]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [f"{c} {xc:.6f} {yc:.6f} {w:.6f} {h:.6f}" for c, xc, yc, w, h in labels]
    path.write_text("\n".join(lines) + ("\n" if lines else ""))


def clip_labels(labels: list[Label]) -> list[Label]:
    out: list[Label] = []
    for cls, xc, yc, w, h in labels:
        if w <= 1e-6 or h <= 1e-6:
            continue
        x1 = max(0.0, xc - w / 2)
        y1 = max(0.0, yc - h / 2)
        x2 = min(1.0, xc + w / 2)
        y2 = min(1.0, yc + h / 2)
        if x2 <= x1 or y2 <= y1:
            continue
        out.append((cls, (x1 + x2) / 2, (y1 + y2) / 2, x2 - x1, y2 - y1))
    return out


def random_brightness_contrast(img: np.ndarray) -> np.ndarray:
    alpha = random.uniform(0.65, 1.35)
    beta = random.uniform(-35, 35)
    return cv2.convertScaleAbs(img, alpha=alpha, beta=beta)


def horizontal_flip(img: np.ndarray, labels: list[Label]) -> tuple[np.ndarray, list[Label]]:
    flipped = cv2.flip(img, 1)
    out = [(cls, 1.0 - xc, yc, w, h) for cls, xc, yc, w, h in labels]
    return flipped, out


def random_crop_pad(
    img: np.ndarray, labels: list[Label], min_scale: float = 0.65
) -> tuple[np.ndarray, list[Label]]:
    h, w = img.shape[:2]
    crop_w = max(1, int(w * random.uniform(min_scale, 1.0)))
    crop_h = max(1, int(h * random.uniform(min_scale, 1.0)))
    x0 = random.randint(0, max(0, w - crop_w))
    y0 = random.randint(0, max(0, h - crop_h))
    x1, y1 = x0 + crop_w, y0 + crop_h

    crop = img[y0:y1, x0:x1]
    resized = cv2.resize(crop, (w, h), interpolation=cv2.INTER_LINEAR)

    new_labels: list[Label] = []
    for cls, xc, yc, bw, bh in labels:
        px_c = xc * w
        py_c = yc * h
        px_w = bw * w
        px_h = bh * h
        bx1 = px_c - px_w / 2
        by1 = py_c - px_h / 2
        bx2 = px_c + px_w / 2
        by2 = py_c + px_h / 2

        ix1 = max(bx1, x0)
        iy1 = max(by1, y0)
        ix2 = min(bx2, x1)
        iy2 = min(by2, y1)
        if ix2 <= ix1 or iy2 <= iy1:
            continue

        nbx1 = (ix1 - x0) / crop_w
        nby1 = (iy1 - y0) / crop_h
        nbx2 = (ix2 - x0) / crop_w
        nby2 = (iy2 - y0) / crop_h
        nxc = (nbx1 + nbx2) / 2
        nyc = (nby1 + nby2) / 2
        nw = nbx2 - nbx1
        nh = nby2 - nby1
        new_labels.append((cls, nxc, nyc, nw, nh))

    return resized, clip_labels(new_labels)


def salt_and_pepper(img: np.ndarray, amount: float = 0.004) -> np.ndarray:
    out = img.copy()
    n = int(amount * out.size)
    if n == 0:
        return out
    # salt
    coords = (
        np.random.randint(0, out.shape[0], n // 2),
        np.random.randint(0, out.shape[1], n // 2),
    )
    out[coords] = 255
    # pepper
    coords = (
        np.random.randint(0, out.shape[0], n - n // 2),
        np.random.randint(0, out.shape[1], n - n // 2),
    )
    out[coords] = 0
    return out


def apply_augmentation(img: np.ndarray, labels: list[Label]) -> tuple[np.ndarray, list[Label]]:
    out_img, out_labels = img, list(labels)

    if random.random() < 0.85:
        out_img = random_brightness_contrast(out_img)

    if random.random() < 0.5:
        out_img, out_labels = horizontal_flip(out_img, out_labels)

    if random.random() < 0.6:
        out_img, out_labels = random_crop_pad(out_img, out_labels)

    if random.random() < 0.4:
        out_img = salt_and_pepper(out_img)

    return out_img, clip_labels(out_labels)


def copy_original(
    image_path: Path,
    label_path: Path,
    out_images: Path,
    out_labels: Path,
    stem: str,
) -> None:
    out_img = out_images / f"{stem}{image_path.suffix.lower()}"
    out_lbl = out_labels / f"{stem}.txt"
    shutil.copy2(image_path, out_img)
    if label_path.is_file():
        shutil.copy2(label_path, out_lbl)
    else:
        out_lbl.write_text("")


def main() -> None:
    args = parse_args()
    random.seed(args.seed)
    np.random.seed(args.seed)

    in_root = Path(args.input)
    out_root = Path(args.output)
    in_images, in_labels = resolve_dirs(in_root)
    out_images = out_root / "images"
    out_labels = out_root / "labels"
    out_images.mkdir(parents=True, exist_ok=True)
    out_labels.mkdir(parents=True, exist_ok=True)

    images = list_images(in_images)
    if not images:
        raise SystemExit(f"No images found in {in_images}")

    total = 0
    for image_path in images:
        lbl_path = label_path_for(image_path, in_images, in_labels)
        img = cv2.imread(str(image_path))
        if img is None:
            print(f"Skip unreadable: {image_path}")
            continue
        labels = load_labels(lbl_path)
        stem_base = image_path.stem

        copy_original(image_path, lbl_path, out_images, out_labels, stem_base)
        total += 1

        for i in range(args.factor):
            aug_img, aug_labels = apply_augmentation(img, labels)
            if not aug_labels and labels:
                continue
            out_stem = f"{stem_base}_aug{i:03d}"
            out_img_path = out_images / f"{out_stem}{image_path.suffix.lower()}"
            out_lbl_path = out_labels / f"{out_stem}.txt"
            cv2.imwrite(str(out_img_path), aug_img)
            save_labels(out_lbl_path, aug_labels)
            total += 1

    print(f"Wrote {total} images to {out_images}")
    print(f"Labels in {out_labels}")


if __name__ == "__main__":
    main()
