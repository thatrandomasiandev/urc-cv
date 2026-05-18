# URC Object Detection — YOLOv8 Training

Self-contained pipeline for training, validating, and exporting a YOLOv8 detector for **mallet**, **rock_pick**, and **water_bottle** on desert field terrain.

## 1. Data collection

Capture photos outdoors on rocky/desert terrain similar to competition sites:

- **Distance:** 5–20 m from the rover (objects appear at multiple scales)
- **Angles:** front, side, oblique, partial occlusion behind rocks
- **Lighting:** bright sun, shadow, overcast, golden hour
- **Background:** varied rock/soil; include hard negatives (rocks without tools)
- **Objects:** all three classes in separate and mixed frames

Organize raw captures before labeling; split roughly 70% train / 20% val / 10% test.

## 2. Labeling

Use [Roboflow](https://roboflow.com) (or similar) and export in **YOLOv8** format into:

```text
datasets/urc_objects/
  images/train/
  images/val/
  images/test/
  labels/train/
  labels/val/
  labels/test/
```

Class IDs must match `data.yaml`: `0=mallet`, `1=rock_pick`, `2=water_bottle`.

## 3. Offline augmentation (optional, small datasets)

Expand the training set before `train.py`:

```bash
cd training
pip install opencv-python numpy
python augment.py \
  --input ../datasets/urc_objects/images/train \
  --output ../datasets/urc_objects/aug_train \
  --factor 4
```

Merge augmented `images/` and `labels/` into your train split (or point `train:` in `data.yaml` at a combined folder).

## 4. Training

```bash
cd training
pip install ultralytics torch
python train.py
```

Defaults: `yolov8n.pt`, 150 epochs, batch 16, device auto (`mps` → `cuda` → `cpu`). Outputs:

```text
models/urc_objects/weights/best.pt
```

Override as needed:

```bash
python train.py --epochs 200 --batch 8 --device cuda --name urc_v2
```

## 5. Validation

```bash
python validate.py --model ../models/urc_objects/weights/best.pt
```

Prints overall and per-class mAP50, mAP50-95, precision, and recall.

## 6. Export for Jetson Orin Nano

Run on the Jetson (or a machine with TensorRT + matching CUDA):

```bash
python export.py --model ../models/urc_objects/weights/best.pt
```

Produces an FP16 `.engine` file for onboard inference.

## 7. Deploy on the rover

1. Copy the `.engine` file into `ros2_ws/` (or your runtime assets path).
2. Set the `model_path` parameter in your detector launch file to that engine.
3. Rebuild/relaunch the ROS 2 stack and verify detections on a live camera feed.

## File reference

| File          | Purpose                                      |
|---------------|----------------------------------------------|
| `data.yaml`   | Dataset paths and class names                |
| `train.py`    | Train with desert-tuned augmentations        |
| `validate.py` | Val-split metrics                            |
| `export.py`   | TensorRT engine for Jetson                   |
| `augment.py`  | Offline OpenCV augmentation                  |
