# Models

Deploy weights here for the onboard detector:

| File | Description |
|------|-------------|
| `urc_objects.pt` | PyTorch weights (default in `autonomy.launch.py`) |
| `urc_objects.engine` | TensorRT engine for Jetson (`training/export.py`) |

Training output also lands under `urc_objects/weights/` when using `training/train.py`.
