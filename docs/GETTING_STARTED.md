# Getting started (new teammates)

Step-by-step setup on a **development laptop**. On-robot setup is similar but uses the Jetson’s Ubuntu + ROS install instead of Docker.

**Time:** ~30–60 minutes first time (mostly downloads).

---

## What you need

| Item | Notes |
|------|-------|
| **Git** | Clone the repo |
| **Docker Desktop** (Mac/Windows) or Docker on Linux | Easiest path for ROS 2 Humble |
| **8+ GB free disk** | Images + build artifacts |
| **Optional:** Ubuntu 22.04 natively | Skip Docker if you already run ROS 2 Humble |

You do **not** need a RealSense plugged into your laptop for a first build — only for camera demos.

---

## 1. Clone the repository

```bash
git clone https://github.com/thatrandomasiandev/urc-cv.git
cd urc-cv
```

---

## 2. Build with Docker (recommended)

```bash
./scripts/build_dev.sh
```

This compiles these packages inside a container:

- `urc_autonomy`, `aruco_detector`, `visual_servo`, `search_behavior`, `urc_localization`

`object_detector` and `object_approach` build when you add them to the build line or run a full workspace build.

Open an interactive shell:

```bash
docker compose run --rm dev
```

Inside the container:

```bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
```

---

## 3. Run the full stack (with hardware)

On the rover (or a bench setup with camera + GPS):

```bash
# From repo root on machine that has ROS + hardware
./scripts/run_stack.sh
# OR inside Docker with USB passthrough (see docker-compose.yml)
docker compose up autonomy
```

Expect many log lines — that is normal.

---

## 4. Verify things are alive

In a **second terminal** (same ROS environment):

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash   # or /ros2_ws/install inside Docker

./scripts/status.sh
```

You should see:

- A list of **nodes** (`mission_node`, `detector_node`, …)
- Current **`/mission/state`** (often `IDLE` before waypoints)
- Optional ArUco / e-stop lines if sensors are publishing

---

## 5. Load example waypoints

```bash
# From a JSON file
./scripts/load_waypoints.sh waypoints/example_waypoints.json

# Or inline lat,lon (one or more)
./scripts/load_waypoints.sh 38.4068,-110.7916
```

Watch state change:

```bash
ros2 topic echo /mission/state
```

You should see `NAV_TO_WP` (if GPS and Nav2 are available).

---

## 6. Useful commands cheat sheet

```bash
# List nodes
ros2 node list

# List topics
ros2 topic list

# Mission state (live)
ros2 topic echo /mission/state

# Last marker detection (one message)
ros2 topic echo /aruco/detection --once

# E-stop status
ros2 topic echo /estop/status --once

# Trigger manual e-stop
ros2 topic pub --once /estop/trigger std_msgs/msg/Bool "{data: true}"

# Clear manual e-stop
ros2 topic pub --once /estop/trigger std_msgs/msg/Bool "{data: false}"

# View debug camera overlay (needs display)
ros2 run rqt_image_view rqt_image_view /aruco/debug_image
```

---

## 7. Run tests (no hardware)

From `ros2_ws` after build:

```bash
colcon test --packages-select urc_autonomy --event-handlers console_direct+
colcon test-result --verbose
```

These tests fake sensor messages and check mission / e-stop / servo logic.

---

## 8. Simulation (optional)

Requires Gazebo + sim packages installed:

```bash
ros2 launch urc_simulation sim.launch.py
# second terminal:
ros2 launch urc_autonomy autonomy.launch.py \
  use_sim_time:=true launch_realsense:=false object_detector_device:=cpu
```

---

## 9. Python training environment (separate from ROS)

For labeling / YOLO only:

```bash
pip install -r requirements.txt
cd training
python train.py
```

See [../training/README.md](../training/README.md).

---

## Troubleshooting first setup

| Problem | Try |
|---------|-----|
| `ros2: command not found` | `source /opt/ros/humble/setup.bash` |
| Nodes not found | `source install/setup.bash` after `colcon build` |
| Docker USB / camera | Check `devices:` in `docker-compose.yml`; run on host if needed |
| Nav2 never moves rover | GPS `/fix`? Nav2 lifecycle active? Check `ros2 action list` |
| No ArUco messages | Camera topics? `ros2 topic hz /camera/color/image_raw` |
| Build missing package | Add package to `colcon build --packages-select ...` |

---

## Read next

1. [TEAM_GUIDE.md](TEAM_GUIDE.md) — conceptual overview  
2. [OPERATIONS.md](OPERATIONS.md) — field test day procedures  
3. [ARCHITECTURE.md](ARCHITECTURE.md) — when you need implementation detail
