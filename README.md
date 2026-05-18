# URC CV — University Rover Challenge 2026 Autonomy

ROS 2 (Humble) software for autonomous navigation, ArUco marker acquisition, visual servoing, object detection, and safety monitoring for a URC 2026 competition rover.

## Overview

The stack runs on a Jetson Orin Nano (or in Docker on a dev machine) and wires together perception, planning, and low-level motion through a single launch file.

```
GPS waypoints → Nav2          → search (arc)     → visual servo → SUCCESS
                     ↑              ↑                    ↑
              mission_node    search_node          servo_node
                     ↑              ↑                    ↑
              ArUco detector ──────┴────────────────────┘
              YOLO object detector (parallel task)
              E-stop watchdog (heartbeat, proximity, manual)
```

| Package | Role |
|---------|------|
| `urc_autonomy` | Mission state machine, e-stop, telemetry, Nav2 integration |
| `aruco_detector` | ArUco detection on `/aruco/detection` |
| `visual_servo` | Bearing/distance PID → `/cmd_vel_servo` |
| `search_behavior` | Arc search when Nav2 reaches a waypoint without a marker |
| `urc_localization` | Visual odometry + GPS → EKF |
| `object_detector` | YOLOv8 onboard inference |
| `object_approach` | Approach behavior for science objects |
| `urc_simulation` | Gazebo field + sim launch |

## Requirements

- **On-robot / Linux:** Ubuntu 22.04, ROS 2 Humble, Intel RealSense, Nav2, `robot_localization`, `twist_mux`
- **Dev (optional):** Docker + Docker Compose (see below)
- **Training:** Python 3.10+, Ultralytics — see [`training/README.md`](training/README.md)

## Quick start (Docker)

```bash
# Build image and compile packages
./scripts/build_dev.sh

# Interactive dev shell (mounts full workspace)
docker compose run --rm dev

# Inside container:
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
ros2 launch urc_autonomy autonomy.launch.py
```

Or run the autonomy service directly:

```bash
./scripts/run_stack.sh
```

## Native build (ROS 2 workspace)

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch urc_autonomy autonomy.launch.py
```

Simulation (Gazebo):

```bash
ros2 launch urc_simulation sim.launch.py
# In another terminal, launch autonomy with sim time:
ros2 launch urc_autonomy autonomy.launch.py use_sim_time:=true launch_realsense:=false object_detector_device:=cpu
```

## Mission flow

`mission_node` publishes state on `/mission/state` and commands on `/mission/cmd`.

| State | Description |
|-------|-------------|
| `IDLE` | Waiting for waypoints |
| `NAV_TO_WP` | Nav2 driving to current GPS waypoint |
| `SEARCHING` | Arc search for ArUco (`START_SEARCH`) |
| `VISUAL_SERVO` | Closed-loop approach to marker (`START_SERVO`) |
| `SUCCESS` | Stop distance reached; advance to next waypoint |
| `FAILED` | Timeout or Nav2 failure |

Load waypoints (JSON list of `lat` / `lon`):

```bash
./scripts/load_waypoints.sh waypoints.json
```

## Key topics

| Topic | Type | Description |
|-------|------|-------------|
| `/mission/waypoints` | `std_msgs/String` (JSON) | Waypoint list |
| `/mission/state` | `std_msgs/String` | Current mission state |
| `/mission/cmd` | `std_msgs/String` (JSON) | `START_SERVO`, `STOP_SERVO`, etc. |
| `/aruco/detection` | `std_msgs/String` (JSON) | Marker bearing, distance, confidence |
| `/cmd_vel_servo` | `geometry_msgs/Twist` | Visual servo output |
| `/e_stop` | `std_msgs/Bool` | Hardware mux e-stop line |
| `/estop/trigger` | `std_msgs/Bool` | Manual e-stop |

Velocity multiplexing (`twist_mux`): Nav2, search, servo, and approach inputs are merged per `urc_autonomy/config/twist_mux.yaml`.

## Tests

Integration tests inject fake sensor traffic and assert state/cmd outputs (no hardware):

```bash
cd ros2_ws
colcon build --packages-select urc_autonomy visual_servo
source install/setup.bash
colcon test --packages-select urc_autonomy --event-handlers console_direct+
colcon test-result --verbose
```

## Training (YOLO)

Detect **mallet**, **rock_pick**, and **water_bottle** for science tasks. Full pipeline:

```bash
cd training
pip install ultralytics torch opencv-python
python train.py
python validate.py --model ../models/urc_objects/weights/best.pt
python export.py --model ../models/urc_objects/weights/best.pt   # TensorRT on Jetson
```

Place weights at `models/urc_objects.pt` (or update `model_path` in `autonomy.launch.py`).

## Scripts

| Script | Purpose |
|--------|---------|
| `scripts/build_dev.sh` | Build Docker image |
| `scripts/run_stack.sh` | `docker compose up autonomy` |
| `scripts/load_waypoints.sh` | Publish waypoints to mission node |
| `scripts/collect_data.sh` | Record training images |
| `scripts/status.sh` | Quick topic/node health check |

## Repository layout

```text
.
├── ros2_ws/src/          # ROS 2 packages
├── training/             # YOLO train / validate / export
├── scripts/              # Helper shell scripts
├── Dockerfile
└── docker-compose.yml
```

## License

MIT — see package manifests in each ROS package.
