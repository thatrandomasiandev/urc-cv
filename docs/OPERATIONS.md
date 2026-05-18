# Field operations runbook

Checklists for **bench testing**, **practice runs**, and **competition day**. Assume ROS 2 Humble is installed and workspace is built.

---

## Roles (suggested)

| Role | Responsibility |
|------|----------------|
| **Operator** | Launch stack, load waypoints, e-stop authority |
| **Perception** | Camera lens clean, debug image looks sane |
| **Navigation** | GPS fix quality, Nav2 not in error |
| **Safety** | E-stop wired, heartbeat understood |
| **Logger** | Bags / telemetry path, note run ID |

---

## Pre-flight checklist (every session)

### Hardware

- [ ] RealSense firmly mounted, USB seated, lens clean
- [ ] GPS antenna clear sky view (or known test coords)
- [ ] Battery sufficient for planned run + margin
- [ ] E-stop hardware tested (cuts drive when `/e_stop` true)
- [ ] `cmd_vel` reaches motor controller (bench wheel twitch test)

### Software

- [ ] `source /opt/ros/humble/setup.bash`
- [ ] `source ros2_ws/install/setup.bash`
- [ ] Model file present if using YOLO: `models/urc_objects.pt` (or param override)
- [ ] Waypoint file reviewed: `waypoints/*.json`
- [ ] Disk space for logs / bags

### Launch

```bash
./scripts/run_stack.sh
# OR: ros2 launch urc_autonomy autonomy.launch.py [args]
```

- [ ] `./scripts/status.sh` — nodes listed, no obvious crash
- [ ] `/mission/state` shows `IDLE` (or expected state)
- [ ] `/aruco/debug_image` shows live video when marker present
- [ ] `/estop/status` — `active: false` at rest

---

## Starting a mission

1. Confirm GPS has fix:  
   `ros2 topic echo /fix sensor_msgs/msg/NavSatFix --once`
2. Load waypoints:  
   `./scripts/load_waypoints.sh waypoints/your_mission.json`
3. Monitor state:  
   `ros2 topic echo /mission/state`
4. Expected flow: `IDLE` → `NAV_TO_WP` → (`SEARCHING`) → `VISUAL_SERVO` → `SUCCESS` → …

---

## During run — what to watch

| Signal | Healthy | Concerning |
|--------|---------|------------|
| `/mission/state` | Progresses through states | Stuck >2 min in one state |
| `/aruco/detection` | `confirmed` when marker in view | Only `pending` or nothing |
| `/cmd_vel` | Non-zero when driving | Zero while NAV_TO_WP should move |
| `/e_stop` | `false` | `true` without reason |
| Nav2 | Action goals accepted | Rejected / FAILED immediately |

---

## Manual interventions

### Pause / stop motion

```bash
# Software e-stop latch
ros2 topic pub --once /estop/trigger std_msgs/msg/Bool "{data: true}"
```

Clear when safe:

```bash
ros2 topic pub --once /estop/trigger std_msgs/msg/Bool "{data: false}"
```

### Abort mission logic

- No dedicated “abort” topic yet — use e-stop, then restart stack or reload waypoints after investigation.

### Re-load waypoints only

```bash
./scripts/load_waypoints.sh lat1,lon1 lat2,lon2
```

---

## Recording data

### ROS bag (sensors)

```bash
./scripts/collect_data.sh
# Ctrl+C to stop — creates urc_data_YYYYMMDD_HHMMSS/
```

### Telemetry CSV

Automatic to `~/urc_logs/run_*.csv` when `telemetry_node` is running.

Note run ID + weather + waypoint file in team log.

---

## Post-run checklist

- [ ] E-stop released, rover powered down safely
- [ ] Copy bag + `~/urc_logs` off Jetson if needed
- [ ] Note anomalies in team log (state stuck, false detections, GPS jump)
- [ ] If code changed: run `colcon test --packages-select urc_autonomy`

---

## Failure quick reference

| Symptom | First checks |
|---------|----------------|
| Stuck `NAV_TO_WP` | `/fix`? `ros2 action list` Nav2? mission logs “GPS anchor” |
| Never finds marker | Lighting, marker size, `confirm_frames`, search timeout |
| Jumps to SERVO then stops | Distance below `stop_distance_m`? stale detection timeout |
| Immediate e-stop | `/estop/status` reasons; marker too close? heartbeat? |
| Nav2 FAILED | Costmap, localization, goal in obstacle |
| No YOLO output | `model_path` file exists? `ros2 topic hz /objects/detections` |

Deep debugging: [ARCHITECTURE.md](ARCHITECTURE.md) §15.

---

## Launch argument reminders

```bash
ros2 launch urc_autonomy autonomy.launch.py \
  use_sim_time:=false \
  launch_realsense:=true \
  object_detector_device:=cuda
```

| Arg | Use when |
|-----|----------|
| `use_sim_time:=true` | Gazebo sim |
| `launch_realsense:=false` | Sim or external camera bridge |
| `object_detector_device:=cpu` | Mac dev / no GPU |

---

## Emergency contacts / escalation

Fill in for your team:

| Issue | Contact |
|-------|---------|
| Software / ROS | _________________ |
| Electrical / e-stop wiring | _________________ |
| Mechanical / camera mount | _________________ |

---

## Related docs

- Concepts: [TEAM_GUIDE.md](TEAM_GUIDE.md)  
- Terms: [GLOSSARY.md](GLOSSARY.md)  
- Internals: [ARCHITECTURE.md](ARCHITECTURE.md)
