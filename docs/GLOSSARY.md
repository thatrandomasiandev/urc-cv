# Glossary

Plain-language definitions for ROS, URC, and terms used in this repo.

---

## Competition & hardware

| Term | Meaning |
|------|---------|
| **URC** | University Rover Challenge — annual rover competition. |
| **Waypoints** | GPS coordinates the rover must visit in order. |
| **ArUco marker** | Printed black-and-white square fiducial; we use dictionary `4x4_50`, 20 cm face. |
| **RealSense** | Intel depth camera (color + depth); topics under `/camera/`. |
| **Jetson** | NVIDIA onboard computer on the rover (often Orin Nano). |
| **Science objects** | Task items we detect with YOLO: mallet, rock pick, water bottle. |

---

## ROS 2 basics

| Term | Meaning |
|------|---------|
| **Node** | One running program in ROS (e.g. `mission_node`). |
| **Topic** | Named message stream (e.g. `/mission/state`). Nodes publish and subscribe. |
| **Message** | Data type on a topic (`Twist`, `String`, `NavSatFix`, …). |
| **Publisher** | Node that sends messages on a topic. |
| **Subscriber** | Node that receives messages from a topic. |
| **Launch file** | Script that starts a set of nodes with parameters — `autonomy.launch.py`. |
| **Parameter** | Config value on a node (e.g. `stop_distance_m:=1.8`). |
| **QoS** | Quality of service — delivery rules; waypoints use “latched” (transient-local) QoS. |
| **Frame** | Coordinate system label (`map`, `odom`, `base_link`, `camera_link`). |
| **TF** | Transform tree — how frames relate in space (used by Nav2 and perception). |
| **Bag** | Recorded ROS data file for replay / dataset extraction. |

---

## Our stack — nodes

| Name | One-line job |
|------|----------------|
| **mission_node** | State machine: waypoints → drive → search → servo → success. |
| **aruco_detector** | Find ArUco in images; publish bearing & distance. |
| **servo_node** | Closed-loop drive toward marker when active. |
| **search_node** | Rotate + arc search when marker not found at waypoint. |
| **estop_node** | Watch health; publish emergency stop. |
| **object_detector_node** | YOLO inference on camera. |
| **approach_node** | Drive toward a selected object class. |
| **gps_odom_node** | Convert GPS fix to odometry in `map`. |
| **telemetry_node** | Log events to CSV under `~/urc_logs`. |
| **twist_mux** | Merge multiple `cmd_vel_*` inputs into one `/cmd_vel`. |
| **Nav2** | Path planning and obstacle avoidance stack (external package). |
| **EKF** | `robot_localization` filter fusing GPS + visual odom + IMU. |

---

## Our stack — topics you will hear often

| Topic | Type | Meaning |
|-------|------|---------|
| `/mission/waypoints` | JSON string | List of `{lat, lon}` to visit. |
| `/mission/state` | string | `IDLE`, `NAV_TO_WP`, `SEARCHING`, `VISUAL_SERVO`, `SUCCESS`, `FAILED`. |
| `/mission/cmd` | JSON string | Commands like `START_SERVO`, `START_SEARCH`. |
| `/aruco/detection` | JSON string | Best marker: id, distance, bearing, confidence. |
| `/objects/detections` | JSON array | YOLO detections. |
| `/cmd_vel` | Twist | Final linear/angular command to base driver. |
| `/cmd_vel_nav2` | Twist | Nav2’s command (into mux). |
| `/cmd_vel_servo` | Twist | Visual servo command (into mux). |
| `/cmd_vel_search` | Twist | Search behavior command (into mux). |
| `/e_stop` | bool | When true, mux stops the rover. |
| `/fix` | NavSatFix | GPS position. |
| `/aruco/debug_image` | image | Annotated camera view for demos/debug. |

---

## Message fields (ArUco JSON)

| Field | Meaning |
|-------|---------|
| `id` | Which marker ID (0, 1, …). |
| `distance_m` | Meters from camera to marker (estimate). |
| `bearing_deg` | Degrees off center; positive = marker to the **right**. |
| `confidence` | `pending` (not trusted yet) or `confirmed` (mission listens). |
| `depth_validated` | Whether RealSense depth agreed with geometry. |

---

## Commands on `/mission/cmd`

| Command | Who reacts |
|---------|------------|
| `START_SEARCH` | search_node |
| `STOP_SEARCH` | search_node |
| `START_SERVO` | servo_node |
| `STOP_SERVO` | servo_node |
| `START_APPROACH` | approach_node (optional `target` class) |
| `STOP_APPROACH` | approach_node |

---

## Tools & repo

| Term | Meaning |
|------|---------|
| **colcon** | ROS 2 build tool (`colcon build`, `colcon test`). |
| **Workspace** | `ros2_ws/` — source and build trees. |
| **Docker dev container** | Consistent Ubuntu + ROS environment on any laptop. |
| **YOLO / Ultralytics** | Object detection library used in `training/`. |
| **TensorRT engine** | Optimized model file for fast inference on Jetson. |

---

## Abbreviations

| Abbr | Meaning |
|------|---------|
| **FSM** | Finite state machine (`mission_node`). |
| **PID** | Proportional–integral–derivative controller (servo bearing). |
| **PnP** | Perspective-n-point — pose from 2D points + 3D model. |
| **CLAHE** | Contrast enhancement for bright outdoor images. |
| **EKF** | Extended Kalman filter for sensor fusion. |
