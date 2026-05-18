# Presentation outline — introducing the stack to teammates

Use this as a **slide deck script** or meeting agenda (~20–30 minutes + demo). Pair with live `status.sh` and `/aruco/debug_image` if hardware is available.

---

## Slide 1 — Title

**URC 2026 Autonomy Stack**  
GPS navigation + ArUco acquisition + object detection  
Repo: https://github.com/thatrandomasiandev/urc-cv

---

## Slide 2 — Why this exists

- URC tasks require driving to locations and interacting with targets.
- We need **repeatable, testable software** — not one-off scripts.
- This repo is the **integrated ROS 2 stack** that runs on the Jetson at competition.

**Say:** “Everything talks over ROS topics so we can test pieces separately and run the full mission with one launch command.”

---

## Slide 3 — What you do NOT need to know today

- Every line of C++ in Nav2
- Kalman filter math
- How to train YOLO from scratch (unless you’re on ML)

**You DO need:** what each part *does*, how to start/stop it, and where to look when something breaks.

---

## Slide 4 — Hardware picture

```text
┌─────────────┐     USB      ┌──────────┐
│  RealSense  │─────────────►│  Jetson  │
└─────────────┘              │  ROS 2   │
┌─────────────┐     serial/  │  stack   │──► /cmd_vel ──► motor controller
│    GPS      │─────────────►│          │◄── /e_stop
└─────────────┘              └──────────┘
```

---

## Slide 5 — Software layers (manager + specialists)

| Layer | Package | Job |
|-------|---------|-----|
| Mission | urc_autonomy | Decides IDLE → drive → search → approach → done |
| Drive far | Nav2 | Path to GPS waypoint |
| See marker | aruco_detector | Bearing + distance |
| Search | search_behavior | Spin / arc if marker missing |
| Approach | visual_servo | Close the last meters |
| Science | object_detector + approach | YOLO objects (parallel) |
| Safety | estop_node + twist_mux | Stop + one driver at a time |

**Diagram:** [TEAM_GUIDE.md](TEAM_GUIDE.md) mermaid chart

---

## Slide 6 — The mission timeline (tell a story)

1. Operator loads waypoints  
2. **NAV_TO_WP** — Nav2 drives  
3. Maybe **SEARCHING** — look around  
4. **VISUAL_SERVO** — drive to marker  
5. **SUCCESS** — next waypoint or done  

**Diagram:** state machine in [TEAM_GUIDE.md](TEAM_GUIDE.md)

---

## Slide 7 — “Confirmed” detections (trust but verify)

- Camera is fast; one bad frame is not enough.
- We require **3 consistent frames** before `confirmed`.
- Mission **ignores** pending detections.

**Demo point:** Show `/aruco/debug_image` — green vs yellow labels.

---

## Slide 8 — Who drives the wheels?

Only **one** behavior at a time via `twist_mux`:

`e-stop` > `servo` > `approach` > `search` > `nav2`

**Analogy:** walkie-talkie — only one person talks to the driver.

---

## Slide 9 — Safety

E-stop if:

- Operator hits stop  
- Mission heartbeat lost (~3 s)  
- Marker closer than ~0.5 m  

**Live check:** `ros2 topic echo /estop/status --once`

---

## Slide 10 — Science / YOLO (phase 2 story)

- Detects: **mallet**, **rock_pick**, **water_bottle**
- Training pipeline in `training/`
- Approach node ready; mission FSM integration can grow later

---

## Slide 11 — Repo tour (2 min)

- `ros2_ws/src/` — nodes  
- `docs/` — you are reading this  
- `scripts/` — status, waypoints, record bags  
- `models/` + `datasets/` — ML assets  

---

## Slide 12 — How to contribute without fear

| Task | Entry point |
|------|-------------|
| Field testing | [OPERATIONS.md](OPERATIONS.md) |
| Tune distances/timeouts | `autonomy.launch.py` |
| Marker detection | `aruco_detector/detector_node.py` |
| Approach feel | `visual_servo/servo_node.py` |
| New mission states | `mission_node.py` + tests in `urc_autonomy/test/` |

Always run `colcon test` after changing mission/servo/e-stop.

---

## Slide 13 — Live demo script (10 min)

**If rover or bench camera available:**

1. Start stack — `docker compose up autonomy` or `./scripts/run_stack.sh`
2. `./scripts/status.sh` — show nodes + state
3. Open `/aruco/debug_image` — wave a printed marker
4. `ros2 topic echo /aruco/detection --once` — show JSON
5. Load waypoints — `./scripts/load_waypoints.sh waypoints/example_waypoints.json`
6. Watch `/mission/state` transition (if GPS + Nav2 ready)

**If no hardware:**

1. Walk through [TEAM_GUIDE.md](TEAM_GUIDE.md) diagrams  
2. Show GitHub `docs/` folder  
3. Run `colcon test` recording (screen share)

---

## Slide 14 — Documentation map

| Doc | When |
|-----|------|
| [TEAM_GUIDE.md](TEAM_GUIDE.md) | First read |
| [GLOSSARY.md](GLOSSARY.md) | Terms |
| [GETTING_STARTED.md](GETTING_STARTED.md) | Setup |
| [OPERATIONS.md](OPERATIONS.md) | Test day |
| [ARCHITECTURE.md](ARCHITECTURE.md) | Deep dive / debugging |

---

## Slide 15 — Q&A prompts

- What happens if GPS is wrong?  
- Can we run without Nav2? (servo/search only in lab)  
- How do we add a new mission phase?  
- Where do logs go? (`~/urc_logs`, telemetry CSV)

Full FAQ: [TEAM_GUIDE.md](TEAM_GUIDE.md) § Common questions

---

## Handout (link in chat)

Send teammates:

1. Repo link  
2. [TEAM_GUIDE.md](https://github.com/thatrandomasiandev/urc-cv/blob/main/docs/TEAM_GUIDE.md)  
3. [GETTING_STARTED.md](https://github.com/thatrandomasiandev/urc-cv/blob/main/docs/GETTING_STARTED.md)
