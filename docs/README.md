# Documentation index

Use this page to pick the right doc. **New teammates should follow the path in order.**

---

## Start here (new to the project)

| # | Document | Time | You will learn |
|---|----------|------|----------------|
| 1 | **[TEAM_GUIDE.md](TEAM_GUIDE.md)** | 15 min | What the stack does, mission story, safety, repo map, FAQ |
| 2 | **[GLOSSARY.md](GLOSSARY.md)** | 10 min | ROS terms, topic names, commands |
| 3 | **[GETTING_STARTED.md](GETTING_STARTED.md)** | 30–60 min | Clone, build, first commands, tests |

---

## Presenting to the team

| Document | Use for |
|----------|---------|
| **[PRESENTATION.md](PRESENTATION.md)** | Slide-by-slide outline + live demo script (~30 min) |

Hand out links to **TEAM_GUIDE** and **GETTING_STARTED** after the meeting.

---

## Running on the rover or at practice

| Document | Use for |
|----------|---------|
| **[OPERATIONS.md](OPERATIONS.md)** | Pre-flight checklist, monitoring, e-stop, failures |

---

## Engineering deep dive

| Document | Use for |
|----------|---------|
| **[ARCHITECTURE.md](ARCHITECTURE.md)** | FSM internals, PID, gating, mux, JSON schemas, parameters |

Read this when you are **changing code** or debugging subtle behavior — not required for your first day.

---

## Machine learning (object detection)

| Document | Use for |
|----------|---------|
| **[../training/README.md](../training/README.md)** | Datasets, train, validate, export to Jetson |

---

## Repository root

| Document | Use for |
|----------|---------|
| **[../README.md](../README.md)** | Project summary, quick start, topic table |

---

## Suggested reading by role

| Role | Path |
|------|------|
| **Mechanical / electrical** | TEAM_GUIDE → OPERATIONS (hardware sections) → GLOSSARY (topics only) |
| **New software member** | TEAM_GUIDE → GETTING_STARTED → ARCHITECTURE |
| **ML / perception** | TEAM_GUIDE → training/README → ARCHITECTURE §5, §8 |
| **Team lead presenting** | PRESENTATION → TEAM_GUIDE |
| **Driver / field operator** | OPERATIONS → GLOSSARY (topics table) |
