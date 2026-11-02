# Kinetix

*Fewer flights. Smarter views. Better models.*

Adaptive next-best-view photogrammetry of indoor objects and whole rooms with a monocular drone. Simulation and flight run on
the `bisg_isaac` digital twin (`~/bisg_isaac`) (Isaac Sim 6.0 + Pegasus + PX4 + MAVROS).

**Status:** K1. The object demo loop and whole-room capture fly on the bisg twin: a planned 2-minute tour of the Warehouse
reconstructs with every keyframe registered and 8 cm camera error (`METHOD=tour scripts/room_scan.sh all`,
[acquisition/architecture.md](docs/acquisition/architecture.md) §5a). Pools, online SfM and the K3 scorers are next. See [docs/acquisition/todo.md](docs/acquisition/todo.md).

## Quick start

```bash
uv sync --extra ml
uv run python scripts/check_env.py
```

## Documentation map

Two authors share this repo ([docs/collaboration.md](docs/collaboration.md)): `acquisition/` decides which images are taken,
`reconstruction/` what is made from them, and `kinetix/` is the small library both use. `docs/` follows the same split.

| Need to… | Read |
|---|---|
| **Start here**: reading paths by role, every doc in one line, how to debug a run | [docs/README.md](docs/README.md) |
| **Shared** | |
| know who owns what, the folder tree, branches, frozen runs | [docs/collaboration.md](docs/collaboration.md) |
| see how the system is built (tiers, sensing modes, module map) | [docs/architecture.md](docs/architecture.md) |
| read or write scenes, runs, results | [docs/data-format.md](docs/data-format.md) |
| write code against an API, topic or frame | [docs/interfaces.md](docs/interfaces.md) |
| know why something is the way it is | [docs/decisions.md](docs/decisions.md) |
| know the build phases and their exit tests | [docs/roadmap.md](docs/roadmap.md) |
| set up a machine, write or run tests | [docs/setup.md](docs/setup.md), [docs/testing.md](docs/testing.md) |
| **Acquisition** | |
| the session loop, room tours, exploration, NBV scoring, constraints | [docs/acquisition/architecture.md](docs/acquisition/architecture.md) |
| the ROS 2 node graph, topic budget and QoS (proposed) | [docs/acquisition/ros-pipeline.md](docs/acquisition/ros-pipeline.md) |
| what is in progress | [docs/acquisition/todo.md](docs/acquisition/todo.md) |
| **Reconstruction** | |
| the offline pipeline (SfM, densify, evaluate) | [docs/reconstruction/pipeline.md](docs/reconstruction/pipeline.md) |
| what is in progress | [docs/reconstruction/todo.md](docs/reconstruction/todo.md) |
| **Paper** | |
| what we publish, where and when | [docs/paper/publication-plan.md](docs/paper/publication-plan.md) |
| run or extend the benchmark (experiments, metrics, reporting) | [docs/paper/benchmark.md](docs/paper/benchmark.md) |
| the idea, related work, mono vs stereo | [idea](docs/paper/research/idea.md), [related_work](docs/paper/research/related_work.md), [mono-vs-stereo](docs/paper/research/mono-vs-stereo.md) |
| what is novel and what is taken (objects; rooms) | [novelty search](docs/paper/research/reports/kinetix-novelty-search.md), [room scoping review](docs/paper/research/reports/room-photogrammetry-scoping-review.md) |
| original week plan | [docs/paper/research/work_plan.md](docs/paper/research/work_plan.md) |
