# Kinetix

*Fewer flights. Smarter views. Better models.*

Adaptive next-best-view photogrammetry of indoor objects with a monocular drone. Simulation and flight run on the
`bisg_isaac` digital twin (`~/bisg_isaac`) (Isaac Sim 6.0 + Pegasus + PX4 + MAVROS).

**Status:** K0, design and environment. No runtime code yet. See [docs/todo.md](docs/todo.md).

## Quick start

```bash
uv sync --extra ml
uv run python scripts/check_env.py
```

## Documentation map

| Need to… | Read |
|---|---|
| understand the idea and the research gap | [docs/research/idea.md](docs/research/idea.md), [related_work.md](docs/research/related_work.md) |
| know why mono is the method and stereo the comparison | [docs/research/mono-vs-stereo.md](docs/research/mono-vs-stereo.md) |
| know what is novel and what is already taken | [docs/research/reports/Kinetix novelty search.md](docs/research/reports/Kinetix%20novelty%20search.md) |
| know what we publish, where and when | [docs/publication-plan.md](docs/publication-plan.md) |
| see how the system is built | [docs/architecture.md](docs/architecture.md) |
| write code against an API, topic or frame | [docs/interfaces.md](docs/interfaces.md) |
| read or write scenes, pools, runs | [docs/data-format.md](docs/data-format.md) |
| know what to build next and when it is done | [docs/roadmap.md](docs/roadmap.md), [docs/todo.md](docs/todo.md) |
| write or run tests | [docs/testing.md](docs/testing.md) |
| run or extend the benchmark | [docs/benchmark.md](docs/benchmark.md) |
| know why something is the way it is | [docs/decisions.md](docs/decisions.md) |
| set up a machine | [docs/setup.md](docs/setup.md) |
| original week plan | [docs/research/work_plan.md](docs/research/work_plan.md) |
