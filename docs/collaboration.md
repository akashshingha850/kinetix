# Working together: acquisition and reconstruction

Two authors, two areas, one contract. Acquisition decides **which images are taken** (explore, view and path planning, image count).
Reconstruction decides **what is made from them** (SfM, MVS, 3DGS, depth fusion, evaluation). They meet only at the run directory
([data-format.md](data-format.md) §3) and share the small `kinetix/` library. `kinetix/tests/test_boundaries.py` enforces it: neither area
imports the other, and `kinetix/` imports neither.

## Folder tree

```
kinetix/              shared library (types, geometry, config, runlog, align, coverage, cli) + scenes/ + tests/
acquisition/          author A: planner/  mapping/  rig/  config/  tests/
reconstruction/       author B: sfm.py (densify.py, ...)  tests/
data/<run>/           flights, git-ignored: acquisition writes, reconstruction reads
results/<run>/<m>/    outputs, git-ignored: reconstruction writes, one folder per method
docs/                 shared docs at the top (architecture, data-format, interfaces, decisions, roadmap, setup, testing)
  acquisition/        architecture.md (loop, tour, explore, NBV), ros-pipeline.md, todo.md
  reconstruction/     pipeline.md, todo.md
  paper/              publication-plan.md, benchmark.md, research/
docker/  scripts/
```

## Who owns what

| Area | Owner | Paths | Todo |
|---|---|---|---|
| Acquisition | author A | `acquisition/` (planner, online mapping, rig, config, tests incl. `nbv_bench.py`), `data/`, `scripts/room_scan.sh` (fly part) | [acquisition/todo.md](acquisition/todo.md) |
| Reconstruction | author B | `reconstruction/` (one module per method: `sfm.py`, `densify.py`, `gsplat.py`, `eval.py`, ..., tests in `reconstruction/tests/`), `results/` | [reconstruction/todo.md](reconstruction/todo.md) |
| Shared, change by PR with both reviewing | both | `kinetix/` (incl. `scenes/`), [data-format.md](data-format.md), [interfaces.md](interfaces.md), [paper/publication-plan.md](paper/publication-plan.md), `pyproject.toml` / `uv.lock`, `docker/` | — |

`kinetix/coverage.py` is shared because both sides use it (planner gain, `reconstruction/sfm.py`'s completeness proxy): a change must keep
`CoverageState.for_scene` and `box_surface` working, or say so in the PR.

## The contract: a run directory

- Acquisition **writes** `data/<run>/`: `run.yaml`, `frames.jsonl`, `frames/`, `depth/`, `steps.jsonl`, `explore/`, `rosbag/`.
- Reconstruction **reads** those and writes only to `results/<run>/<method>/` (`sfm/` today, with a `metrics.json`). It never writes
  into `data/`.
- A new field in `frames.jsonl` or `run.yaml` goes into data-format.md §3 first (CLAUDE.md rule), in a small PR to `main`.

## Shared data

`data/` is git-ignored (GBs per flight), so the flights are shared outside git:

- **Frozen benchmark runs.** Reconstruction is compared on a fixed set of flights, so new acquisition work cannot shift its results.
  Start with `20261028-163032_warehouse_room_tour-explore-optimized_mavros_s0`. Add runs to the list here, never edit a listed run.
- Copy them to shared storage (`$KINETIX_DATA/data/`, or rsync between the two workstations). Copy without `rosbag/` unless needed:
  `rsync -a --exclude rosbag data/<run> <host>:<path>/data/`.
- Acquisition reports a new method on its own new flights, then hands the run IDs over for reconstruction.

| Frozen run | Method | Keyframes | Notes |
|---|---|---|---|
| `20261028-163032_warehouse_room_tour-explore-optimized_mavros_s0` | tour + explore (optimized) | 267 | Warehouse; explore view 900004 has 7 points (BA bug, reconstruction/todo.md) |

## Branches

- `main` stays runnable: `uv run pytest` passes and `METHOD=tour scripts/room_scan.sh all` works.
- Long-lived branches `acquisition` (author A) and `reconstruction` (author B), each branched from `main`.
- Merge into `main` by pull request, small and often (at least weekly). Rebase your branch on `main` before opening one.
- A change to a shared path goes in its own PR, never mixed into an area PR, so the other author can review it in isolation.
- Each author updates only their own todo file. Paper-level decisions go in [decisions.md](decisions.md) by PR.
