# Working together: acquisition and reconstruction

Two authors, two areas, one contract. Acquisition decides **which images are taken** (explore, view and path planning, image count).
Reconstruction decides **what is made from them** (SfM, MVS, 3DGS, depth fusion, evaluation). They meet only at the run directory
([data-format.md](data-format.md) §3), which architecture.md §4 already enforces: `offline/` reads run directories and never imports `planner/`.

## Who owns what

| Area | Owner | Paths | Todo |
|---|---|---|---|
| Acquisition | author A | `src/kinetix/planner/`, `src/kinetix/rig/`, `src/kinetix/recon/` (online models: coverage, depthmap, roomfit, register), `config/`, `tests/nbv_bench.py`, `tests/unit/` (existing), `scripts/room_scan.sh` (fly part) | [todo.md](todo.md) |
| Reconstruction | author B | `src/kinetix/offline/` (one module per method: `sfm.py`, `densify.py`, `gsplat.py`, `eval.py`, ...), `tests/unit/test_offline_*.py` | [todo-reconstruction.md](todo-reconstruction.md) |
| Shared, change by PR with both reviewing | both | `src/kinetix/core/`, `src/kinetix/recon/align.py`, `src/kinetix/cli.py`, [data-format.md](data-format.md), [interfaces.md](interfaces.md), [publication-plan.md](publication-plan.md), `pyproject.toml` / `uv.lock`, `docker/` | — |

`offline/sfm.py` uses `recon/coverage.py` for the completeness proxy: a coverage change on the acquisition side must keep
`CoverageState.for_scene` and `box_surface` working, or say so in the PR.

## The contract: a run directory

- Acquisition **writes** `runs/<run>/`: `run.yaml`, `frames.jsonl`, `frames/`, `depth/`, `steps.jsonl`, `explore/`, `rosbag/`.
- Reconstruction **reads** those and writes only into its own subfolder: `runs/<run>/sfm/offline/` today, and one folder per new method
  (`runs/<run>/<method>/`, with a `metrics.json`). It never edits the acquisition files.
- A new field in `frames.jsonl` or `run.yaml` goes into data-format.md §3 first (CLAUDE.md rule), in a small PR to `main`.

## Shared data

`runs/` is git-ignored (GBs per flight), so the flights are shared outside git:

- **Frozen benchmark runs.** Reconstruction is compared on a fixed set of flights, so new acquisition work cannot shift its results.
  Start with `20261028-163032_warehouse_room_tour-explore-optimized_mavros_s0`. Add runs to the list here, never edit a listed run.
- Copy them to shared storage (`$KINETIX_DATA/runs/`, or rsync between the two workstations). Copy without `rosbag/` unless needed:
  `rsync -a --exclude rosbag runs/<run> <host>:<path>/runs/`.
- Acquisition reports a new method on its own new flights, then hands the run IDs over for reconstruction.

| Frozen run | Method | Keyframes | Notes |
|---|---|---|---|
| `20261028-163032_warehouse_room_tour-explore-optimized_mavros_s0` | tour + explore (optimized) | 267 | Warehouse; explore view 900004 has 7 points (BA bug, todo-reconstruction.md) |

## Branches

- `main` stays runnable: `uv run pytest` passes and `METHOD=tour scripts/room_scan.sh all` works.
- Long-lived branches `acquisition` (author A) and `reconstruction` (author B), each branched from `main`.
- Merge into `main` by pull request, small and often (at least weekly). Rebase your branch on `main` before opening one.
- A change to a shared path goes in its own PR, never mixed into an area PR, so the other author can review it in isolation.
- Each author updates only their own todo file. Paper-level decisions go in [decisions.md](decisions.md) by PR.
