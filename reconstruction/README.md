# reconstruction/: 3D reconstruction and evaluation (Taufiq)

This package turns the images a drone captured into a 3D model and scores that model against ground truth (GT). The same
captured frames go through several reconstruction algorithms, and the results are compared on one set of metrics:

| Method | Family | Status |
|---|---|---|
| **COLMAP** (pycolmap) | classical SfM, known-pose triangulation or incremental mapping | implemented: [sfm.py](sfm.py) |
| **COLMAP MVS / 3DGS** | dense reconstruction on top of SfM | planned: `densify.py` ([todo](../docs/reconstruction/todo.md)) |
| **Depth Anything 3** | feed-forward multi-view depth and pose | to add |
| **VGGT** | feed-forward cameras, depth and point maps | to add |
| **π³ (Pi3)** | feed-forward permutation-equivariant point maps | to add |

The "Status" column describes the repo on branch `reconstruction` (identical to `main` at commit `6a1f7b2` when this file was
written). Only COLMAP SfM exists in code. Everything else in this README is a plan for how to add the rest.

Contents: [1 Scope](#1-scope-and-rules) · [2 Inputs](#2-inputs-a-run-directory) · [3 What exists](#3-what-exists-today-sfmpy) ·
[4 Adding a method](#4-adding-a-reconstruction-method) · [5 Evaluation](#5-evaluation-against-ground-truth) ·
[6 Setup and commands](#6-setup-and-commands) · [7 Known issues](#7-known-issues-and-open-work) · [8 Working on the branch](#8-working-on-this-branch)

---

## 1. Scope and rules

Reconstruction owns *what is made from the images*. Acquisition (`acquisition/`) owns which images get taken. Rules from
[CLAUDE.md](../CLAUDE.md) and [collaboration.md](../docs/collaboration.md) that matter here:

- **Meeting point is the run directory.** Read `data/<run>/`, write only `results/<run>/<method>/`. Never write into `data/`.
- **No cross-imports.** `reconstruction/` must not import `acquisition/`, and `kinetix/` imports neither area
  (`kinetix/tests/test_boundaries.py` checks it). Only `kinetix/cli.py` dispatches to both.
- **Shared paths change by their own PR**: `kinetix/` (including `kinetix/scenes/`), `docs/data-format.md`, `docs/interfaces.md`,
  `pyproject.toml`, `uv.lock`, `docker/`. Adding a dependency such as `vggt` is a shared-path change, so do it in a separate PR.
- **Poses are `T_map_cam`**: ENU `map` frame, OpenCV optical camera (x right, y down, z forward), quaternions xyzw. Conversion to
  COLMAP or OpenGL conventions belongs only in `kinetix.geometry`.
- **Compare on frozen runs only.** The benchmark run list is in [collaboration.md](../docs/collaboration.md). Add runs, never edit a
  listed one. Currently: `20261028-163032_warehouse_room_tour-explore-optimized_mavros_s0` (267 keyframes).
- Update [docs/reconstruction/todo.md](../docs/reconstruction/todo.md) when work finishes. That is your todo file. Do not edit
  `docs/acquisition/todo.md`.
- Python runs through `uv` (`uv run ...`). Large data lives in `$KINETIX_DATA`, outside the repo.

## 2. Inputs: a run directory

One flight is one directory, `data/<run_id>/` ([data-format.md §3](../docs/data-format.md)):

```
run.yaml        resolved config, camera intrinsics, provenance
frames.jsonl    one JSON line per keyframe
frames/         left images (PNG), named in frames.jsonl as "frames/000005.png"
depth/<idx>.npy ZED depth_registered per keyframe (float32 m, NaN/inf = no return), recorded but not yet used by any method
steps.jsonl     planning steps (acquisition's log)
rosbag/         MCAP recording (GBs, usually not copied)
```

Each `frames.jsonl` line carries what a reconstruction method needs:

| Field | Meaning |
|---|---|
| `image` | path to the left image, relative to the run directory |
| `K` | `{fx, fy, cx, cy, ...}`: the ZED rectified image is undistorted, so a PINHOLE camera with no distortion |
| `pose` | rig pose `T_map_cam` from the drone's own EKF2 estimate (`p` xyz, `q` xyzw). Available on real flights too |
| `gt_pose` | simulator ground-truth pose, **sim only**. Use for evaluation and alignment, never as an input to a method |
| `depth`, `depth_source` | path to the ZED depth `.npy` and `"zed"`, when recorded |

Explore views have indices 900000 and up (`pool_id` `explore_<n>`). Some have very few triangulated points (see
[7](#7-known-issues-and-open-work)).

**Mono vs stereo.** The project's method is monocular (left image, intrinsics, vehicle pose). The ZED depth is allowed as a
*recorded* extra for comparison studies. When a method uses it, the result must say so in its `metrics.json`.

**The data is not in git.** At the time of writing there is no `data/` folder on this machine and `KINETIX_DATA` is unset. Copy the
frozen run from shared storage without the rosbag: `rsync -a --exclude rosbag <host>:<path>/data/<run> data/`
([collaboration.md](../docs/collaboration.md)).

## 3. What exists today: `sfm.py`

`kinetix sfm <run>` calls `run_sfm()` in [sfm.py](sfm.py) and writes `results/<run>/sfm/`:
`database.db`, `pairs.txt`, `sparse/<k>/` (COLMAP binary), `points.ply` (in `map` frame) and `metrics.json`.

Pipeline:

1. **Features**: SIFT, up to 8192 per image, one shared PINHOLE camera fixed to the run's `K` (focal length and principal point are not refined).
2. **Pairing** (`--pairing`): `video` (default above 150 frames: each frame with its 10 successors plus its 8 most co-visible
   frames for loop closure), `covis` (pairs that see the same proxy-surface points) or `exhaustive`. The Warehouse repeats every
   6 m, so exhaustive matching registers images into the wrong bay. That is why pairing uses the rig poses.
3. **Rig check** (`--no-rig-check` disables): drops verified pairs whose relative rotation (more than 5°) or baseline direction
   (more than 30°) disagrees with the EKF2 poses.
4. **Poses** (`--pose-source`):
   - `prior` (default for `tour` runs and more than 150 frames): cameras are placed at the EKF2 poses, points are triangulated,
     then 2 rounds of bundle adjustment with fixed intrinsics.
   - `sfm`: COLMAP incremental mapping with EKF2 camera centres as position priors (`--prior-sigma`, default 0.5 m).
5. **Metrics** in `metrics.json`: registered fraction, point count, mean track length, reprojection error, ATE vs EKF2 and vs GT
   (Sim3-aligned, from `kinetix/align.py`), and a `completeness_proxy`.

Reference result on the 2026-10-27 Warehouse tour (from [acquisition/todo.md](../docs/acquisition/todo.md)): 314/314 registered,
21.9k points, reprojection error 0.64 px, ATE vs GT 14.5 cm (median 8.4 cm), EKF2 vs GT 0.45 m RMS.

Reusable pieces: `kinetix/align.py` (`umeyama`, `apply`, `ate`), `kinetix/coverage.py` (`CoverageState`, `box_surface`: the proxy
surface), `kinetix/geometry.py` (`quat_to_R`, `R_to_quat`), `kinetix/config.py` (`RESULTS`, `SCENES`, `load_scene`).

**What the current completeness number is.** `completeness_proxy` checks the sparse cloud against a *box model* of the room
(walls, floor, ceiling), not a real mesh. It is a lower bound, and it says nothing about accuracy. The Warehouse scene has
`gt_mesh: null` ([warehouse_room.yaml](../kinetix/scenes/warehouse_room.yaml)). So there is **no real GT surface yet**, which is
the first thing to build (see [5](#5-evaluation-against-ground-truth)).

## 4. Adding a reconstruction method

Goal: COLMAP, Depth Anything 3 (DA3), VGGT and π³ run on identical frames and produce outputs that the same evaluator reads.

### 4.1 What the methods produce

| | COLMAP | DA3 | VGGT | π³ |
|---|---|---|---|---|
| Needs known poses | no (can use them as priors) | optional | no | no |
| Output | sparse points, poses | depth + poses (check which variant) | per-view cameras, depth, point maps | per-view point maps and cameras |
| Scale / frame | arbitrary (Sim3-aligned afterwards) | model-dependent, arbitrary | arbitrary | arbitrary |
| Frames per forward pass | any (pairs) | limited by GPU memory | limited by GPU memory | limited by GPU memory |

The last row matters. A 267-frame tour will not fit in one forward pass for the feed-forward models. Plan on **chunking** the
frame sequence into overlapping windows, running the model per chunk, and merging chunks by Sim3 alignment on shared frames.
Everything in the table about each model's outputs and limits is from general knowledge and must be checked against the model's
repo and paper before you rely on it. Also check each model's code and weight **licence** before committing to it.

### 4.2 Proposed layout

One module per method (the convention in collaboration.md), all with the same signature:

```
reconstruction/
  sfm.py            COLMAP (exists)
  densify.py        COLMAP MVS / 3DGS (planned)
  da3.py            Depth Anything 3
  vggt.py           VGGT
  pi3.py            π³
  fusion.py         per-view depth/point maps + poses -> one fused point cloud / mesh (TSDF); shared by the feed-forward methods
  metrics.py        accuracy, completeness, Chamfer, F-score, ATE (shared)
  evaluate.py       results/<run>/<method>/ -> metrics.json (shared)
  tests/            unit tests (the folder does not exist yet)
```

Contract for every method (a suggestion, not code that exists):

```python
def run(run_dir: Path, out_dir: Path | None = None, **opts) -> dict:
    """Read data/<run>/ only. Write results/<run>/<method>/{points.ply | mesh.ply, cameras.json, metrics.json}. Return metrics."""
```

Every method should write, in the **`map` frame, metres**:

- `points.ply` (or `mesh.ply`): the reconstruction after Sim3 alignment.
- `cameras.json`: estimated `T_map_cam` per image, keyed by the `image` path from `frames.jsonl` (so ATE needs no model-specific code).
- `metrics.json`: the common schema from [5](#5-evaluation-against-ground-truth), plus method-specific fields
  (model name and weights version, chunk size, runtime, peak GPU memory, whether ZED depth or EKF2 poses were used).

### 4.3 Rules that keep the comparison fair

- Same frames, same intrinsics, same ordering for every method. Record the frame list in `metrics.json`.
- Say what each method is given: images only, images plus EKF2 poses, or images plus ZED depth. Do not compare a pose-prior method
  with an image-only one without stating it. The project's headline is the monocular case.
- Never feed `gt_pose` or GT geometry into a method. It is only for alignment and scoring.
- Align every result to the **same reference** with the same procedure (see 5.1).
- Report failures (non-registered frames, out-of-memory, chunk merge failures) as numbers, not by dropping the frame.

### 4.4 Dependencies and conventions

- `torch` is already in the optional `ml` extra (`uv sync --extra ml`), pinned to the `pytorch-cu128` index. This machine has two
  RTX 5090 GPUs (checked with `nvidia-smi`), so confirm that the pinned torch build supports them. The docs describe workstation B
  (2× RTX 6000 Ada), a different machine.
- DA3, VGGT and π³ are research repos with their own requirements. Install them as separate optional extras or in their own
  environment if they conflict with the pins. A dependency change is its own PR (shared path).
- Import heavy libraries lazily inside `run()`, as `sfm.py` does with `pycolmap`, so `uv run pytest` stays fast without the weights.
- Camera convention: COLMAP uses `cam_from_world` with OpenCV axes. Several feed-forward models use their own axes. Put any
  conversion in `kinetix.geometry` (that is a shared-path PR), not inline in a method module.

## 5. Evaluation against ground truth

The protocol is in [docs/paper/benchmark.md §3](../docs/paper/benchmark.md): Sim3-align to GT camera centres, crop to the
region of interest, compare with the *observable* GT surface, and use thresholds relative to the ROI diagonal `D`.
Planned in `metrics.py` / `evaluate.py` (neither exists yet):

| Metric | Definition |
|---|---|
| registered fraction | registered frames / captured frames |
| ATE | RMSE of camera centres after Sim3 (cm); `kinetix.align.ate` exists |
| accuracy | mean distance reconstruction → GT (mm) |
| completeness | mean distance GT → reconstruction (mm) |
| Chamfer | (accuracy + completeness) / 2 |
| F-score@τ | harmonic mean of precision and recall, τ = 0.5 % and 1 % of `D` |
| time, memory | per stage; add peak GPU memory for the learned methods |

Guard from the benchmark spec: if the alignment residual is above 5 % of `D`, or the registered fraction is below 50 %, mark the
run `eval_valid: false` and exclude it from means.

### 5.1 Alignment

Feed-forward models have no metric scale and use their own frame. Estimate one Sim3 per result with `kinetix.align.umeyama`
between the estimated camera centres and the reference camera centres. Use **GT centres (`gt_pose`)** to score in simulation,
because the EKF2 poses drift (0.45 m RMS vs GT on the reference tour) and would put the drift into the error. On real flights
use the rig poses.

The reconstruction is then compared with the GT surface in `map`. Do not use the points of cameras that are far from their
reference pose. That is the "few-points camera" problem below, and it applies to learned methods too.

### 5.2 Ground truth: the first thing to build

The Warehouse has no GT mesh: `gt_mesh: null`. The todo already names the fix: export a mesh from the bisg USD of the emptied
room (floor, stripes, walls, columns, ceiling, roof beams) into `map` coordinates, so accuracy and completeness are measured
cloud-to-mesh rather than against the room box. Two practical points:

- The USD lives in `~/bisg_isaac` (assets: Isaac `Simple_Warehouse/warehouse.usd`). CLAUDE.md says to use only its interface
  contract and never copy its code. Exporting a mesh from the asset is a data task, but agree with the bisg owner first, and the
  scene note says the asset is not redistributable (`redistributable: false`), so the mesh stays out of git.
- The `map` frame is world plus a measured offset: `sim.world_to_map: [2.376, 0.0, 0.116]` in
  [warehouse_room.yaml](../kinetix/scenes/warehouse_room.yaml). Use it, or you will be off by that offset.
- Score against the **observable** surface (raycast from the feasible camera poses), not the whole mesh, or completeness will
  punish methods for surfaces the drone could never see. The scene file's `roi` box gives the crop.

A mesh with room-box geometry only (walls, floor and ceiling from `roi`) is a quick interim reference, but it ignores the
columns and beams, so label it as such in `metrics.json`.

## 6. Setup and commands

```bash
git switch reconstruction            # local branch tracking origin/reconstruction (see section 8)
uv sync --extra ml                   # env from uv.lock, includes torch
uv run python scripts/check_env.py   # every line should say ok
uv run pytest                        # 58 unit tests per the docs, about 90 s; not run for this README
```

Run the existing method on a flight (the scene is looked up from `run.yaml`):

```bash
uv run python -m kinetix sfm data/20261028-163032_warehouse_room_tour-explore-optimized_mavros_s0
# options: --pairing {video,covis,exhaustive}  --pose-source {prior,sfm}  --prior-sigma 0.5
#          --max-features 8192  --min-shared 10  --no-rig-check  --scene <scene.yaml>
```

Output goes to `results/<run>/sfm/`. `kinetix eval` and `kinetix bench` are described in the docs but **do not exist yet**.
`cli.py` only has `run` and `sfm`. A new method needs a subcommand there (a small shared-path change), for example
`kinetix recon <method> <run>`.

Tests go in `reconstruction/tests/` (create it). Cheap, useful ones: Sim3 alignment recovers a known transform, the metric
functions on a synthetic plane and sphere, and a method's output loader on a tiny fixture. `tests/fixtures/` is the git-lfs
location for small data. Mark GPU tests `@pytest.mark.gpu`.

## 7. Known issues and open work

From [docs/reconstruction/todo.md](../docs/reconstruction/todo.md), in the order I would take them:

1. **Cameras with too few points get moved far off by bundle adjustment** in `sfm.py`: explore views with 4 to 8 points gave ATE max
   9.7 to 20.6 m, while the other cameras are at 0.14 to 0.16 m RMSE. Hold those cameras at their prior pose or drop them. Any
   ATE number you compare must say whether they were included, because they dominate RMSE.
2. **No real GT mesh** for the Warehouse (see 5.2). Everything else in the evaluation depends on it.
3. **One `metrics.json` schema for every method**, so the paper tables come from a single script (no hand-edited tables).
4. **Dense reconstruction** (`densify.py`: COLMAP MVS needs the CUDA COLMAP container, because the pip pycolmap wheel is CPU-only; or
   gsplat). The sparse completeness (0.41 on the frozen run) is a lower bound.
5. **Use the recorded ZED depth**: fusion (TSDF), depth priors for MVS/3DGS, scale. Plain orange wall panels give half-invalid
   depth (frame 900024 of flight `134802`). The sim ZED reads long up to 10 m and short beyond.
6. **Mono vs stereo as reconstruction input**: the same frames with and without the ZED depth or right image.
7. Observed in the data, not looked into: 16.5 % of `134802`'s sparse points lie more than 0.25 m outside the true room box
   (most beyond x+).

**Things to expect from this scene** (they affect all four methods, not only COLMAP):

- Repeating structure: the Warehouse repeats every 6 m, and many panels are plain. Wrong-bay matches, low texture and far points
  (median depth 12.5 m on `163032`, 84 % beyond 9 m) will hurt every method. Triangulation angles are small (median 7.5° per point).
- The simulator renders no motion blur, so conclusions about speed or blur do not transfer to the real ZED.
- Pose drift in EKF2 is slow and global, not frame-to-frame (neighbouring keyframes agree to 2.4 to 3.4 %).
- The frame rate and overlap are high (314 frames over 98 m), which suits chunked feed-forward inference, but consecutive frames
  have small baselines.

## 8. Working on this branch

`reconstruction` is a long-lived branch from `main` ([collaboration.md](../docs/collaboration.md)):

- Local branch note: a plain `git checkout reconstruction` fails in this repo because `reconstruction/` is also a folder name.
  Use `git switch reconstruction` (or `git checkout reconstruction --`).
- Rebase on `main` before opening a PR, merge by small pull requests (at least weekly).
- Keep a change to a shared path (`kinetix/`, `pyproject.toml`, `uv.lock`, `docker/`, data-format, interfaces) in its own PR.
  Format changes (a new field in `run.yaml` or `frames.jsonl`) go into data-format.md §3 first, in a small PR to `main`.
- Do not claim novelty from this work without the novelty sweep (CLAUDE.md). The relevant notes on feed-forward models in
  planning are in
  [foundation_models_in_planning.md](../docs/paper/research/research_notes/kinetix-novelty-search/foundation_models_in_planning.md).
  The benchmark already lists "Depth Anything 3 or VGGT multi-view depth with drone-pose scale" as an optional ablation
  ([benchmark.md §4](../docs/paper/benchmark.md)).

## Further reading

| Topic | File |
|---|---|
| Offline pipeline | [docs/reconstruction/pipeline.md](../docs/reconstruction/pipeline.md) |
| Todo for this area | [docs/reconstruction/todo.md](../docs/reconstruction/todo.md) |
| Run and result layouts | [docs/data-format.md](../docs/data-format.md) |
| Metrics and benchmark protocol | [docs/paper/benchmark.md](../docs/paper/benchmark.md) |
| Frames and conventions | [docs/interfaces.md](../docs/interfaces.md) §1 |
| Why sensing modes are split | [docs/architecture.md](../docs/architecture.md) §2a |
| Room scoping and prior work | [room-photogrammetry-scoping-review.md](../docs/paper/research/reports/room-photogrammetry-scoping-review.md) |
