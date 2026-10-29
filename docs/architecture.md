# Kinetix architecture

Status: **K1** (object demo loop and room capture run on the bisg twin; pools, ReconState and the K3 scorers are not built). Research framing: [paper/research/idea.md](paper/research/idea.md).
Related docs: [interfaces.md](interfaces.md) (APIs, ROS topics, frames), [data-format.md](data-format.md)
(on-disk layouts), [roadmap.md](roadmap.md) (phases + exit tests), [testing.md](testing.md),
[paper/benchmark.md](paper/benchmark.md), [decisions.md](decisions.md) (ADRs).

## 1. What Kinetix is

A closed loop that captures **few, well-chosen images** of one indoor object with a drone, then
reconstructs the object with a standard photogrammetry pipeline (COLMAP-format SfM, then MVS or 3D
Gaussian Splatting). The primary method is **monocular**. Because the drone carries a ZED Mini, the
same loop also runs with **stereo** inputs as a measured comparison (§2a,
[paper/research/mono-vs-stereo.md](paper/research/mono-vs-stereo.md)).

A scene is either an **object** (cameras outside, around one ROI box) or a **room** (cameras inside; the room's walls, floor,
ceiling and obstacles are the target; ADR-K11). Rooms use a separate capture method that plans the whole flight first and flies
it without stopping ([acquisition/architecture.md](acquisition/architecture.md) §5a, ADR-K12).

```
 seed views ──► capture ──► update online SfM ──► score candidate views ──► pick next view ──┐
                   ▲                                                                       │
                   └───────────────────────────── fly there ◄──────────────────────────────┘
 stop (budget / no gain) ──► offline: full SfM ──► densify ──► evaluate vs ground truth
```

Design goals, in priority order (the paper contributions they serve are in [paper/publication-plan.md](paper/publication-plan.md)):

1. **One planner, four execution tiers.** The planning code is identical from an offline replay to
   the real drone; only the *rig adapter* changes (§3).
2. **Minimal.** Pure Python on top of pycolmap, Open3D and PyTorch. No ROS in the planner and no
   custom C++. One Docker image, used only where ROS is needed.
3. **Modular.** Every research choice (scorer, sampler, pose source, depth source, densifier) is one file behind a
   small protocol, selected by name from config. Each baseline is a configuration of the same loop.
4. **Measurable.** Every run writes one self-describing run directory, so evaluation and
   benchmarks are pure functions of it ([data-format.md](data-format.md)).

## 2. System context

Kinetix does **not** build its own simulator or flight stack. It reuses the
`bisg_isaac` digital twin (`~/bisg_isaac`) (Isaac Sim 6.0 + Pegasus + PX4 v1.17 SITL + MAVROS, ROS 2
Jazzy, same drone as the hardware: Jetson Orin NX, Pixracer, ZED Mini). It talks to it **only through
the bisg vehicle interface contract** (`bisg_isaac/docs/interface-contract.md`), so sim and hardware
look identical to Kinetix (ADR-K01).

```
┌──────────────────────────── kinetix (this repo) ─────────────────────────────┐
│  planner loop ── rig adapter ──┬── PoolRig        (T0: offline replay)       │
│       │                        ├── IsaacRenderRig (T1: render on demand) ─┐  │
│       │                        └── MavrosRig      (T2 sim / T3 real) ──┐  │  │
│  mapping · reconstruction (sfm, densify, eval) · bench                 │  │  │
│  sim/render_server.py  (runs inside the bisg/sim image) ◄──────────────┼──┘  │
└────────────────────────────────────────────────────────────────────────┼─────┘
                       ROS 2 Jazzy, host network, /drone_1/...           │
┌─────────────── bisg_isaac (platform, unchanged API) ───────────────────▼─────┐
│ Isaac Sim + Pegasus + PX4 SITL ── MAVROS ── ZED SDK twin   |   real drone    │
└──────────────────────────────────────────────────────────────────────────────┘
```

## 2a. Sensing modes: monocular method, stereo comparison (ADR-K07, supersedes ADR-K03)

The hardware is stereo, and "monocular" is a property of the method. Two **independent** config axes
decide which camera data planning and reconstruction may use:

| Axis | Key | Values |
|---|---|---|
| depth for **planning** | `sensing.plan_depth` | `none` · `mono` (Depth Anything V2, scale-anchored to SfM, the C1 method) · `mono_metric_noanchor` · `mono_stereo_anchor` (ablations of the anchoring) · `stereo` · `gt` (oracle) |
| input to **reconstruction** | `sensing.recon_input` | `left` · `stereo_rig` (L+R as a pycolmap rig) · `stereo_rig+depth` (+ stereo-depth TSDF) |

Presets: `mono` (= Kinetix: mono / left), `mono_noprior` (none / left), `stereo_plan` (stereo / left),
`stereo_full` (stereo / stereo_rig+depth), `oracle_depth` (gt / left). Keeping the axes separate lets
the benchmark tell *planning* value apart from *reconstruction* value (hypothesis H1 in the study).

Where `stereo` depth comes from, per tier: T2/T3 use the ZED SDK `depth/depth_registered` topic (NEURAL), with its
`confidence/confidence_map`. T1 pools can carry the **same SDK depth**. The bisg probe of 2026-10-10 streamed teleported views of
the ZED twin into the real SDK: settled in ≤ 0.54 s, median error 0.35 % at 0.3–2 m and 0.65 % at 2–5 m (bisg
`docs/zed-sdk-sim.md`, ADR-K10). The SDK cannot read stored PNG pairs, so pools rendered without it, and real dense captures
without an SVO, fall back to FoundationStereo (fallback: OpenCV SGBM). The depth study (study §6) measures how far those differ.

The topics a rig may subscribe to are **derived from the sensing mode** (whitelist in
[interfaces.md](interfaces.md) §4, checked by a test). In `mono` that is only the left image, its
`camera_info` and the vehicle pose. In every mode Kinetix never reads ZED odometry, point cloud or mapping.
Navigation is always the drone's own (PX4 EKF2, fed by stereo ZED VIO on hardware). It is used to fly
and, per `recon.pose_source`, as a metric prior.

## 3. Execution tiers

Same loop, same config, a different `rig:`. Each tier is a superset of the realism of the one before.

| Tier | Rig adapter | Where images come from | Poses | Ground truth | Speed | Used for |
|---|---|---|---|---|---|---|
| **T0** replay | `PoolRig` | pre-rendered **view pool** (discrete candidates) | exact pool poses (+ optional noise) | mesh + GT depth | seconds | algorithm dev, unit/component tests, most benchmark runs |
| **T1** render | `IsaacRenderRig` | Isaac renders a camera teleported to any pose | exact (+ optional noise) | mesh + GT depth | ~1 s/view | continuous candidate spaces, photoreal checks, **generates T0 pools** |
| **T2** sim flight | `MavrosRig` | ZED SDK twin left (+ right, SDK depth in stereo modes) in the bisg sim | EKF2 `local_position` | Pegasus `state/pose` + mesh | real time × 0.3–0.5 | dynamics, settle time, flight-time metrics, pose noise |
| **T3** real | `MavrosRig` | real ZED Mini left (+ right, SDK depth) | EKF2 (ZED VIO) | external scan of the object (if any) | real time | validation |

T0 is the workhorse: a **view pool** is a dense set of pre-rendered views around an object (e.g. 600
views on 4-DoF reachable poses + a held-out test split). Each view stores the left **and right** image
and GT depth, so one pool serves every sensing mode. The planner may only pick pool views, so
the next-best-view problem becomes a discrete selection. That is deterministic, cheap and standard in
the NBV literature. T1 (batch mode) produces pools. T0 also works for real data: a dense real capture
becomes a real pool.

## 4. Module map

Layout: three top-level packages, no `src/`. `kinetix/` is the small shared library, `acquisition/` (author A) and
`reconstruction/` (author B) are the two areas ([collaboration.md](collaboration.md)); each keeps its tests in its own `tests/`.
Target layout (created incrementally per [roadmap.md](roadmap.md); not built yet: `pool.py`, `isaac_render.py`, `state.py`,
`depth/`, `seed.py`, `sequence.py`, most scorers, `external/`, `densify.py`, `metrics.py`, `evaluate.py`, `bench/`, `sim/`):

```
kinetix/                         shared: both areas import it, it imports neither (cli.py excepted)
  types.py       Pose, Intrinsics, Frame, Candidate, Scene, RunInfo (dataclasses)
  geometry.py    SE3/Sim3, look-at, frame conversions (ENU map, OpenCV optical, COLMAP)
  config.py      YAML layering + key=value overrides; ROOT, DATA, RESULTS, SCENES paths
  runlog.py      run-directory writer/reader (data-format.md)
  align.py       Sim3 / Umeyama, ATE
  coverage.py    surface-coverage model (planner gain, reconstruction's completeness proxy)
  cli.py         kinetix run | sfm (| eval | bench | pool | check)
  scenes/        scene files (*.yaml): what acquisition flies, what reconstruction scores against
  tests/
acquisition/                     author A: decides where to fly and what to capture
  planner/
    loop.py        the session entry point (§5): `run_session` picks the method's loop
    nbv.py         stepwise NBV loop (roi_coverage, random, orbit): select (argmax of gain − λ·travel cost),
                   stop rule (budget / coverage / gain plateau), orbit order
    explore.py     unknown room: occupancy map from ZED depth, frontier views, room fit (§5b)
    tour.py        rooms: set cover, TSP order, dense path and the tour loop `run_tour` (method `tour`, §5a)
    sampler.py     candidate poses (pool lookup | view sphere | ring stack | Tammes sphere | room grid)
    constraints.py workspace box, no-fly boxes, standoff, altitude, straight-line path check
    seed.py · sequence.py (orbit, grid baselines)
    scorers/       random · roi_coverage · fvs · volumetric_ig · coverage_oracle · sfm_uncertainty · depth_frontier · combo
    external/      wrappers for third-party pool baselines: fisherrf.py, ma_scvp.py (their code runs in its own env)
  mapping/         online models: depthmap.py (occupancy), register.py (drift), roomfit.py, state.py (online SfM),
                   depth/ (DepthSource per frame: mono / stereo_learned / stereo_zed / gt)
  rig/             base.py (Rig protocol + RigCaps), dry.py + simdepth.py, mavros.py (T2/T3), pool.py (T0), isaac_render.py (T1)
  config/          default.yaml, room.yaml (methods/*.yaml, rigs/*.yaml later)
  tests/           unit tests + nbv_bench.py
reconstruction/                  author B: turns a flight into a model and scores it
  sfm.py         final SfM over all captured frames (pycolmap; video/covis pairing, rig check, known-pose mode)
  densify.py     backends: colmap_mvs (CUDA COLMAP container) | gsplat (+ TSDF mesh)
  metrics.py     accuracy, completeness, Chamfer, F-score, ATE, PSNR/SSIM
  evaluate.py    run dir → metrics.json
  tests/
data/            flights, one run dir each (git-ignored, data-format.md §3); written by acquisition
results/         results/<run>/<method>/ (git-ignored); written by reconstruction
bench/           matrix.py (scenes × methods × budgets × seeds → runs), report.py (tables + efficiency curves)
sim/             render_server.py (Isaac standalone, pool batch + serve mode), scene build helpers
docker/          Dockerfile (ros:jazzy-ros-base), compose.yaml
scripts/         check_env.py, room_scan.sh
```

**Dependency rule** (enforced by `kinetix/tests/test_boundaries.py`, which parses imports):

- `kinetix/` imports neither area; only `kinetix/cli.py` dispatches to both.
- `acquisition/planner/` never imports `rclpy`, `rig.mavros`, `rig.isaac_render` or `reconstruction/`.
- Only `acquisition/rig/mavros.py` and `acquisition/rig/isaac_render.py` import `rclpy`, lazily. So T0 and every unit test
  run in the host venv with no ROS installed.
- `reconstruction/` reads run directories only. It never imports `acquisition/`, which keeps evaluation
  independent of the method under test.

## 5–9. Acquisition and reconstruction

The session loop (§5), room capture with `tour` (§5a), exploration of unknown rooms (§5b), online reconstruction (§6), NBV scoring
(§7) and the indoor constraints (§8) are in [acquisition/architecture.md](acquisition/architecture.md), under the same section
numbers. The offline pipeline (§9) is [reconstruction/pipeline.md](reconstruction/pipeline.md).

## 10. Configuration

One YAML per run, resolved from layers and stored verbatim in the run directory:

```
acquisition/config/default.yaml  <  acquisition/config/rigs/<tier>.yaml  <  acquisition/config/methods/<method>.yaml  <  data/scenes/<scene>/scene.yaml  <  CLI key=value
```

Components are chosen by name (`planner.scorer: combo`, `rig: pool`, `reconstruction.densify: gsplat`).
Each key lives in exactly one default layer. Machine-specific paths (data root, GPU index) go in
`.env` (git-ignored), never in YAML.

## 11. Runtime placement

| Piece | Runs in | Why |
|---|---|---|
| acquisition + reconstruction + bench, T0 | host venv (`uv`) or the kinetix image | no ROS needed |
| T1/T2/T3 sessions | `kinetix` image: `ros:jazzy-ros-base` + the same uv venv (`--system-site-packages` for `rclpy`; Jazzy is Python 3.12 like the host) | needs `rclpy` |
| `sim/render_server.py` | `bisg/sim:6.0.0` (Isaac's Python) | needs Isaac |
| sim + PX4 + MAVROS | bisg_isaac compose (`./bisg all headless`) | platform |
| COLMAP MVS | `colmap/colmap` CUDA image | pycolmap wheel has no CUDA |

GPU budget on workstation B (2× RTX 6000 Ada): Isaac on GPU 0, depth prior / gsplat / MVS on GPU 1
(`CUDA_VISIBLE_DEVICES` in `.env`). The vLLM service `lc-chat` must be stopped before sim runs (bisg CLAUDE.md).

## 12. Open questions

| # | Question | Blocks |
|---|---|---|
| Q1 | ZED mount pitch for scanning: keep 0° (top surfaces barely seen) or a tilted mount for Kinetix flights? Changes the bisg twin too. | K4 / T3 |
| Q2 | Object set: which USD assets (SimReady / YCB / Objaverse-converted) with clean GT meshes? 3 for K1–K3, 8–10 for the paper. | K1 |
| Q3 | bisg integration: a read-only mount of `$KINETIX_DATA/scenes` into the sim container so `world.usd_path` can point to a Kinetix scene (small bisg compose change, done in bisg with its own docs). | K1 (pool generation) |
| Q4 | Ground truth for real objects (T3): handheld scanner / reference photogrammetry with 300+ images? | K6 |
| Q5 | Is the drone pose (EKF2 + ZED VIO) allowed as a prior in the paper's "monocular" claim? Default: yes, reported as an ablation (`pose_source: sfm` vs `prior`). Navigation is stereo in every mode. | paper |
| Q6 | Stereo depth on T0 pools. **Answered 2026-10-10 (ADR-K10):** pools store real ZED SDK depth rendered through the twin (`zed/`), and FoundationStereo stays as the fallback and as an ablation. Open: whether a 0.6 s settle per view is fast enough for the large Tammes pools (≈ 7 min / 600 views measured). | K1 |
