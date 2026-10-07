# Kinetix architecture

Status: **design, K0** (no runtime code yet). Research framing: [research/idea.md](research/idea.md).
Related docs: [interfaces.md](interfaces.md) (APIs, ROS topics, frames), [data-format.md](data-format.md)
(on-disk layouts), [roadmap.md](roadmap.md) (phases + exit tests), [testing.md](testing.md),
[benchmark.md](benchmark.md), [decisions.md](decisions.md) (ADRs).

## 1. What Kinetix is

A closed loop that captures **few, well-chosen monocular images** of one indoor object with a
drone, then reconstructs the object with a standard photogrammetry pipeline (COLMAP-format SfM, then
MVS or 3D Gaussian Splatting).

```
 seed views ──► capture ──► update online SfM ──► score candidate views ──► pick next view ──┐
                   ▲                                                                       │
                   └───────────────────────────── fly there ◄──────────────────────────────┘
 stop (budget / no gain) ──► offline: full SfM ──► densify ──► evaluate vs ground truth
```

Design goals, in priority order:

1. **One planner, four execution tiers.** The planning code is identical from an offline replay to
   the real drone; only the *rig adapter* changes (§3).
2. **Minimal.** Pure Python on top of pycolmap, Open3D and PyTorch. No ROS in the planner and no
   custom C++. One Docker image, used only where ROS is needed.
3. **Modular.** Every research choice (scorer, sampler, pose source, densifier) is one file behind a
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
│  recon (pycolmap) · offline (densify, eval) · bench                    │  │  │
│  sim/render_server.py  (runs inside the bisg/sim image) ◄──────────────┼──┘  │
└────────────────────────────────────────────────────────────────────────┼─────┘
                       ROS 2 Jazzy, host network, /drone_1/...           │
┌─────────────── bisg_isaac (platform, unchanged API) ───────────────────▼─────┐
│ Isaac Sim + Pegasus + PX4 SITL ── MAVROS ── ZED SDK twin   |   real drone    │
└──────────────────────────────────────────────────────────────────────────────┘
```

**Monocular rule (ADR-K03).** For planning and reconstruction Kinetix reads only the **left ZED image +
its `camera_info`** and the **vehicle pose**. It never reads the right image, ZED depth, point cloud
or ZED mapping. The vehicle's own state estimate (PX4 EKF2, fed by ZED VIO on hardware) is used to
fly and as a metric pose prior. That is the drone's navigation, not a reconstruction input.

## 3. Execution tiers

Same loop, same config, a different `rig:`. Each tier is a superset of the realism of the one before.

| Tier | Rig adapter | Where images come from | Poses | Ground truth | Speed | Used for |
|---|---|---|---|---|---|---|
| **T0** replay | `PoolRig` | pre-rendered **view pool** (discrete candidates) | exact pool poses (+ optional noise) | mesh + GT depth | seconds | algorithm dev, unit/component tests, most benchmark runs |
| **T1** render | `IsaacRenderRig` | Isaac renders a camera teleported to any pose | exact (+ optional noise) | mesh + GT depth | ~1 s/view | continuous candidate spaces, photoreal checks, **generates T0 pools** |
| **T2** sim flight | `MavrosRig` | ZED SDK twin left image in the bisg sim | EKF2 `local_position` | Pegasus `state/pose` + mesh | real time × 0.3–0.5 | dynamics, settle time, flight-time metrics, pose noise |
| **T3** real | `MavrosRig` | real ZED Mini left image | EKF2 (ZED VIO) | external scan of the object (if any) | real time | validation |

T0 is the workhorse: a **view pool** is a dense set of pre-rendered views around an object (e.g. 600
views on 4-DoF reachable poses + a held-out test split). The planner may only pick pool views, so
the next-best-view problem becomes a discrete selection. That is deterministic, cheap and standard in
the NBV literature. T1 (batch mode) produces pools. T0 also works for real data: a dense real capture
becomes a real pool.

## 4. Module map

Target layout (created incrementally per [roadmap.md](roadmap.md); nothing below exists yet except `scripts/`):

```
src/kinetix/
  core/        types.py       Pose, Intrinsics, Frame, Candidate, Scene, RunInfo (dataclasses)
               geometry.py    SE3/Sim3, look-at, frame conversions (ENU map, OpenCV optical, COLMAP)
               config.py      YAML layering + key=value overrides, named-component registry
               runlog.py      run-directory writer/reader (data-format.md)
  rig/         base.py        Rig protocol + RigCaps
               pool.py        T0
               isaac_render.py T1 client (ROS 2)
               mavros.py      T2/T3 (ROS 2, bisg contract)
  recon/       state.py       ReconState: online SfM over pycolmap (+ metric alignment)
               align.py       Sim3 / Umeyama, ATE
               depth_prior.py monocular depth (Depth Anything V2), scale-fit to sparse points
  planner/     loop.py        the session loop (§5) — the only orchestrator
               seed.py        initial views (e.g. 4-6 views on a ring around the ROI)
               sampler.py     candidate poses (pool lookup | view sphere | ring stack)
               constraints.py workspace box, no-fly boxes, standoff, altitude, straight-line path check
               select.py      argmax of gain − λ·travel cost
               stop.py        budget / time / gain plateau
               sequence.py    fixed-sequence planners: orbit, grid (baselines)
               scorers/       random · coverage_oracle · sfm_uncertainty · depth_frontier · combo
  offline/     sfm.py         final SfM over all captured frames (pycolmap)
               densify.py     backends: colmap_mvs (CUDA COLMAP container) | gsplat (+ TSDF mesh)
               metrics.py     accuracy, completeness, Chamfer, F-score, ATE, PSNR/SSIM
               evaluate.py    run dir → metrics.json
  bench/       matrix.py      expand scenes × methods × budgets × seeds → runs
               report.py      tables + efficiency curves from metrics.json files
  cli.py       kinetix run | eval | bench | pool | check
sim/
  render_server.py            Isaac standalone script (bisg/sim image): pool batch mode + serve mode
  scenes/                     scene build helpers (object USD + room → scene.usd, GT mesh export)
config/        default.yaml, methods/*.yaml, rigs/*.yaml
docker/        Dockerfile (ros:jazzy-ros-base + uv venv), compose.yaml
tests/         unit/ component/ integration/ fixtures/
scripts/       check_env.py
```

**Dependency rule** (enforced by a unit test that parses imports):

```
core  ◄── recon ◄── planner ◄── cli
  ▲         ▲          │
  │         └─ offline ◄── bench
  └── rig (base.py only is imported by planner; adapters are loaded by name)
```

- `planner/` never imports `rclpy`, `rig.mavros`, `rig.isaac_render` or `offline/`.
- Only `rig/mavros.py` and `rig/isaac_render.py` import `rclpy`, lazily. So T0 and every unit test
  run in the host venv with no ROS installed.
- `offline/` reads run directories only. It never imports `planner/`, which keeps evaluation
  independent of the method under test.

## 5. The session loop

```python
def run_session(cfg, rig, recon, planner, log):
    rig.start()                                   # T2/T3: arm, take off, OFFBOARD, go to standby pose
    for pose in planner.seed(cfg.scene):          # a few views to bootstrap SfM
        log.frame(frame := rig.capture_at(pose)); recon.add(frame)
    while not planner.stop(recon, log.budget):
        cands = planner.sample(cfg.scene, recon, rig.caps)          # list[Candidate]
        cands = planner.constraints(cands, rig.pose(), cfg.scene)   # feasible + reachable only
        gains = planner.scorer.score(cands, recon)                  # np.ndarray, one per candidate
        pick  = planner.select(cands, gains, rig.pose())            # gain − λ·travel
        log.step(cands, gains, pick)
        log.frame(frame := rig.capture_at(pick.pose)); recon.add(frame)
    rig.finish()                                  # T2/T3: return + land
    log.close(recon)                              # write sfm/ snapshot, run.yaml summary
```

- **Stop-and-shoot** in v1: the rig flies to the pose, settles, then grabs one frame. Capturing
  while moving is a K5 optimization.
- **Baselines are configs.** `orbit` / `grid` use `sequence.py` (the planner returns the next pose
  of a fixed list, no scorer); `random` uses the random scorer; `oracle` uses `coverage_oracle`
  (sim only, reads GT, flagged `uses_gt: true` and reported as an upper bound).

## 6. Online reconstruction (`recon/state.py`)

`ReconState` wraps a pycolmap database + `Reconstruction` and answers what the scorers need: the
registered images, sparse points with track length, triangulation angle and reprojection error,
and the observed-surface bounding box.

- **v1 (simple):** on every new frame, extract features, match the new image against all previous
  ones (exhaustive; N ≤ ~150), and re-run incremental mapping. That is a few seconds per step at
  these sizes, and correct by construction.
- **v2 (fast, K5):** `pycolmap.IncrementalMapper` registering only the next image, plus local BA.
- **Metric frame.** Monocular SfM is up to scale. After each update the model is Sim3-aligned to the
  rig poses (`align_reconstructions_via_proj_centers` / Umeyama on camera centres), so scorers,
  constraints and the planner all work in the metric `map` frame.
- **Pose source** (`recon.pose_source`, an ablation axis): `sfm` (pure SfM, aligned after),
  `prior` (rig poses as priors / known-pose triangulation), `gt` (oracle, sim only).

## 7. Next-best-view scoring

Every scorer has the same signature, `score(cands, recon) -> gains`. The research contribution is the
`combo` scorer and its components:

| Scorer | Signal | Needs | Role |
|---|---|---|---|
| `random` | — | — | baseline |
| `coverage_oracle` | unseen GT-surface area visible from the candidate (Open3D raycast on the GT mesh) | GT mesh | upper bound, sim only |
| `sfm_uncertainty` | sparse points visible in the candidate, weighted by poor triangulation angle, short track or high reprojection error; plus angular gaps in the view distribution around the ROI | sparse model | core monocular signal |
| `depth_frontier` | Depth Anything V2 depth per frame, scale-fit to sparse points, fused into a coarse TSDF in the ROI; gain = unknown/frontier voxels visible from the candidate | GPU, sparse model | completeness signal without a depth sensor |
| `combo` | weighted sum of the above (weights in config) | | Kinetix method |

`select` maximises `gain − λ · travel_cost(current → candidate)`, where travel cost is the
straight-line distance, plus a yaw-change term. Candidates are generated **inside the camera's
reachable pose space** (§8), so the planner never proposes a view the drone cannot take.

## 8. Indoor and vehicle constraints

- **Camera DoF.** The ZED Mini is rigidly mounted (bisg: forward-looking, pitch 0°), so a camera
  pose is **4-DoF** (x, y, z, yaw) with pitch fixed by the mount. Samplers, pools and T1 all read
  `camera.mount_pitch_deg`. A pitched-down mount (needed to see top surfaces) is open question Q1.
- **Scene file** (per object, [data-format.md](data-format.md)): `roi` box around the object,
  `workspace` box (room minus margin), `no_fly` boxes, `standoff` [min, max] from the ROI and
  `altitude` [min, max].
- **Feasibility** (`constraints.py`): the pose is inside the workspace and outside the no-fly boxes and
  the ROI standoff shell, *and* the straight segment from the current pose clears all boxes. v1 has
  no general path planner. If a segment is blocked, the candidate is dropped.
- **Lighting/texture adaptation** (exposure control, avoiding specular angles) is K5 scope.

## 9. Offline pipeline (identical for every method)

To keep the comparison fair, the online model is only for planning. Every method's final model is
rebuilt the same way from its captured frames alone:

1. `offline/sfm.py`: pycolmap SfM over all frames (`sfm` or `prior` poses, per config).
2. Sim3-align to GT camera centres (sim) or to the rig poses (real).
3. `offline/densify.py`: `colmap_mvs` (CUDA COLMAP in Docker: the pip pycolmap wheel is CPU-only)
   or `gsplat` (3DGS, then TSDF-fused rendered depth → mesh).
4. `offline/evaluate.py`: crop to the ROI, compare with the **observable** GT surface, write
   `metrics.json` ([benchmark.md](benchmark.md) §3).

## 10. Configuration

One YAML per run, resolved from layers and stored verbatim in the run directory:

```
config/default.yaml  <  config/rigs/<tier>.yaml  <  config/methods/<method>.yaml  <  data/scenes/<scene>/scene.yaml  <  CLI key=value
```

Components are chosen by name (`planner.scorer: combo`, `rig: pool`, `offline.densify: gsplat`).
Each key lives in exactly one default layer. Machine-specific paths (data root, GPU index) go in
`.env` (git-ignored), never in YAML.

## 11. Runtime placement

| Piece | Runs in | Why |
|---|---|---|
| planner + recon + offline + bench, T0 | host venv (`uv`) or the kinetix image | no ROS needed |
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
| Q5 | Is the drone pose (EKF2 + ZED VIO) allowed as a prior in the paper's "monocular" claim? Default: yes, reported as an ablation (`pose_source: sfm` vs `prior`). | paper |
