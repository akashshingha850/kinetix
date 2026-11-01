# Acquisition architecture

How Kinetix decides which images to take: the session loop, room tours, exploration, online reconstruction, view scoring and the
flight constraints. Code in `acquisition/`, todo in [todo.md](todo.md). The system overview, tiers and module map are in
[../architecture.md](../architecture.md); sections keep the numbers they had there (§5–§8), so older references still hold.

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

- **Stop-and-shoot** for objects and the NBV research loop: the rig flies to the pose, settles, then grabs one frame. Rooms
  capture while moving (§5a).
- **Baselines are configs.** `orbit` / `grid` use `sequence.py` (the planner returns the next pose
  of a fixed list, no scorer); `random` uses the random scorer; `oracle` uses `coverage_oracle`
  (sim only, reads GT, flagged `uses_gt: true` and reported as an upper bound).
- **`surface_normal`** (objects; `planner/surface.py`, the benchmark's `explore_exploit`, after Liu et al. 2026, related_work [50]):
  a coarse orbit (`planner.surface.coarse_*`, 8 views at 1.6 m) whose ZED depth (`rig.save_depth`, set by the CLI) becomes oriented
  points binned into 10 cm patches; one level view per patch at `standoff_m` along its normal; KD-tree grouping (same heading
  side only) down to the rest of the budget; nearest neighbour + 2-opt order with legs through the ROI shell barred; stop-and-go.
  A leg that would cross the object goes round it via `rig.move_to` transits on the coarse ring. Uses ZED depth for planning,
  so it is a stereo-plan baseline, not a mono method.

## 5a. Room capture: method `tour` (ADR-K11, ADR-K12)

Rooms are large, so flight time dominates (stop-and-shoot NBV took 12-17 min of drone time in the Warehouse, 62 % of it in
zig-zag travel, 38 % stopping); images are cheap. `tour` plans before take-off and never stops:

1. **Viewpoints**: `GridSampler` (2 m grid × 3 heights × 12 yaws in the scene's `workspace`) → greedy set cover of the room proxy
   (`kinetix/coverage.py`: room faces inward, obstacles occlude and count, triangulation needs two rays 8-20° apart) up to
   `planner.tour.coverage_target` (0.8).
2. **Order**: open TSP from the take-off pose (nearest neighbour + 2-opt; metres + `yaw_cost_m` per radian of turn).
3. **Fly**: `rig.fly_path` streams a time-parameterised setpoint along the dense path at `rig.cruise_mps` (1.0) with turns paced
   at `yaw_rate_deg_s`, and keeps a keyframe every 0.4 m / 6° of camera motion from the continuous left stream. The flight is
   recorded to `<run>/rosbag/` (MCAP).
4. **Reconstruct offline**: `kinetix sfm <run>` matches each keyframe with its 10 successors and its 8 most co-visible frames,
   checks every verified pair against the rig's relative pose (repeated bays alias), triangulates at the rig (EKF2) poses and
   refines with bundle adjustment (`pose_source: prior`). Pure incremental SfM (`--pose-source sfm`) put whole blocks of a
   tour one 6 m bay off.

2026-10-18 in the Warehouse: 2.0 min of drone time, 108 m, 403 keyframes; SfM 403/403, ATE 8.2 cm vs GT, 55 s of processing.
The room box and obstacles come from the scene file, so this is coverage planning of a known room, not exploration.

```
METHOD=tour scripts/room_scan.sh all          # bisg sim up -> fly -> kinetix sfm (scripts/room_scan.sh sfm [RUN] re-runs SfM)
```

## 5b. Unknown rooms: explore, then tour (ADR-K13)

`--explore` replaces the scene file's room box and obstacles with what the ZED depth shows, then runs `tour` on it, starting from
what exploration already covered (every exploration view is a keyframe, so the tour plans only the gaps):

```
rig.capture_at -> depth (32FC1 m) + rig pose (EKF2 ∘ mount)
   -> OccupancyMap.integrate     (acquisition/mapping/depthmap.py: occupied / free / unknown voxels, 0.2 m)
   -> fit_scene                  (acquisition/mapping/roomfit.py: walls, floor, ceiling, obstacle boxes -> Scene)
   -> next_view                  (acquisition/planner/explore.py: frontier voxels + the fitted room's unseen faces / obstacle sides (face_targets)
                                  in the view cone, line-of-sight re-score, safe parking + corridor)
   -> ... until the room is closed or stable -> run_tour on the fitted Scene, the exploration views credited to its coverage
```
Exploration views go to `frames.jsonl` (idx 900000+, `pool_id` `explore_<n>`, EKF2 pose) like tour keyframes, so `kinetix sfm` uses
them. `explore.reuse_views=false` is the traditional baseline for benchmarking (exploration only maps, the tour scans the whole
room, no exploration frames in `frames.jsonl`): `kinetix run --scene ... --method tour --explore explore.reuse_views=false`; `run.yaml`
records which arm a run was; the run dir is named `..._tour-explore-optimized_...` / `..._tour-explore-traditional_...`. Warehouse,
dry rig: reuse 17 viewpoints / 80 m, traditional 24 / 90 m. Room fit (acquisition/mapping/roomfit.py): floor and ceiling first, then each wall
from the occupied voxels between them, placed on the densest slice within `fit.wall_artifact_m` inward of the outermost one (a
weaker sheet beyond it is that wall mapped again after pose drift: fitting to it left the real wall inside the room as an obstacle).
Outputs in `<run>/explore/`: `scene.yaml` (the fitted room in scene-file format), `occupancy.npz`, `steps.jsonl` (one record per view:
faces seen, known fraction, frontier, target) and `report.json` (with `vs_truth` when a scene file is given). `steps.jsonl` of the
run itself still starts with the tour plan. Dry rig: `acquisition/rig/simdepth.py` ray-casts depth against a true scene the planner never
sees. Config: `explore.*` in `acquisition/config/room.yaml`. From the GCS: the Kinetix tab's "explore it first" box.

## 6. Online reconstruction (`acquisition/mapping/state.py`)

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
- **Stereo rig** (`sensing.recon_input: stereo_rig*`): the right image is added as a second camera of a
  pycolmap `Rig` with the baseline from `camera_info`. SfM is then metric without any pose prior.

## 7. Next-best-view scoring

Every scorer has the same signature, `score(cands, recon) -> gains`. The research contribution is the
`combo` scorer and its components:

| Scorer | Signal | Needs | Role |
|---|---|---|---|
| `random` | — | — | baseline |
| `fvs` | distance to the farthest already-captured view (farthest-view sampling) | poses only | strong simple baseline |
| `volumetric_ig` | entropy of unobserved / rear-side voxels in the TSDF visible from the candidate (Isler/Delmerico family) | a depth source | classical NBV baseline |
| `fisherrf`, `ma_scvp` (external) | 3DGS Fisher information; learned one-shot + NBV view planning | their own code | radiance-field and learned baselines |
| `coverage_oracle` | unseen GT-surface area visible from the candidate (Open3D raycast on the GT mesh) | GT mesh | upper bound, sim only |
| `sfm_uncertainty` | sparse points visible in the candidate, weighted by poor triangulation angle, short track or high reprojection error; plus angular gaps in the view distribution around the ROI | sparse model | core monocular signal |
| `depth_frontier` | per-frame depth from the configured `DepthSource` (`sensing.plan_depth`: mono / stereo / gt), fused into a coarse TSDF in the ROI; gain = unknown/frontier voxels visible from the candidate | a depth source | completeness signal. The same scorer serves mono and stereo, so the comparison changes only the depth |
| `combo` | weighted sum of `sfm_uncertainty` and `depth_frontier` (weights in config) | | Kinetix method (C1) |

`select` maximises `gain − λ · travel_cost(current → candidate)`, where travel cost is the
straight-line distance, plus a yaw-change term. Candidates are generated **inside the camera's
reachable pose space** (§8), so the planner never proposes a view the drone cannot take.

## 8. Indoor and vehicle constraints

- **Camera DoF.** The ZED Mini is rigidly mounted (bisg: forward-looking, pitch 0°), so a camera
  pose is **4-DoF** (x, y, z, yaw) with pitch fixed by the mount. Samplers, pools and T1 all read
  `camera.mount_pitch_deg`. A pitched-down mount (needed to see top surfaces) is open question Q1.
- **Scene file** (per object, [data-format.md](../data-format.md)): `roi` box around the object,
  `workspace` box (room minus margin), `no_fly` boxes, `standoff` [min, max] from the ROI and
  `altitude` [min, max].
- **Feasibility** (`constraints.py`): the pose is inside the workspace and outside the no-fly boxes and
  the ROI standoff shell, *and* the straight segment from the current pose clears all boxes. v1 has
  no general path planner. If a segment is blocked, the candidate is dropped.
- **Lighting/texture adaptation** (exposure control, avoiding specular angles) is K5 scope.

