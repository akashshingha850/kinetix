# Todo

Current phase: **K1**, with a room-capture track run ahead of it (ADR-K11, ADR-K12). Phase definitions and exit tests:
[roadmap.md](roadmap.md). The evidence for each done item is in the ADR it names and in its run directory.

## Now — room photogrammetry (Warehouse, `config/scenes/warehouse_room.yaml`)
How to run it: `METHOD=tour scripts/room_scan.sh all` (sim up, fly, `kinetix sfm`), or [architecture.md](architecture.md) §5a.
- [x] **Pose + depth per frame (2026-10-27)**: `rig.save_depth` (on for room scenes, `config/room.yaml`) saves the ZED `depth_registered` of every
  keyframe to `runs/<run>/depth/<idx>.npy` (float32 m, NaN/inf = no return, matched to the image by stamp) and records it in
  `frames.jsonl` (`depth`, `depth_source: zed`); recorded only, never a planner input (interfaces.md §4, data-format.md §3). Old `runs/` purged
  (26 runs, 38 GB). Tour `runs/20261027-001233_warehouse_room_tour_mavros_s0`: 314 keyframes, 98 m, 313 with depth (frame 0 got none: fixed
  after the flight, the first keyframe now waits for a depth message; not re-flown), 89 % valid depth pixels, median range 9.2 m;
  SfM 314/314 registered, 21.9 k points, reproj 0.64 px, ATE 14.5 cm vs GT (median 8.4); EKF2 vs GT 0.45 m RMS (yaw mean 1.6 deg);
  depth of neighbouring keyframes agrees to 2.4 % (3 apart) / 3.4 % (10 apart) under EKF2 poses, the same as under GT poses: relative poses are
  good, the error is slow global drift. Depth-aided dense fusion / scale is the open next step (SfM does not use the depth yet).
- [x] **Emptied Warehouse (2026-10-22)**: bisg now strips the room to floor + stripes, walls, columns, ceiling + roof beams (no racks,
  cart, bins, fuse box, lamps, lattice trusses). Scene: `no_fly: []`, workspace out to the west wall margin (x size 15.6 -> 16.34).
  Tour `runs/20261022-123944_warehouse_room_tour_mavros_s0`: 26 viewpoints, 97 m, 368/368 registered, ATE 9.9 cm vs GT, 26.5 k points,
  completeness proxy 0.35 (0.55 in the full room: the empty room has less texture to triangulate), EKF2 vs GT 0.38 m RMS.
  The default `roi_coverage` (stop-and-shoot) crashed into a wall in the same room on bisg's then GEN_3 VIO (2-4 m drift over a long
  flight; bisg sim ZED is GEN_1 since): use `METHOD=tour` for rooms, `scripts/room_scan.sh` still defaults to `roi_coverage`.
- [ ] Dense reconstruction (K2 `offline/densify.py`: COLMAP MVS or gsplat) for a real completeness figure; `completeness_proxy`
  from the sparse cloud is a lower bound (0.55 on the 2026-10-18 tour)
- [ ] Tour: re-plan online from the keyframes' real coverage (dry 0.91 vs flown 0.79: the drone lags the carrot in turns)
- [ ] Tour: try cruise 1.5 m/s and size it against motion blur on the real ZED (exposure × speed / GSD)
- [x] Tour on an unknown room (2026-10-24, dry rig): `--explore` maps the ZED depth at the EKF2 pose, fits the room box + obstacles and tours that (ADR-K13, architecture §5b).
  Warehouse (dry rig, no drift): 27 views / 124 m, walls within 0.04 m. **Flown twice on the sim (2026-10-24): not reliable.** EKF2 drifts 0.2-1 m in the
  first 5 min, the fit came out 1.3-1.6 m off with 15-30 false obstacles, and the second run's tour crashed the drone (EKF2 diverged at minute 14).
  Guards added (stall / divergence abort, 240 s exploration budget, no tour on an unfitted room). Pose fixed on the bisg side 2026-10-26 (Manhattan
  compass + ZED velocity fusion, ADR-K13): EKF2 max 0.62-0.98 m vs 1.50-2.73 m, worst wall 0.66-1.04 m vs 2.41-4.11 m over two flights each; still no tour
  (20-view budget, large-room limit below; walls ~1 m off from the sim ZED's far depth)
- [x] Explore views are keyframes (2026-10-27): they go to `frames.jsonl` for `kinetix sfm` and count as coverage before the tour is planned, so the
  tour flies the gaps (Warehouse dry: 16 viewpoints / 73 m / est. 83 s vs 24 / 101 m / 116 s). Not flown yet.
  Benchmark arms: `explore.reuse_views=true` (default, optimized) vs `false` (traditional: explore maps, the tour scans from scratch;
  dry 24 viewpoints / 100 m). To do: fly both on the sim and compare total flight time, SfM registration, ATE, completeness
- [ ] Explore: continuous forward legs instead of stop-and-shoot (16 s a view live), stop once the walls fit
- [ ] Tour: camera within ~±45 deg of the direction of flight (the ZED is the only obstacle sensor, forward only); scan walls crabbing at 30-45 deg
- [x] Minimal pass (2026-10-26): the code was already close to its minimum (3.1k lines of Python, 89-100 % executed by the unit tests + dry runs, apart from `sfm.py`
  and `rig/mavros.py`, which need pycolmap / ROS). Removed the dead code that was left (`yaw_of_R`, `FACES`, three unused locals) and the unused `scipy` dependency
  (pyproject + uv.lock, nothing else locked depends on it); deleted the 131 GB rosbag of the stuck 20261022-092516 run (runs/ 161 -> 31 GB). Output unchanged: 54 tests
  and every method x scene dry run (7 runs, frames + steps + explore report) are byte-identical to before. Kept on purpose: `torch`/`open3d`/`trimesh`/`transformers`
  (not imported yet, planned for K2+; the Docker image already skips them), the `RigCaps` stereo/depth fields (ADR-K07 sensing modes)
- [x] NBV benchmark (2026-10-26, `tests/nbv_bench.py`): prediction vs flown outcome, baselines + oracle, parameter sweep; six fixes (ADR-K13)
- [ ] Explore, large rooms: no strategy closes the Warehouse in 20 views (level camera, 8 m range, 9 m ceiling); a camera pitch-up sweep or a larger budget
- [ ] Explore: validate on a flight (depth latency while moving is irrelevant, stop-and-shoot), then tune `explore.depth_range_m` / `clearance` / `closed_target` on real ZED depth; dense room with obstacles (the emptied Warehouse has none)
- [ ] Long flights drift (bisg VIO: 2.4 m over 416 m; a 108 m tour stays at 0.19 m RMS). Since 2026-10-26 the sim holds its heading
  (bisg docs/vio-drift.md: a 359 m flight within 1.76 m, 0.93 m RMS); position still drifts ~0.4 % of distance flown: keep tours short
- [ ] Sim ZED depth reads ~10 % short at 8-15 m (ADR-K13 open): measure it (zed_depth_box at range) before trusting far walls
- [ ] Paper scope: C1-C5 are written for objects; decide whether rooms become the target or a second scene class (ADR-K11, ADR-K12)
- [ ] `--rig mavros` start once aborted "MAVROS not connected after 30 s sim" with the topics visible (DDS discovery; retry passed),
  and the process segfaulted (rc 139) on that exit

## Next (K1)
- [ ] Review ADR-K10 (pools with SDK depth via the twin); if accepted, `render_server.py --zed-depth` builds on bisg's
  `docker/sim/tools/zed_pool_probe.py` pattern
- [ ] Review the ROS 2 pipeline proposal (`docs/ros-pipeline.md`, ADR-K09). On acceptance, move its §3/§4 tables into interfaces.md §4/§5
- [ ] Review the doc set; answer Q1 (mount pitch), Q2 (object set), Q6 (T0 stereo source): architecture.md §12
- [ ] Review the contribution wording C1–C5 (`publication-plan.md` §2) and the venue plan (§5)
- [ ] Check asset licences (SimReady, Objaverse++, GSO, OmniObject3D) before fixing the object set
- [ ] Arrange the real-object scanner for K6 GT (Q4), needed by G3
- [ ] Set `KINETIX_DATA` in `.env`; move `config/scenes/` there with scene.usd / gt_mesh (`.env` exists, the key does not)
- [ ] Switch `tool.uv.package = true` (`src/kinetix/` exists)
- [ ] Q3: a bisg change so the sim container can read `$KINETIX_DATA/scenes` (not done: bisg's compose has no such mount)
- [ ] First object scene: pick the object, build `scene.usd` / `gt_mesh.ply` / `scene.yaml` (the bisg sim has no scan objects now)
- [ ] `sim/render_server.py` batch mode + `kinetix pool make`
- [ ] `rig/pool.py`, `recon/state.py` v1 (pycolmap ReconState replaces the coverage proxy online)
- [ ] `tests/fixtures/pool_tiny/` (git-lfs)
- [ ] ZED image stamps are wall clock in the sim (the rig takes the first image *received* after settle)

## Done
- [x] Fast room capture: method `tour` (set cover + TSP + continuous keyframes) and known-pose SfM. Flown 2026-10-18: 2.0 min of drone
  time, 108 m, 403 keyframes, SfM 403/403, ATE 8.2 cm vs GT (`runs/20261018-152242_warehouse_room_tour_mavros_s0`). ADR-K12
- [x] Room scenes + inside-out NBV + `kinetix sfm` + MCAP rosbag per flight. Flown 2026-10-17: 71 images, 28 min wall, SfM 47/71,
  ATE 8.8 cm. ADR-K11
- [x] Camera loss (`no_image`) aborts and lands at once (2026-10-18; before, the session retried 95 s after the ZED wrapper died)
- [x] Sim room = bisg's Warehouse (2026-10-17); `simple_room_blocks` needs bisg `world: Simple Room` and has no blocks there now
- [x] Sim NBV demo on the bisg twin (2026-10-11): object loop, rigs `dry` + `mavros`, 7 images, 96.5 % proxy coverage, camera pose vs
  GT 1.8–3.3 cm
- [x] K0 doc set, Python env, mono-vs-stereo study (ADR-K07), novelty search (C1–C5), references cross-checked (49 DOIs),
  Q6 tested in bisg (ADR-K10), ROS 2 pipeline proposal (ADR-K09) — 2026-10-08..10

## Watch list (check monthly; novelty sweeps at G2, G4, G5)
- ObjView-Bench code + object pool release (arXiv 2605.10707). It may become the default object protocol
- ActMVS code (GitHub TrickyGo/ActMVS): a possible monocular baseline
- Hestia / VIN-NBV code release (object-NBV baselines)
- AREA3D (CVPR 2026, arXiv 2512.05131): backbone and sensors not yet verified
- New arXiv preprints on Isaac Sim + PX4 NBV, monocular/foundation-depth NBV, or mono-vs-stereo active reconstruction
- Room-scale: PredRecon / FC-Planner style coverage planners, on-the-fly UAV SfM, Doppelgangers++ (aliasing in repetitive interiors)
