# Todo

Current phase: **K1**, with a room-capture track run ahead of it (ADR-K11, ADR-K12). Phase definitions and exit tests:
[roadmap.md](../roadmap.md). The evidence for each done item is in the ADR it names and in its run directory.

## Now — room photogrammetry (Warehouse, `kinetix/scenes/warehouse_room.yaml`)
How to run it: `METHOD=tour scripts/room_scan.sh all` (sim up, fly, `kinetix sfm`), or [architecture.md](architecture.md) §5a.
- [x] **Pose + depth per frame (2026-10-27)**: `rig.save_depth` (on for room scenes, `acquisition/config/room.yaml`) saves the ZED `depth_registered` of every
  keyframe to `data/<run>/depth/<idx>.npy` (float32 m, NaN/inf = no return, matched to the image by stamp) and records it in
  `frames.jsonl` (`depth`, `depth_source: zed`); recorded only, never a planner input (interfaces.md §4, data-format.md §3). Old `data/` purged
  (26 runs, 38 GB). Tour `data/20261027-001233_warehouse_room_tour_mavros_s0`: 314 keyframes, 98 m, 313 with depth (frame 0 got none: fixed
  after the flight, the first keyframe now waits for a depth message; not re-flown), 89 % valid depth pixels, median range 9.2 m;
  SfM 314/314 registered, 21.9 k points, reproj 0.64 px, ATE 14.5 cm vs GT (median 8.4); EKF2 vs GT 0.45 m RMS (yaw mean 1.6 deg);
  depth of neighbouring keyframes agrees to 2.4 % (3 apart) / 3.4 % (10 apart) under EKF2 poses, the same as under GT poses: relative poses are
  good, the error is slow global drift. Depth-aided dense fusion / scale is the open next step (SfM does not use the depth yet).
- [x] **Emptied Warehouse (2026-10-22)**: bisg now strips the room to floor + stripes, walls, columns, ceiling + roof beams (no racks,
  cart, bins, fuse box, lamps, lattice trusses). Scene: `no_fly: []`, workspace out to the west wall margin (x size 15.6 -> 16.34).
  Tour `data/20261022-123944_warehouse_room_tour_mavros_s0`: 26 viewpoints, 97 m, 368/368 registered, ATE 9.9 cm vs GT, 26.5 k points,
  completeness proxy 0.35 (0.55 in the full room: the empty room has less texture to triangulate), EKF2 vs GT 0.38 m RMS.
  The default `roi_coverage` (stop-and-shoot) crashed into a wall in the same room on bisg's then GEN_3 VIO (2-4 m drift over a long
  flight; bisg sim ZED is GEN_1 since): use `METHOD=tour` for rooms, `scripts/room_scan.sh` still defaults to `roi_coverage`.
- Reconstruction (SfM fixes, dense, GT mesh, depth-aided) moved to [reconstruction/todo.md](../reconstruction/todo.md) (2026-10-28,
  [collaboration.md](../collaboration.md)); this file is acquisition (explore, planning, flight)
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
  dry 24 viewpoints / 90 m, optimized 17 / 80 m since the wall fix below). To do: fly both on the sim and compare total flight time, SfM registration, ATE, completeness
  Run dirs are named `..._tour-explore-optimized_...` / `..._tour-explore-traditional_...`; the MRGCS Kinetix tab has the arm
  selector next to *explore it first* (2026-10-27)
- [x] Room fit: a wall mapped twice (2026-10-27). Flight `20261027-144459`: the east wall also appeared 1.4 m beyond itself (pose drift);
  the fit took that copy, the room came out 0.65-1.7 m too big and the real walls became 2.6-2.8 m thick room-long obstacles (8 boxes).
  Walls are now fitted from the voxels between floor and ceiling, on the densest slice within 2 m of the outermost: the same map refits
  to walls within 0.06-0.16 m of truth and 4 boxes (two full-height posts near the walls, two roof beams/lamps at 8 m; not checked
  against the USD). On `20261027-103147` (2.9 m drift, y- wall 5 % seen) the y- wall comes out 4 m inside the truth: a smaller room
  is the safe side, the map is unusable either way. Not flown yet
- [x] Explore flight `20261027-164441` (2026-10-27, fresh sim, optimized arm, via the GCS bridge): explore_incomplete, landed, no tour
  (correct: the y+ face was a guess). The camera stayed within y -2..7 of the 30 m hall, the 8 m depth range reached y 15.1, so the end
  walls (y -12, 17.8) were never seen; x walls within 0.2 m, y- 0.6 m; EKF drift estimate 0.05 m. 5 obstacles, none real (the USD has
  nothing in the flight band but 0.2-0.3 m wall columns): a 1.4 x 6 m full-height slab 0.25-1.65 m in front of the east wall = the wall
  drawn with a transient EKF pose error (frame 900009: yaw +5.5 deg, 0.56 m right after a 95 deg turn + move; 1446 of its points; the
  same rays at the GT pose land on the wall), two crane-rail bracket beams (z 6.2-6.4 m, real, above the flight band), one floor blob.
  EKF yaw vs GT per frame: 0.5 -> 2.6 deg over the take-off sweep, 4-5.5 deg spikes after big turns/moves (frames 9, 24-26), back
  under 1 deg within ~10 s
- [x] Explore speed and wall markers (2026-10-27, Warehouse, fresh sim each, optimized arm, one flight per setting):
  | run | setting | explore pose err, worst yaw / xy | room fit, worst wall | false obstacles | outcome |
  | `164441` | 1 m/s, 60 deg/s yaw steps, settle 0.2 s | 5.5 deg / 0.59 m | 4.29 m (y+ never seen) | 5 | no tour |
  | `170418` | 0.5 m/s, 20 deg/s paced, settle 1 s | 6.1 deg / 0.30 m | 3.05 m (y+ never seen) | 0 | tour, coverage 0.94 |
  | `171625` | slow + 52 wall markers (bisg sim.yaml) | 7.1 deg / 0.87 m | 0.59 m | 1 (+2 real corner pieces) | tour, coverage 0.946, 180 m |
  Slow is the explore default now (explore.rig); every turn is paced at rig.yaw_rate_deg_s (mavros rig `_yaw_carrot`). The markers
  did not make the pose better (yaw spikes 6-7 deg after turns, xy 0.5-0.9 m over the longer flight, ~0.5 % of distance as before);
  the better fit came from exploration reaching both ends (y -5.4..9.6). One flight per setting: noisy. Tour keyframes show a steady
  -3.5 deg 'yaw error' while turning at 50 deg/s: a pose vs GT stamp offset (~70 ms), not EKF error
- [x] Wall markers removed again (no pose gain); bisg compass v4 (2026-10-27, bisg docs/vio-drift.md): the walls below a high camera
  count, 1/z, sharpness 2.5, median of 5. Explore flight `20261027-182638`: heading vs GT 0.7 / 2.0 deg (median / max) vs 1.1 / 6.1,
  walls within 0.27 m, 0 obstacles; still no tour: 'floor seen 8 %' (fitted 0.23 m low: the ZED reads 4-9 % long, from 4 m up the floor
  smears) -> roomfit puts the floor on its densest slice within floor_margin_m (refit: 68 %), obstacles still start from the lowest
  slice. With that the same map would still stop on y+ seen 9.2 % (< tour_min_wall 0.10): the camera reached y -4..6.9 of the 30 m hall
- [x] Explore faster (2026-10-28, `explore.rig`): 1.0 m/s + 30 deg/s (was 0.5 + 20; settle 1 s kept), on bisg's magnetometer yaw +
  range height. Flight `20261028-103323` (fresh sim) vs `100033` (old pace): explore 367 vs 520 s wall, worst wall 0.19 vs 0.41 m,
  0 false obstacles both. Pose at the 28 explore views: xy p50/p90/max 0.18/0.40/0.42 vs 0.17/0.39/0.39 m, yaw 0.20/0.54/0.75 vs
  0.14/0.35/0.38 deg; jitter (EKF - GT minus its 2 s mean) xy 5-6 vs 3 mm, z 6 vs 3 mm, yaw 0.48 vs 0.12 deg std (mostly while
  turning: the ~70 ms pose/GT stamp offset); slow drift unchanged. Outcome: no tour (y+ seen 10 %, < tour_min_wall), one flight
  only. z there -0.02 vs +0.10 m: the +0.1 m was bisg's take-off height bias (bisg docs/todo.md, `EKF2_HGT_REF 2`), not speed.
  bisg back on baro height (`EKF2_HGT_REF 0`), two more flights on one boot (`104849`, `105507`): xy 0.09/0.16/0.23 and
  0.15/0.29/0.36 m, z p90 0.15 / 0.11 m (no start-up ramp, mid-flight wander), yaw max 0.8 / 1.7 deg; again no tour (y+ seen
  4 % / 7 %; walls 4.48 / 0.29 m). New pace 0/3 tours vs `100033` 1/1: the 20-view large-room limit below, or the pace; not separated
- [x] Explore view score knows the room (2026-10-28, ADR-K13 amendment): `explore.face_weight` 25 per m^2 of the fitted room's unseen
  faces / obstacle sides / small blobs in view (`face_targets`), no closed/stable stop while a blob is pending, and the fit's obstacle
  band starts at most one slice below the fitted floor (a floor smeared 0.6 m deep became an 18 x 25 m obstacle). nbv_bench (dry):
  Warehouse 8 starts, weakest wall/floor median 0.20 -> 0.56, gate 6/8 -> 7/8, 100 -> 78 m; small rooms: same obstacles, room12 16.6 -> 8.8 m,
  partition 18 -> 34 m from one start. Sim, same pace and boot: `121155` weakest wall 0.29 (fit then still made the floor an
  obstacle), `123651` 0.54 (x- 0.76, x+ 0.69, y- 0.66, floor 0.72), walls 0.37 m, the tour flew both times (0/3 before).
  Open: legs are 8-12 of the 20 steps on the sim (5-10 before): the targets pull past the known-free corridor (legs could stop
  counting against `max_steps`, `max_time_s` still bounds it); `123651` tour 89 viewpoints / 162 m, coverage 0.79 (`100033`: 21 / 104 m,
  0.94): explore views credit 0.22 of coverage (0.32) and the fit has a 13 x 4 m box at z 3.8-8.6 near the y- wall (portal frame or
  noise? not checked against the USD); tour xy p90 0.93 m over the 14.5 min flight.
  `kinetix sfm` on `123651`: 642/642 registered, 42.8 k points, reproj 0.75 px, 70 s; ATE vs GT median 0.12 m but RMSE 0.73 / max 17 m from
  two explore views with 4-5 points each (900012, 900018) that bundle adjustment moved off their EKF2 poses (0.35 m right); without them
  RMSE 0.16 / median 0.10 / p90 0.23 m (EKF2 0.74 m RMS). Completeness proxy 0.51 (walls 0.44, floor 0.52, ceiling 0.63; the 2026-10-22
  emptied-room tour 0.35). To do in `sfm.py`: hold cameras with too few points at their prior pose in BA (or drop them)
  Flight `134802` (MRGCS, same settings): explore ran out its 20 steps (28 views), walls within 0.22 m, ceiling seen 8 %; tour 33 viewpoints /
  114 m planned, coverage 0.85, 389 keyframes, 784 s in all (`123651`: 642, coverage 0.79). SfM 389/389, 27.6 k points, 0.68 px;
  tour cameras 0.11 m median / 0.23 max vs GT after Sim3, but RMSE 0.60 / max 9.7 m from three explore views with 2-8 points (900016/25/03,
  the bug above; without them 0.14 m). EKF2 vs GT 0.57 m RMS, y wandering +0.39..-0.71 m. Findings: (1) the blob gate never clears: 28 of
  the 33 blobs are roof beams (z >= 7.8) or floor stripes (z <= 0.6) no view resolves, so explore always ends on `max_steps` - blobs need
  the obstacles' `obstacle_reach_m` filter; (2) the boxes at y -9.3, z 3.3-8.7 (here 3, `123651`'s 13 x 4 m one) are FALSE: frame
  900024 (y -4, facing the y- wall 8.0 m away) shows nothing in between, its ZED depth reads 7.0-7.4 m on the white brick and 3.8-6.5 m
  (half invalid) on the plain orange panels; (3) sparse completeness is texture-bound: 0.25 m cells with a point, x walls 0.18/0.23, end
  walls 0.00/0.03 (plain panels), floor 0.14, ceiling 0.25; 16.5 % of points lie > 0.25 m outside the true room box (median 0.7 m, most
  beyond x+), not looked into
  Flight `163032` (same settings): tour 267 keyframes / 195 m, coverage 0.92; SfM 267/267, 22.8 k points, 0.60 px, track 6.4; ATE max
  20.6 m from explore view 900004 (7 points; the BA bug again), cameras with >= 30 points RMSE 0.15 / median 0.09 m; EKF2 vs GT 0.34 m.
  Completeness proxy 0.41 (walls 0.33). Acquisition limits: (a) points are far: median depth 12.5 m, 84 % beyond 9 m (2.4 cm/px at
  fx 530), because the coverage model counts surface up to 12 m (`room.yaml` `depth_range_m`); (b) max triangulation angle per point
  median 7.5 deg, 34 % below 5 deg; (c) the sim renders no motion blur (Laplacian variance vs speed r = -0.02, 0.6-1.3 m/s), so a sim
  speed study says nothing about blur
- [ ] Explore, pose after big turns: capture only once the VIO has settled (spikes of 4-5.5 deg / 0.3-0.6 m for seconds), or slower
  turns while exploring; bisg side: why the ZED/EKF yaw jumps after a turn (map_correction / compass in the zed bridge)
- [x] Room fit: obstacles must reach within `fit.obstacle_reach_m` (1 m) of the workspace (2026-10-27): roof beams at 8 m dropped
- [x] Planner split by method (2026-10-27): `loop.py` = entry point only, `nbv.py` = stepwise NBV (with the former `select.py` +
  `stop.py`), `tour.py` = tour planning + `run_tour`. Every method x scene dry run byte-identical before/after, 55 tests
- [ ] Explore: continuous forward legs instead of stop-and-shoot (16 s a view live), stop once the walls fit
- [ ] Tour: camera within ~±45 deg of the direction of flight (the ZED is the only obstacle sensor, forward only); scan walls crabbing at 30-45 deg
- [x] Minimal pass (2026-10-26): the code was already close to its minimum (3.1k lines of Python, 89-100 % executed by the unit tests + dry runs, apart from `sfm.py`
  and `acquisition/rig/mavros.py`, which need pycolmap / ROS). Removed the dead code that was left (`yaw_of_R`, `FACES`, three unused locals) and the unused `scipy` dependency
  (pyproject + uv.lock, nothing else locked depends on it); deleted the 131 GB rosbag of the stuck 20261022-092516 run (data/ 161 -> 31 GB). Output unchanged: 54 tests
  and every method x scene dry run (7 runs, frames + steps + explore report) are byte-identical to before. Kept on purpose: `torch`/`open3d`/`trimesh`/`transformers`
  (not imported yet, planned for K2+; the Docker image already skips them), the `RigCaps` stereo/depth fields (ADR-K07 sensing modes)
- [x] NBV benchmark (2026-10-26, `acquisition/tests/nbv_bench.py`): prediction vs flown outcome, baselines + oracle, parameter sweep; six fixes (ADR-K13)
- [ ] Explore, large rooms: no strategy closes the Warehouse in 20 views (level camera, 8 m range, 9 m ceiling); a camera pitch-up sweep or a larger budget
- [ ] Explore: validate on a flight (depth latency while moving is irrelevant, stop-and-shoot), then tune `explore.depth_range_m` / `clearance` / `closed_target` on real ZED depth; dense room with obstacles (the emptied Warehouse has none)
- [ ] Long flights drift (bisg VIO: 2.4 m over 416 m; a 108 m tour stays at 0.19 m RMS). Since 2026-10-26 the sim holds its heading
  (bisg docs/vio-drift.md: a 359 m flight within 1.76 m, 0.93 m RMS); position still drifts ~0.4 % of distance flown: keep tours short
- [x] Sim ZED depth vs range, measured 2026-10-27 (run `20261027-164441`, every 8th pixel of 28 views vs the ray's true distance to the
  room box at the GT camera pose): reads LONG up to 10 m (+9 % at 1-3 m, +4 % at 5-7 m, +2 % at 8 m), short beyond (-5 % at 12-15 m,
  -11 % at 15-16 m). Within explore's 8 m range walls land slightly outward, not inward
- [ ] Paper scope: C1-C5 are written for objects; decide whether rooms become the target or a second scene class (ADR-K11, ADR-K12).
  Room scoping review 2026-10-28: `paper/research/reports/room-photogrammetry-scoping-review.md` (closest prior work: ICON drone; open: acquisition
  geometry vs accuracy under VIO drift, fewest images / shortest flight at a stated accuracy, stereo depth for planning vs reconstruction)
- [ ] `--rig mavros` start once aborted "MAVROS not connected after 30 s sim" with the topics visible (DDS discovery; retry passed),
  and the process segfaulted (rc 139) on that exit

## Next (K1)
- [ ] Review ADR-K10 (pools with SDK depth via the twin); if accepted, `render_server.py --zed-depth` builds on bisg's
  `docker/sim/tools/zed_pool_probe.py` pattern
- [ ] Review the ROS 2 pipeline proposal (`docs/acquisition/ros-pipeline.md`, ADR-K09). On acceptance, move its §3/§4 tables into interfaces.md §4/§5
- [ ] Review the doc set; answer Q1 (mount pitch), Q2 (object set), Q6 (T0 stereo source): architecture.md §12
- [ ] Review the contribution wording C1–C5 (`paper/publication-plan.md` §2) and the venue plan (§5)
- [ ] Check asset licences (SimReady, Objaverse++, GSO, OmniObject3D) before fixing the object set
- [ ] Arrange the real-object scanner for K6 GT (Q4), needed by G3
- [ ] Set `KINETIX_DATA` in `.env`; move `kinetix/scenes/` there with scene.usd / gt_mesh (`.env` exists, the key does not)
- [x] Switch `tool.uv.package = true` (2026-10-28, hatchling backend): `uv run pytest` 59 passed, `uv run python -m kinetix` works
- [x] Flat two-author layout (2026-10-29): `kinetix/` shared lib (+ `scenes/`), `acquisition/` (planner, mapping = old `recon/`, rig,
  config), `reconstruction/` (old `offline/`), flights `data/` (was `runs/`), outputs `results/<run>/<method>/` (was `<run>/sfm/offline/`),
  tests next to the code; `kinetix/tests/test_boundaries.py` replaces the two import tests (which passed vacuously on a missing folder).
  MRGCS `kinetix_bridge.py` follows. 58 passed (uv and the kinetix image). docs/ follows the same split: shared docs at the top,
  `acquisition/` (architecture §5-8 split out, ros-pipeline, todo), `reconstruction/` (pipeline = old architecture §9, todo),
  `paper/` (publication-plan, benchmark, research/ with space-free names); 91 relative links checked
- [ ] Q3: a bisg change so the sim container can read `$KINETIX_DATA/scenes` (not done: bisg's compose has no such mount)
- [ ] First object scene: pick the object, build `scene.usd` / `gt_mesh.ply` / `scene.yaml` (the bisg sim has no scan objects now)
- [ ] `sim/render_server.py` batch mode + `kinetix pool make`
- [ ] `acquisition/rig/pool.py`, `acquisition/mapping/state.py` v1 (pycolmap ReconState replaces the coverage proxy online)
- [ ] `tests/fixtures/pool_tiny/` (git-lfs)
- [ ] ZED image stamps are wall clock in the sim (the rig takes the first image *received* after settle)

## Done
- [x] Fast room capture: method `tour` (set cover + TSP + continuous keyframes) and known-pose SfM. Flown 2026-10-18: 2.0 min of drone
  time, 108 m, 403 keyframes, SfM 403/403, ATE 8.2 cm vs GT (`data/20261018-152242_warehouse_room_tour_mavros_s0`). ADR-K12
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
