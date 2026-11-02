# Todo

Current phase: **K1**, with a room-capture track run ahead of it (ADR-K11, ADR-K12). Phase definitions and exit tests:
[roadmap.md](../roadmap.md). The evidence for each done item is in the ADR it names and in its run directory.

## Structure and research framing (2026-11-02)
- [x] One method table, `acquisition/planner/methods.py`: loop, fixed/scored, surface (box | gt | depth), needs depth, and
  `uses_gt(method, rig)`. `kinetix/cli.py`, `planner/loop.py` and `planner/nbv.py` read it instead of testing method names
  (`DEPTH_METHODS` and the rig-name check in cli.py are gone). Behaviour unchanged; `acquisition/tests/test_methods.py`.
  The method × rig table (oracle to real flight) is architecture.md §3a.
- [x] Docs: `docs/README.md` (index, reading paths, how to debug a run); methodology.md §2.1 (each stage's peer-reviewed
  evidence, refs [51]–[61] added to related_work.md, [54] and [58]–[61] still to Crossref-check); paper/research-questions.md and
  ADR-K14 (**proposed**: RQ1–RQ4, K-A–K-D, N@q as a metric); roadmap "Steps to submission" S1–S8 by RQ.
- [ ] Both authors accept or reject ADR-K14; on acceptance replace publication-plan.md §2 and CLAUDE.md's C1–C5 line.
- [ ] Novelty sweep for K-A (registration-aware view planning, research-questions.md §4) before S2's result is called a contribution.
- [x] Debug aids (docs/README.md "Debug a run"): every frame/step checked as written (`WARN` in log.txt, `summary.warnings`),
  Python/numpy warnings into log.txt, a crash closes the run with `stop_reason: error` + traceback, `kinetix run --strict` raises
  at the first NaN/warning/problem, `kinetix check <runs>` gates each step (kinetix/checks.py, kinetix/tests/test_checks.py).
  First `kinetix check data/2*` (2026-11-02): 5 isaac runs of `depth_coverage` / `surface_normal` (261031-220608, -231423,
  -231433, -231556, -231615) say `uses_gt: false` but read renderer GT depth (written before methods.py): never count them as
  GT-free. 261031-213914 died without a summary; 20261028-163032 has no run.yaml (old layout).
- [ ] Both authors decide the two-paper split (research-questions.md §6: planning paper at IROS 2027, deadline 2027-03-01;
  reconstruction paper later); on acceptance update publication-plan §5 (shared path, own PR).

## Now — room photogrammetry (Warehouse, `kinetix/scenes/warehouse_room.yaml`)
How to run it: `METHOD=tour scripts/room_scan.sh all` (sim up, fly, `kinetix sfm`), or [architecture.md](architecture.md) §5a.
- [ ] **Config split, room height, experiments (2026-10-30)**: `acquisition/config/default.yaml` is now `common.yaml` (camera, rig,
  shared keys) + `object.yaml` (rings, orbit, object coverage/planner) or `room.yaml` (grid, tour, and `explore`, moved here), picked by
  the scene's `kind`; the merged configs are unchanged apart from the height keys below. Room height: the band follows the room,
  1.1 m over the floor to 1.0 m under the ceiling (`explore.fit.altitude_above_floor_m: [1.1, null]`, was a 4.9 cap; one rule,
  `kinetix/config.py` `altitude_band`): from the fitted room with `--explore`, from the room box for a scene file with
  `altitude_m: auto` (warehouse_room.yaml since 2026-10-30, was [1.1, 4.9]; now map 1.22..8.12, workspace z follows). Exploration
  itself keeps a fixed search band (`explore.altitude_m` [1.1, 4.9]). `planner.grid.heights: auto` fills the band with layers
  `<= layer_m` 1.0 m apart. Dry Warehouse tour: coverage 0.987 / 98 m (fixed band + 3 layers: 0.964 / 101 m); the plan uses z
  1.2..5.2 only, the upper layers win no viewpoint in the dry model. Not flown yet: the roof beams (8.8 m) now count as obstacles
  within `obstacle_reach_m` of the band (unit test `test_fit_puts_the_wall...`). Experiments: overlays `acquisition/config/experiments/{close, dense,
  sparse, fine_grid, slow, traditional}.yaml` (`--config`, or `room_scan.sh --experiment NAME`; each sets `experiment:` in run.yaml),
  compared by SfM result with `uv run python -m acquisition.compare` (one row per run + the config keys that differ). Next: fly each
  overlay twice in the Warehouse with `--explore`, then `kinetix sfm` + compare.
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
- [ ] Viewing distance vs flight cost (2026-10-29, `acquisition/tests/tour_bench.py`, dry rig, known Warehouse, 20 m true sight).
  The ray model reproduces `163032`'s far views: nearest viewing distance p50 9.0 m flown / 9.2 m dry, a close (<= 5 m) 8-20 deg pair
  (`near_tri`) on 0.7 % / 4 % of the surface; it cannot predict SfM's track angle (chains continuous keyframes into 59 deg tracks
  where SfM had 7.5 deg), so that stays an SfM measurement. A flat `depth_range_m` cap <= 5 m loses half the room (a level camera at
  1.2-4.4 m sees the 9 m ceiling and the wall tops only from >= ~8 m). `planner.tour.range_factor` (off by default) counts each surface
  point only from within factor x its closest reachable viewing distance (line of sight, incidence, vertical FOV), clipped to
  [near_m 5, 12 m]. x1.5: budget 120 -> 120 viewpoints, 919 keyframes, 274 m, est. 5.8 min, near_tri 0.30, all surface seen (today:
  24 / 301 / 101 m / 1.9 min / 0.04); budget 400 -> 2283 keyframes, 12.4 min, near_tri 0.40, still short of the 0.8 plan target.
  Flown (`20261029-115558`, explore + x1.5, fresh sim, same config as `163032` but the knob): 958 keyframes, 288 m, 15.5 min wall;
  SfM ATE vs GT rmse 1.30 -> 0.11 m (median 0.21 -> 0.09, max 20.6 -> 0.73), completeness 0.41 -> 0.57 (walls 0.33 -> 0.50),
  points 23 k -> 58 k, track length 6.4 -> 6.9; nearest view p50 9.0 -> 7.0 m. But near_tri only 0.058 (dry `--explore` the same,
  0.075): the fitted floor sat 4 cm high, the band started at 1.257 m and `grid_poses` dropped the 1.2 m layer (its doc said clip),
  so the floor was only seen from >= 2.8 m. Fixed (2026-10-29): heights are clipped into workspace + altitude band, not dropped
  (`test_grid_heights_outside_the_workspace_are_clipped_not_dropped`); dry `--explore` x1.5 near_tri 0.075 -> 0.28.
  Flown again (`20261029-132346`, `explore.max_steps=26`: two flights at 20 never reached the y- end, `125530` landed, `130358`
  fitted the room 3.2 m short): near_tri 0.30, near_seen 0.37 (was 0.175), 928 keyframes, 341 m, coverage 0.98. SfM, though, is no
  better than without the fix: ATE rmse 0.15 m (median 0.096; was 0.11 / 0.091), completeness 0.53 (was 0.57; walls 0.52 vs 0.50,
  floor 0.54 vs 0.63, ceiling 0.53 vs 0.63), one flight each. So the gain over `163032` comes from more viewpoints (120 vs 24), not
  from closer views, as far as ATE and the 0.25 m completeness proxy can tell (neither sees GSD). Exploration at max_steps 20 is
  marginal in the Warehouse (2 of 4 flights missed the y- end after a first move that saw ~0 frontier). Next: the control, budget
  120 at a flat 12 m (more views, not closer), and a resolution metric (GSD / point density on the true surface) before trading
  flight time for distance.
  Repeat (`20261029-194829`, same config as `132346`, fresh sim, after the rebase on main): explore reached the y- end (34 views,
  max wall error 0.41 m), 990 frames, 384 m, 17.7 min wall, coverage 0.97. SfM: all 990 registered, 49 k points, track 6.2,
  completeness 0.54 (walls 0.52, floor 0.58, ceiling 0.53, as `132346`); ATE rmse 0.52 m, but that is 5 explore views
  (900005-900026, 1.6-12.5 m off, the explore-view BA bug of `163032`): without them rmse 0.24, median 0.16 (was 0.096). EKF2 vs
  GT p50 0.57 m, max 1.0 m over the flight
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
- [x] **T1 without flight (2026-10-31):** `sim/render_server.py` (batch + serve) + rig `isaac` (`acquisition/rig/isaac_render.py`,
  `scripts/render_server.sh`): every view rendered at the exact pose, a 20-view run in ~4 s instead of a 3-6 min flight + sim restart.
  The reference flights aim at. `office_buddha`, seed 0, points of the sparse model vs the GT statue (median / share within 1 cm):

  | method | rig | views | SfM registered | points on object | median | <1 cm |
  |---|---|---|---|---|---|---|
  | roi_coverage | isaac (T1) | 11 | 9/11 incremental, 11/11 known-pose | 286 / 287 | 2.1 / 3.0 mm | 0.98 |
  | roi_coverage | mavros (T2) | 11 | 9/11 | 396 | 21.8 mm | 0.22 |
  | surface_normal | isaac (T1) | 19 | 8/19 incremental, 19/19 known-pose | 715 / 1498 | 0.4 / 6.4 mm | 1.00 / 0.99 |
  | surface_normal | mavros (T2) | 20 | 9/20 | 811 | 23.6 mm | 0.10 |

  Reading: on flights the error is the EKF2 pose (3-7 cm) carried into the model; with exact poses the same images give mm. Incremental
  SfM registers about half the views even on perfect images (the object fills a small part of a 100 deg view): reconstruction, not
  acquisition, is the limit here. Open: known-pose mode sits 3-6 mm off where incremental + Sim3 sits 0.4-2 mm (focal is fixed at
  529.8 in both: check the render camera's pose/intrinsics convention); MRGCS can start only `--rig mavros`
- [x] **Planning without SfM in the loop (2026-10-31):** the coverage model (kinetix/coverage.py) now also scores views on a real
  surface: `coverage_oracle` = NBV on the scene's GT-visible surface (`gt_observable.npz`, sim only, `uses_gt: true`), `depth_coverage`
  = NBV on the box until `planner.depth_surface.after_views` (4), then on the depth patches those views saw + the still-unseen box.
  Objects now require SfM-matchable rays (`coverage.match_max_deg` 30, `min_overlap` 0.2; object.yaml). T1 (`--rig isaac`), seed 0,
  scored on common ground (incremental SfM vs the GT statue; patch completeness = 4 cm GT patches with a model point within 2 cm):

  | method (match deg / overlap) | images | registered | points on object | patch completeness |
  |---|---|---|---|---|
  | orbit | 20 | 20/20 | 1029 | 0.248 |
  | roi_coverage (30 / 0.2, the new default) | 20 | 20/20 (9/11 before matchability) | 1267 | 0.265 |
  | depth_coverage (30 / 0.2) | 20 | 20/20 | 707 | 0.204 |
  | coverage_oracle (30 / 0.2) | 20 | 13/20 | 298 | 0.114 |
  | coverage_oracle (20 / 0.3) | 20 | 20/20 | 674 | 0.243 |
  | **coverage_oracle (15 / 0.5)** | **14** | 14/14 | 1233 | **0.272** |
  | roi_coverage / depth_coverage (15 / 0.5) | 3 | 3/3 | ~240 | ~0.09 (stall: the box proxy cannot meet 50 % overlap) |
  | roi_coverage (20 / 0.3) | 20 | 20/20 | 952 | 0.238 |
  | depth_coverage (20 / 0.3) | 13 | 13/13 | 633 | 0.196 |
  | surface_normal | 19 | 9/19 | 791 | 0.154 |

  Reading: the reconstructability rule must be calibrated to SfM, and per surface: on the true surface strict (15 deg / 0.5) is
  best and reaches roi_coverage's completeness with 14 images instead of 20 (the reference), lenient (30 / 0.2) misleads the
  oracle; on the box proxy strict stalls. depth_coverage does not beat the box yet. One seed each: run seeds 0-4 before any
  claim; sparse-point completeness is a stand-in until densify (reconstruction todo). Next: a better realistic surface
  (depth patches all round, e.g. from the coarse orbit as in surface_normal) with the strict rule, measured against the oracle
- [x] Method `surface_normal` (planner/surface.py, the benchmark's explore_exploit after Liu 2026): flown 2026-10-31
  (`261031-214529`, 8 coarse + 12 close views at 0.71-0.81 m, transits round the object, all views inside the scene rules)
- [ ] Review ADR-K10 (pools with SDK depth via the twin); if accepted, `render_server.py --zed-depth` builds on bisg's
  `docker/sim/tools/zed_pool_probe.py` pattern
- [ ] Review the ROS 2 pipeline proposal (`docs/acquisition/ros-pipeline.md`, ADR-K09). On acceptance, move its §3/§4 tables into interfaces.md §4/§5
- [ ] Review the doc set; answer Q1 (mount pitch), Q2 (object set), Q6 (T0 stereo source): architecture.md §12
- [ ] Review the contribution wording C1–C5 (`paper/publication-plan.md` §2) and the venue plan (§5)
- [ ] Check asset licences (SimReady, Objaverse++, GSO, OmniObject3D) before fixing the object set
- [ ] Arrange the real-object scanner for K6 GT (Q4), needed by G3
- [ ] Move `kinetix/scenes/` to `$KINETIX_DATA` (`KINETIX_DATA=/opt/kinetix-data` set in workstation A's `.env` 2026-10-31; the
  room scenes still live in git)
- [x] Switch `tool.uv.package = true` (2026-10-28, hatchling backend): `uv run pytest` 59 passed, `uv run python -m kinetix` works
- [x] Flat two-author layout (2026-10-29): `kinetix/` shared lib (+ `scenes/`), `acquisition/` (planner, mapping = old `recon/`, rig,
  config), `reconstruction/` (old `offline/`), flights `data/` (was `runs/`), outputs `results/<run>/<method>/` (was `<run>/sfm/offline/`),
  tests next to the code; `kinetix/tests/test_boundaries.py` replaces the two import tests (which passed vacuously on a missing folder).
  MRGCS `kinetix_bridge.py` follows. 58 passed (uv and the kinetix image). docs/ follows the same split: shared docs at the top,
  `acquisition/` (architecture §5-8 split out, ros-pipeline, todo), `reconstruction/` (pipeline = old architecture §9, todo),
  `paper/` (publication-plan, benchmark, research/ with space-free names); 91 relative links checked
- [x] Q3 (2026-10-31): bisg mounts `SCENES_DIR` (= `$KINETIX_DATA/scenes`) read-only at `/scenes`; a scene's `bisg.yaml` locator is
  `-c` / `SIM_SCENARIO`. The kinetix container mounts `$KINETIX_DATA` at its host path (docker/compose.yaml)
- [x] First object scene `office_buddha` (2026-10-31): recipe `sim/scenes/office_buddha.yaml` -> `sim/build_scene.py` -> `$KINETIX_DATA/scenes/`
  (Smithsonian Cosmic Buddha laser scan, CC0, 150k tris + 4k texture, in the Isaac Office lobby, CC BY 4.0; scene lights: the
  Office's own lamps are authored invisible). Flown on fresh sims: `roi_coverage` s0 13 views (gain plateau), cov 0.86, SfM 11/13,
  ATE 0.7 cm (`261031-012800`); s1 20 views (budget), cov 0.94, SfM 12/20, 1076 pts, ATE 1.6 cm (`261031-021929`).
  **One capture per sim run:** a 2nd flight on the same sim (`261031-013603`) held position but yaw swung +-15 deg (period ~12 s,
  EKF2 = truth, so the vehicle really turned) and timed out 13x on the 3 deg settle; the same seed on a restarted sim was clean. Runs record `summary.map_shift` (measured - scene world_to_map; z varies
  ~0.2 m per run, the Warehouse too): GT + map_shift = the run's map. Open: SfM/densify scored against `gt_observable.ply`
  (reconstruction), lighting probe like the Warehouse's, 2 more objects (Q2)
- [ ] **Validation to the flight gate (2026-11-01, [methodology.md](../methodology.md))**: method stages M0–M9 and checks V0–V8; no
  new T2 work for the object paper before the flight gate (§5). Built: `coverage.k_min` + `MeshOcclusion` (the oracle now sees GT
  self-occlusion; `kinetix/tests/test_coverage_occlusion.py`), `bench/validate.py` (V3: sparse ρ 0.54–0.56, fails until dense),
  `densify` + `runs_from` bench keys, `config/bench/{dense,oracle}.yaml` (running, ~12 h), `acquisition/mapping/monodepth.py`
  (DA-V2 + SfM anchoring, `test_monodepth.py`), `acquisition/tests/anchor_bench.py` (V5a: AbsRel 0.035 anchored vs 0.20 metric
  no-anchor on Buddha). Dense results (2026-11-01, `bench/dense` + `bench/oracle`): V1 F_ref 0.86–0.93 (Chair never reaches
  0.9: q relative to F_ref is open), V3 coverage → completeness ρ 0.76 (`k_min` 3, passes), overlap → registered ρ 0.24 (fails),
  V4 fails (oracle ≥ best non-GT in 1 of 24 cells: it spreads views and loses them in SfM). Next: a registration model that
  predicts SfM registration (methodology §8 2b), re-run the oracle, then V5a with online-sized anchors, ReconState (V5c)
- [x] **K2 harness + first baseline matrix (2026-11-01, roadmap reformed the same day):** `kinetix pool make` (ring-stack pool, 192
  views, ~90 s/scene) + rig `pool` (T0), `kinetix eval` (reconstruction/metrics.py, evaluate.py), `kinetix bench` (bench/matrix.py,
  report.py: budget prefixes, N@q, image reduction, Kendall's tau), baselines `orbit_rings`, `fvs`, `volumetric_ig`, bench seeds,
  scenes `office_monster` + `office_chair` (Smithsonian CC0, on a plinth). `config/bench/baselines.yaml`: 3 scenes x 9 methods x
  3 seeds x N 5-80, ~35 min on 6 jobs → `$KINETIX_DATA/bench/baselines/report.md`. F@1 % of D (sparse points), mean of 3 seeds:

  | method | buddha N=20 / 80 | monster N=20 / 80 | chair N=20 / 80 |
  |---|---|---|---|
  | orbit (1 ring, ≤ 24) | 0.39 / 0.44 | 0.56 / 0.63 | 0.03 / 0.05 |
  | orbit_rings | 0.07 / 0.61 | 0.23 / 0.85 | 0.00 / 0.22 |
  | random | 0.13 / 0.57 | 0.22 / 0.81 | 0.01 / 0.23 |
  | fvs | 0.00 / 0.59 | 0.00 / 0.82 | 0.00 / 0.20 |
  | volumetric_ig † | 0.23 / **0.64** | **0.50 / 0.89** | **0.13 / 0.28** |
  | roi_coverage | 0.23 / 0.54 | 0.38 / 0.82 | 0.01 / 0.19 |
  | depth_coverage † | 0.23 / 0.52 | 0.28 / 0.79 | 0.00 / 0.23 |
  | coverage_oracle † (30° / 0.2) | 0.00 / 0.50 | 0.24 / 0.76 | 0.01 / 0.20 |
  | coverage_oracle † (15° / 0.5) | 0.28 / 0.28 (stops ~14) | 0.29 / 0.30 | 0.08 / 0.13 |
  | reference (192 views) | 0.76 | 0.94 | 0.40 |

  † reads GT (oracle surface, or rendered GT depth). Readings: (1) **N@q is not measurable yet**: on sparse points F keeps rising
  with the image count, so nothing reaches 0.9 x F_ref by 80 views except volumetric_ig on the monster (N@0.9 = 57); densify first
  (reconstruction todo). (2) **Spread views do not register**: `fvs` / `orbit_rings` at 5-20 views are 30-90 deg apart and incremental
  SfM chains 0-40 % of them (eval_valid false = F 0); the matchability-aware planners register from N = 5. (3) **The K2 oracle check
  fails**: neither coverage_oracle beats every baseline (lenient misleads, strict stops at ~14 views: no view keeps 50 % overlap),
  so the coverage model, not the evaluation, is what is wrong; volumetric_ig with GT depth is the best method on all three scenes.
  (4) office_chair (no colour texture) is hard for everyone (F_ref 0.40). (5) T0 vs T1 (`config/bench/tiers_t1.yaml`, office_buddha,
  6 methods x 3 seeds, sessions in the kinetix container, scored on the host): ranking tau = 0.87, F@80 within 0.035 for 5 of 6
  methods (volumetric_ig 0.644 vs 0.587) → `$KINETIX_DATA/bench/tiers/report.md`. The ring stack's 2.0 m ring sees little of
  the 0.9 m objects with a level camera (Q1 mount pitch / per-scene rings)
- [ ] `acquisition/mapping/state.py` v1 (pycolmap ReconState replaces the coverage proxy online)
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
