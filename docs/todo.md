# Todo

Current phase: **K0 → K1**. Phase definitions and exit tests: [roadmap.md](roadmap.md).

## Done
- [x] **Sim NBV demo** (asked for ahead of the K1→K4 order) — 2026-10-11: first closed loop on the bisg twin. `src/kinetix/`:
  `core/` (types, geometry, config + scene loader, runlog), `recon/coverage.py` (GT-free ROI-proxy coverage: a surface point
  counts once two rays ≥ 8° apart see it; stand-in for `recon/state.py`), `planner/` (ring-stack sampler, box constraints with
  straight-segment check, gain − λ·travel select, budget/coverage/plateau stop, loop), scorers `roi_coverage` (the demo NBV,
  NOT the K3 method) and `random`, `orbit` sequence; rigs `dry` (kinematic, tests) and `mavros` (T2: OFFBOARD setpoints, settle,
  lazy left-image grab, T_base_cam from TF, Pegasus GT logged). Scene `config/scenes/simple_room_blocks.yaml` (bisg
  `docker/sim/sim.yaml`: Simple Room, two cuboids on the table). Image `docker/` (ros:jazzy-ros-base + mavros_msgs + tf2, no uv
  venv yet), `python -m kinetix run --scene ... --method roi_coverage|orbit|random --rig dry|mavros`. Unit tests 12/12 in the image.
  Dry rig: roi_coverage 96 % in 7 images / 7.7 m vs orbit 79 % and random 71 % after 20.
  **Flown on the twin (T2, raptor, MAVROS)**: 7 images, coverage 96.5 %, 9.5 m, 112 s wall, landed by itself
  (`runs/20261011-034929_simple_room_blocks_roi_coverage_mavros_s0`); camera pose vs Pegasus GT 1.8–3.3 cm, vs target ≤ 5 cm.
  The 2.0 m views put the object at the bottom image edge (level mount): evidence for Q1.
  Open: real ReconState (pycolmap) replaces the coverage proxy; ZED image stamps are wall clock in the sim (the rig takes the
  first image *received* after settle); move `config/scenes/` to `$KINETIX_DATA/scenes` with scene.usd/gt_mesh (K1)
- [x] **Sim room = Warehouse** — 2026-10-17: bisg's default world is now Isaac's Simple Warehouse (found with NVIDIA USD
  Search; textured, ~9 m ceiling, inner walls x -10.5..9.5 / y -12..18, obstacle-free floor x -8.5..9.0 / y -12.0..12.75, racks on
  the west wall), its lights retuned for exposure (`sim.yaml` `light_scale`). **Target = photogrammetry of the whole room**, not an
  object: the scan cuboids are gone from bisg `sim.yaml`, so `simple_room_blocks` no longer has its blocks (it also needs
  `world: Simple Room`). Flown once with a temporary object scene before the change (7 images, landed); the first
  `--rig mavros` attempt aborted "MAVROS not connected after 30 s sim" (DDS discovery, retry passed) and the process
  segfaulted (rc 139) on exit
- [x] **Room photogrammetry: NBV + offline SfM** — 2026-10-17 (ADR-K11). Room scenes (`kind: room`, `config/room.yaml` layer,
  `config/scenes/warehouse_room.yaml` from bisg USD bounds + PhysX probes); `recon/coverage.py` inside-out proxy (room faces inward,
  obstacles occlude and count), matchable-angle triangulation (`match_max_deg`), absolute shared-surface gate (`min_shared`);
  `GridSampler` (2 m grid x 3 heights x 12 yaws, ~4000 candidates, 0.4 s per step); room constraints and safe return; `MavrosRig`
  carrot setpoints (`cruise_mps` 0.8), measured `world_to_map` + scene shift (the EKF2 z origin moves ~0.25 m between boots),
  MCAP rosbag per flight (`rig.bag`, `runs/<run>/rosbag/`, git-ignored); `kinetix sfm <run>` (pycolmap 4.2.1 in the image:
  co-visibility pairs from the rig poses, every verified pair checked against the EKF2 relative pose, fixed PINHOLE K, 4° init
  pair, ATE vs EKF2/GT, proxy completeness, `points.ply`). 21 unit tests.
  **Flown (T2, Warehouse, lights x0.5 + ambient 2):** flight 1 (35° / 30 % overlap): 48 images, 91 % proxy coverage, 351 m, 21 min,
  4.1 GB bag; exhaustive SfM was wrong (bay aliasing), with the rig check 24/48 registered, ATE 6.9 cm vs GT. Thresholds then
  measured from its 1128 pairs (ADR-K11). Flight 2 (20° / 120 shared points, gain-chosen seed): 71 images, 90.3 %, 416 m, 28 min,
  5.5 GB bag (`runs/20261017-135843_warehouse_room_roi_coverage_mavros_s0`); SfM **47/71 registered in one model, all correct:
  ATE 8.8 cm vs GT** (EKF2 itself 1.3 m RMS off), 3941 points, reprojection 0.56 px, sparse points near 14 % of the proxy surface.
- [x] **Fast room capture: tour + continuous capture + known-pose SfM** — 2026-10-18 (ADR-K12). `planner/tour.py` (set cover,
  TSP 2-opt, dense path), `run_tour` in the loop, `MavrosRig.fly_path` (time-parameterised carrot, keyframes 0.4 m / 6°, writer
  thread), `DryRig.fly_path`; `kinetix sfm`: `video` pairing, rig check keeps pose-less pairs, position priors (sfm mode),
  `pose_source: prior` (triangulate at EKF2 poses + BA), the default for tours. Camera loss (`no_image`) now aborts and lands at once:
  in the 2026-10-17 abort the ZED wrapper died (bisg R9) and EKF2 held position on flow + range (eph +2.5 cm/s), but the session
  retried for 95 s until PX4 fell to ALTCTL. Flown: 2.0 min of drone time, 108 m, 403 keyframes; SfM 403/403, ATE 8.2 cm vs GT,
  65 k points, 55 s (`runs/20261018-152242_warehouse_room_tour_mavros_s0`, 1 GB bag). 23 unit tests
- [x] Room SfM registration: solved for tours by continuous keyframes + known-pose triangulation (403/403, ADR-K12); stop-and-shoot NBV
  still registers 2/3 (incremental SfM + priors puts blocks one bay off in this repetitive hall)
- [ ] Dense reconstruction (K2 `offline/densify.py`, COLMAP MVS or gsplat) for a real completeness number: sparse SIFT points cannot
  cover the 0.5 m proxy grid, so `completeness_proxy` from the sparse cloud is a lower bound
- [ ] Long flights drift: EKF2 horizontal error grows to 2.4 m over 416 m (bisg VIO; its bench flies 26 m); a 108 m tour stays at 0.19 m RMS.
  Keep room flights to tours (2-3 min); loop closure (ZED GEN_3 area memory) is the next bisg step for longer ones
- [ ] Tour follow-ups: re-plan online from the keyframes' real coverage (dry 0.91 vs flown 0.79: the drone lags the carrot in turns);
  try cruise 1.5 m/s (motion blur on the real ZED: exposure vs speed); dense reconstruction for a real completeness figure
- [ ] Paper scope: C1-C5 are written for objects; decide whether rooms become the target or a second scene class (ADR-K11)
- [x] K0 doc set (architecture, interfaces, data-format, roadmap, testing, benchmark, decisions, setup) — 2026-10-08
- [x] K0 Python env (`pyproject.toml`, `uv.lock`), `scripts/check_env.py` passes on workstation B — 2026-10-08
- [x] Mono-vs-stereo study (`docs/research/mono-vs-stereo.md`). Plan updated: ADR-K07 sensing axes, stereo in pools/interfaces/benchmark/roadmap — 2026-10-08
- [x] Q6 tested in bisg 2026-10-10: real ZED SDK depth on teleported twin views works (settle ≤ 0.54 s, 0.35–0.65 % error < 5 m). Docs: ADR-K10 (proposed), architecture §2a/Q6, interfaces §5 `--zed-depth`, data-format `zed/`, `zed_conf/`, ros-pipeline §10. bisg now advertises `confidence/confidence_map` and has `./bisg zed record` (SVO)
- [x] ROS 2 topic analysis + pipeline proposal (`docs/ros-pipeline.md`, ADR-K09 proposed) — 2026-10-10
- [x] Novelty search (`docs/research/reports/Kinetix novelty search.md`) and plan updated: contributions C1–C5, baselines fvs/volumetric_ig/fisherrf/ma_scvp, Tammes pool, N@q metrics, anchoring ablation, `docs/publication-plan.md` (ADR-K08) — 2026-10-08

## Next (K1)
- [ ] Review ADR-K10 (pools with SDK depth via the twin); if accepted, `render_server.py --zed-depth` builds on bisg's `sim/tools/zed_pool_probe.py` pattern (Kinetix code, not a copy)
- [ ] Review the ROS 2 pipeline proposal (`docs/ros-pipeline.md`, ADR-K09). On acceptance, move its §3/§4 tables into interfaces.md §4/§5
- [ ] Review the doc set; answer Q1 (mount pitch), Q2 (object set), Q6 (T0 stereo source): architecture.md §12
- [ ] Review the contribution wording C1–C5 (`publication-plan.md` §2) and the venue plan (§5)
- [x] References cross-checked (Crossref + arXiv), DOI links on all 49 entries; fixed [3] [11] [20] [21] [27] [34] [45] — 2026-10-08
- [ ] Check asset licences (SimReady, Objaverse++, GSO, OmniObject3D) before fixing the object set
- [ ] Arrange the real-object scanner for K6 GT (Q4), needed by G3
- [ ] Create `$KINETIX_DATA` and `.env`
- [ ] `src/kinetix/core/` (types, geometry, config, runlog) + unit tests; switch `tool.uv.package = true`
- [ ] First scene: pick the object, build `scene.usd` / `gt_mesh.ply` / `scene.yaml`
- [ ] Q3: bisg change so the sim container can read `$KINETIX_DATA/scenes` (done in bisg_isaac)
- [ ] `sim/render_server.py` batch mode + `kinetix pool make`
- [ ] `rig/pool.py`, `planner/loop.py` + orbit/grid/random, `recon/state.py` v1, `kinetix run`
- [ ] `tests/fixtures/pool_tiny/` (git-lfs)

## Watch list (check monthly; novelty sweeps at G2, G4, G5)
- ObjView-Bench code + object pool release (arXiv 2605.10707). It may become the default object protocol
- ActMVS code (GitHub TrickyGo/ActMVS): a possible monocular baseline
- Hestia / VIN-NBV code release (object-NBV baselines)
- AREA3D (CVPR 2026, arXiv 2512.05131): backbone and sensors not yet verified
- New arXiv preprints on Isaac Sim + PX4 NBV, monocular/foundation-depth NBV, or mono-vs-stereo active reconstruction
