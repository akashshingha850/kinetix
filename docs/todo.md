# Todo

Current phase: **K1**, with a room-capture track run ahead of it (ADR-K11, ADR-K12). Phase definitions and exit tests:
[roadmap.md](roadmap.md). The evidence for each done item is in the ADR it names and in its run directory.

## Now — room photogrammetry (Warehouse, `config/scenes/warehouse_room.yaml`)
How to run it: `METHOD=tour scripts/room_scan.sh all` (sim up, fly, `kinetix sfm`), or [architecture.md](architecture.md) §5a.
- [ ] Dense reconstruction (K2 `offline/densify.py`: COLMAP MVS or gsplat) for a real completeness figure; `completeness_proxy`
  from the sparse cloud is a lower bound (0.55 on the 2026-10-18 tour)
- [ ] Tour: re-plan online from the keyframes' real coverage (dry 0.91 vs flown 0.79: the drone lags the carrot in turns)
- [ ] Tour: try cruise 1.5 m/s and size it against motion blur on the real ZED (exposure × speed / GSD)
- [ ] Tour on an unknown room: needs an exploration pass to build the room box + obstacles the planner takes from the scene file
- [ ] Long flights drift (bisg VIO: 2.4 m over 416 m; a 108 m tour stays at 0.19 m RMS): keep room flights to 2-3 min tours until
  bisg has loop closure (bisg todo)
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
