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
