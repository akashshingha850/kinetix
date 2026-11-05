# Graph Report - kinetix  (2026-11-02)

## Corpus Check
- 117 files · ~112,088 words
- Verdict: corpus is large enough that graph structure adds value.

## Summary
- 1123 nodes · 2914 edges · 72 communities (48 shown, 24 thin omitted)
- Extraction: 92% EXTRACTED · 8% INFERRED · 0% AMBIGUOUS · INFERRED: 238 edges (avg confidence: 0.89)
- Token cost: 0 input · 0 output

## Community Hubs (Navigation)
- Room Tour Planner
- SfM and Bench Harness
- Scene and Bench Configs
- Rig Adapters and Drift
- Run Comparison Reports
- Config and Session Loop
- Coverage Model
- Metrics and Scene Build
- NBV Bench and Explore
- Core Types and Validation
- MAVROS Rig
- Exploration Planner
- Explore Tests
- Occupancy Map
- Rig Liveness Tests
- Novelty Search Report
- Validation Architecture V0-V8
- Render Server
- Tiers and Component Protocols
- Room Fit
- Geometry and Scorers
- Roadmap and Testing
- Room Tour ADRs
- Project Docs Hub
- Dry Rig Motion
- Kinetix CLI
- Related Work and Mono vs Stereo
- Aerial Reconstruction Notes
- Baselines and Pool Format
- Foundation Models in Planning
- Tour Bench
- C1 Contribution and Anchored Depth
- Densify and Offline Pipeline
- NBV Scorer Family
- Run Directory and Area Split
- Sensing Modes
- Radiance Field NBV
- Room Scan Script
- Exploration Tests
- Depth and Snapshot Streams
- Room Photogrammetry Review
- Proxy Validation (V3)
- ZED Depth Study
- Benchmarks and Venues
- Work Plan Phases
- Run Logging
- Import Boundary Test
- Setup and Environment
- Cube Mesh Test
- Data Prune Workflow
- Render Server Script
- ZED Stream Budget
- Known Issues
- Runtime Placement
- Metrics Guard
- Timeline and Venues
- Post-checkout Hook
- Post-merge Hook
- Pre-push Hook
- ROS Executors Design
- ROS at the Edges
- Zero-copy Upgrade
- Bench Matrix Dir
- Kinetix CLI Commands
- kinetix namespace

## God Nodes (most connected - your core abstractions)
1. `Pose` - 72 edges
2. `Intrinsics` - 64 edges
3. `CoverageState` - 57 edges
4. `camera_pose()` - 53 edges
5. `load_scene()` - 49 edges
6. `OccupancyMap` - 48 edges
7. `MavrosRig` - 43 edges
8. `Scene` - 43 edges
9. `load_config()` - 40 edges
10. `Box` - 38 edges

## Surprising Connections (you probably didn't know these)
- `method: depth_coverage` --semantically_similar_to--> `depth_coverage depth_surface planning`  [INFERRED] [semantically similar]
  config/bench/baselines.yaml → acquisition/config/object.yaml
- `method: volumetric_ig` --semantically_similar_to--> `volumetric_ig baseline (occupancy map)`  [INFERRED] [semantically similar]
  config/bench/baselines.yaml → acquisition/config/object.yaml
- `method: orbit` --semantically_similar_to--> `orbit baseline`  [INFERRED] [semantically similar]
  config/bench/baselines.yaml → acquisition/config/object.yaml
- `OccupancyMap` --uses--> `Intrinsics`  [INFERRED]
  acquisition/mapping/depthmap.py → kinetix/types.py
- `OccupancyMap` --uses--> `Pose`  [INFERRED]
  acquisition/mapping/depthmap.py → kinetix/types.py

## Import Cycles
- None detected.

## Hyperedges (group relationships)
- **Room tour experiment variants** — acquisition_config_experiments_close, acquisition_config_experiments_dense, acquisition_config_experiments_fine_grid, acquisition_config_experiments_slow, acquisition_config_experiments_sparse, acquisition_config_experiments_traditional [EXTRACTED 1.00]
- **Benchmark chain baselines -> dense -> oracle** — config_bench_baselines, config_bench_dense, config_bench_oracle, config_bench_tiers_t1 [EXTRACTED 1.00]
- **Office object scan scenes** — sim_scenes_office_buddha, sim_scenes_office_monster, sim_scenes_office_chair, sim_scenes_office_world [EXTRACTED 1.00]
- **C1 SfM-anchored monocular NBV components** — docs_methodology_m3_anchored_depth, docs_acquisition_architecture_depth_frontier_scorer, docs_acquisition_architecture_sfm_uncertainty_scorer, docs_acquisition_architecture_combo_scorer, docs_paper_publication_plan_c1 [EXTRACTED 1.00]
- **Four-tier realism protocol T0-T3** — docs_architecture_t0_pool_replay, docs_architecture_t1_render, docs_architecture_t2_sim_flight, docs_architecture_t3_real, docs_paper_benchmark_tier_transfer [EXTRACTED 1.00]
- **Validation V0-V8 ladder gating T2 flights** — docs_methodology_v0, docs_methodology_v1, docs_methodology_v3, docs_methodology_v4, docs_methodology_v7, docs_methodology_v8, docs_methodology_flight_gate [EXTRACTED 1.00]
- **Offline reconstruction pipeline stages** — docs_reconstruction_pipeline_sfm, docs_reconstruction_pipeline_align, docs_reconstruction_pipeline_densify, docs_reconstruction_pipeline_evaluate [EXTRACTED 1.00]
- **Kinetix roadmap phases K0-K6** — docs_roadmap_k0, docs_roadmap_k1, docs_roadmap_k2, docs_roadmap_k3, docs_roadmap_k4, docs_roadmap_k5, docs_roadmap_k6 [EXTRACTED 1.00]
- **Execution tiers T0-T3 test levels** — docs_testing_t0, docs_testing_t1, docs_testing_t2, docs_testing_t3 [EXTRACTED 1.00]

## Communities (72 total, 24 thin omitted)

### Community 0 - "Room Tour Planner"
Cohesion: 0.05
Nodes (75): BoxConstraints, ndarray, Feasibility (acquisition/architecture.md §8): inside the workspace, at an…, apply_shift(), The session entry point (acquisition/architecture.md §5): picks the method's…, depth_surface(), orbit_order(), ndarray (+67 more)

### Community 1 - "SfM and Bench Harness"
Cohesion: 0.05
Nodes (73): anchor(), MonoDepth, ndarray, Monocular depth for planning (methodology.md M3, sensing.plan_depth `mono` /…, (H, W) float32 at the image's size: relative inverse depth (RELATIVE) or metres…, (s, t, inliers) with 1/z ≈ s·rel[v, u] + t over the anchors (pixel u, v; depth…, Anchored metric depth (m); NaN where s·rel + t gives no positive depth below…, to_depth() (+65 more)

### Community 2 - "Scene and Bench Configs"
Cohesion: 0.06
Nodes (62): common.yaml (shared acquisition settings), Camera model (ZED Mini 720p), Coverage model (GT-free ROI proxy), Planner defaults (timeouts, ROI clearance), Rig settings (mavros/dry rig, cruise, tolerances), Closer views experiment (range_factor 1.5), Dense overlap experiment (0.25 m / 4 deg keyframes), Fine grid experiment (1.5 m spacing, 0.7 m layers) (+54 more)

### Community 3 - "Rig Adapters and Drift"
Cohesion: 0.07
Nodes (16): depth_points(), DriftTracker, best_around(), ndarray, Map-frame points (n, 3) of the valid near returns of a depth image (same…, Closeness-to-occupied field over the box around the occupied voxels, and that…, Mean field value of `pts` shifted by each offset in T (m, 3)., Register one frame taken at `ekf_pose` against the map; moves self.t when the… (+8 more)

### Community 4 - "Run Comparison Reports"
Cohesion: 0.07
Nodes (29): config_diff(), _flat(), _get(), load_run(), main(), Path, Compare flights by their SfM result: which acquisition setting gave the better…, _run() (+21 more)

### Community 5 - "Config and Session Loop"
Cohesion: 0.14
Nodes (36): run_session(), load_rig(), The K2 baselines on the dry rig (fvs, volumetric_ig, orbit_rings), bench seeds,…, session(), test_fvs_spreads_the_views(), test_orbit_rings_prefixes_are_spread(), test_pool_rig_replays_a_pool(), test_seeds_change_the_first_view() (+28 more)

### Community 6 - "Coverage Model"
Cohesion: 0.10
Nodes (23): test_room_surface_normals_point_inward_and_floor_under_obstacle_is_dropped(), box_surface(), CoverageState, MeshOcclusion, ndarray, Self-occlusion test against a mesh (the GT mesh: oracle only): a depth image…, (h, w) optical depth (m, inf = no hit) at 1/scale of K's resolution., room_surface() (+15 more)

### Community 7 - "Metrics and Scene Build"
Cohesion: 0.08
Nodes (36): io, open3d, ndarray, Geometric metrics of a reconstruction against GT (paper/benchmark.md §3).…, Distance (m) from each point to the triangle mesh (V, F)., Distance (m) from each query point to its nearest neighbour in `points`., accuracy = mean recon -> GT mesh, completeness = mean GT observable points ->…, score() (+28 more)

### Community 8 - "NBV Bench and Explore"
Cohesion: 0.10
Nodes (37): acquisition_planner, _cands(), explore_once(), wrapped(), go(), is_frontier(), main(), make_oracle() (+29 more)

### Community 9 - "Core Types and Validation"
Cohesion: 0.12
Nodes (23): IsaacRenderRig (T1, interfaces.md §5): every capture is rendered by…, PoolRig (T0, interfaces.md §3): replays a pre-rendered view pool (data-…, Ray-cast depth of a box room (+ box obstacles) for the dry rig and tests: what…, surface_normal (planner/surface.py) on the dry rig: coarse orbit -> ZED-like…, test_gt_surface_thins_and_requires_the_npz(), V3 proxy validity (methodology.md §4): does the coverage a planner optimises…, copy, importlib_util (+15 more)

### Community 10 - "MAVROS Rig"
Cohesion: 0.14
Nodes (8): MavrosRig, on_depth(), on_img(), settled(), EKF2 says the drone moves faster than it can: the estimate has diverged (sim:…, Raise RigError(aborted) when the rig can no longer fly: the stream thread died,…, RigError, RuntimeError

### Community 11 - "Exploration Planner"
Cohesion: 0.12
Nodes (27): The scene's room, workspace, obstacles and altitude band moved by d (a drift…, A scene in the scene-file format (config/scenes/*.yaml), so a fitted room can…, scene_to_dict(), translated(), face_targets(), in_frustum(), los_clear(), next_view() (+19 more)

### Community 12 - "Explore Tests"
Cohesion: 0.11
Nodes (27): compare(), How far the fitted scene is from a known one (the scene file): wall offsets and…, ndarray, (H/stride, W/stride) float32 depth; NaN beyond `max_range` (no return). `K` is…, render_depth(), explore(), Exploration of an unknown room (ADR-K13): ray-cast depth, the occupancy map,…, Real ZED depth is valid almost everywhere (14400/14400 rays in the sim). A… (+19 more)

### Community 13 - "Occupancy Map"
Cohesion: 0.13
Nodes (12): OccupancyMap, ndarray, Free voxels with an unknown face neighbour: the edge of what the drone has seen…, Box dilation by rxy voxels in x and y, rz in z (separable)., Dilation radii in voxels. Centre-to-centre distances undercount the gap to an…, Voxels where the drone may stop: free, and no occupied or unknown voxel within…, Voxels the drone may pass through: known free and clear of occupied space.…, Integer voxel indices (n, 3) of map points and the mask of those inside the… (+4 more)

### Community 14 - "Rig Liveness Tests"
Cohesion: 0.11
Nodes (21): _diverged_rig(), A dead rig must abort the session instead of timing out forever (2026-10-22…, A rig whose EKF2 position jumped `speed_jump_m` within the last half second…, A yaw step made PX4 turn at 60 deg/s and the ZED's pose lagged 4-5 deg…, Every move times out, like a rig whose clock froze., A MavrosRig shell without ROS: only what _check_alive / _stream_loop touch., _rig(), _StuckRig (+13 more)

### Community 15 - "Novelty Search Report"
Cohesion: 0.14
Nodes (23): Statistics plan (bootstrap CI, Wilcoxon, Holm, Kendall), V1 evaluation fidelity, N@q images-to-quality metric, ActMVS, Missing baselines: FVS, volumetric IG, FisherRF, learned planner, FisherRF, Four-tier realism protocol (replay, render, SITL, real flight), FrontierNet (+15 more)

### Community 16 - "Validation Architecture V0-V8"
Cohesion: 0.10
Nodes (23): 4-DoF camera pose with fixed mount pitch, coverage_oracle scorer (GT upper bound), Indoor feasibility constraints (workspace, no-fly, standoff, straight segment), K2 oracle check fails (volumetric_ig best), Validation to the flight gate task, Open question Q1: ZED mount pitch, Flight gate before PX4/ROS T2 work, k_min view coverage and occlusion-aware visibility (+15 more)

### Community 17 - "Render Server"
Cohesion: 0.10
Nodes (18): carb, geometry_msgs_msg, isaacsim, isaacsim_core_experimental_utils_app, omni_replicator_core, omni_timeline, omni_usd, pil (+10 more)

### Community 18 - "Tiers and Component Protocols"
Cohesion: 0.10
Nodes (21): Pose convention T_map_cam (ENU map, OpenCV optical), Baselines are configurations of the same loop, Sim3 alignment of SfM to rig poses (metric map frame), pose_source ablation axis (sfm, prior, gt), ReconState online reconstruction (pycolmap), select = gain minus lambda times travel cost, Session loop run_session, RosRig unified client for T1-T3 (+13 more)

### Community 19 - "Room Fit"
Cohesion: 0.16
Nodes (19): components(), _extent(), face_coverage(), face_openings(), fit_scene(), _plane(), ndarray, Fit a room `Scene` (room box, workspace, obstacle boxes) to an OccupancyMap:… (+11 more)

### Community 20 - "Geometry and Scorers"
Cohesion: 0.29
Nodes (13): OccupancyMap: a live voxel map of the room from depth images posed by the…, DriftTracker: estimate how far the EKF2 pose has drifted by registering each…, body_from_camera(), camera_from_body(), ndarray, quat_to_R(), R_to_quat(), rot_y() (+5 more)

### Community 21 - "Roadmap and Testing"
Cohesion: 0.19
Nodes (17): Paper claims C1-C5 and gates G1-G5, K0 Foundations, K1 Core, scenes, T0 replay, K2 Evaluation + benchmark harness, K4 Closed loop in sim, K5 Optimisation, K6 Real drone + paper, methodology.md stages M0-M9, validation V0-V8, flight gate (+9 more)

### Community 22 - "Room Tour ADRs"
Cohesion: 0.16
Nodes (16): Unknown rooms: explore then tour (ADR-K13), Room capture method tour (ADR-K11/K12), Current phase K1 with room track, Explore flights and pose drift findings, First object scene office_buddha, Room photogrammetry track (Warehouse), Tour flown 2026-10-18: 2.0 min, 403 keyframes, ATE 8.2 cm, Viewing distance vs flight cost (range_factor) (+8 more)

### Community 24 - "Dry Rig Motion"
Cohesion: 0.19
Nodes (7): _cost(), on_img(), Turn the streamed yaw towards `target` by at most yaw_rate_deg_s x elapsed sim…, Transit: reach the camera pose (position and yaw tolerances, no settle time),…, The depth image of the same grab as an image stamped `stamp` (the wrapper…, camera_yaw(), wrap()

### Community 25 - "Kinetix CLI"
Cohesion: 0.22
Nodes (12): argparse, cmd_bench(), cmd_eval(), cmd_pool(), cmd_sfm(), default_pool(), _git_sha(), main() (+4 more)

### Community 26 - "Related Work and Mono vs Stereo"
Cohesion: 0.16
Nodes (12): Export to COLMAP and nerfstudio, ADR-K04 pycolmap only SfM backend, VGGT, ActMVS (Pu, Han, Lian 2026), COLMAP Structure-from-motion revisited, Depth Anything, FlyCo, FrontierNet (+4 more)

### Community 27 - "Aerial Reconstruction Notes"
Cohesion: 0.19
Nodes (12): Hestia (WACV 2026), Huang et al. toy drone (monocular Bebop), Gap: no quantitative real-drone indoor object NBV with COLMAP output, Hestia entry (closest real-drone object NBV), Huang toy drone (108 vs 390 views), Verification legend [V]/[S]/[M], ActiveGS / NARUTO RGB-D baselines, Depth for planning vs depth for reconstruction (2x2) (+4 more)

### Community 28 - "Baselines and Pool Format"
Cohesion: 0.18
Nodes (12): Method surface_normal (explore_exploit baseline), K2 harness and baseline matrix (9 methods, 3 scenes), T0 view-pool replay (PoolRig), View pool format pools/scene/pool, ADR-K02 Discrete view pools T0 as primary loop, Experimental axes (scenes, pools, methods, budgets, seeds), Baseline method set (orbit, fvs, volumetric_ig, fisherrf, ma_scvp, oracle), Adaptive view planning for aerial 3D reconstruction (Peng, Isler) (+4 more)

### Community 29 - "Foundation Models in Planning"
Cohesion: 0.23
Nodes (11): Depth Anything V2, GraspView, MASt3R / DUSt3R, Relative depth scale-anchored online to SfM + TSDF frontier + SfM uncertainty, Next Best Sense, DUSt3R / MASt3R / VGGT pointmap models, No work scale-fits relative depth to SfM for object NBV on a drone, GraspView (VGGT + kinematic scale) (+3 more)

### Community 30 - "Tour Bench"
Cohesion: 0.35
Nodes (10): geometry(), intrinsics(), main(), plan_and_fly(), Path, Tour benchmark: what a planning knob costs in flight and what it buys in…, Viewing geometry of `poses` on the true surface of `scene` (see the module doc)., One dry-rig tour of the Warehouse with `coverage.depth_range_m` capped at… (+2 more)

### Community 31 - "C1 Contribution and Anchored Depth"
Cohesion: 0.22
Nodes (11): T3 real flight (MavrosRig on real ZED Mini), ADR-K08 Measurement paper positioning, M3 depth: Depth Anything V2 anchored to SfM points, Ablations incl. decisive depth anchoring, C1 SfM-anchored monocular NBV, C5 Quantitative real-flight evaluation and release, Key innovations C1-C5 (revised 2026-10-08), Kinetix project overview: adaptive dynamic photogrammetry (+3 more)

### Community 32 - "Densify and Offline Pipeline"
Cohesion: 0.35
Nodes (9): Sim3 alignment to GT, densify.py (COLMAP MVS / gsplat / tsdf_depth), evaluate.py metrics.json, Offline reconstruction pipeline identical for every method, sfm.py pycolmap SfM, Dense reconstruction for real completeness figure, One metrics.json schema for every method, SfM bundle adjustment moves low-point cameras far from prior (+1 more)

### Community 33 - "NBV Scorer Family"
Cohesion: 0.22
Nodes (10): combo scorer (Kinetix method, C1), depth_frontier scorer, fvs farthest-view sampling scorer, NBV scorers table (random, fvs, volumetric_ig, sfm_uncertainty, depth_frontier, combo, oracle), OccupancyMap depthmap and roomfit, sfm_uncertainty scorer, volumetric_ig scorer (Isler/Delmerico family), Connolly next best views 1985 (+2 more)

### Community 34 - "Run Directory and Area Split"
Cohesion: 0.20
Nodes (10): Dependency rule enforced by test_boundaries.py, Module map (kinetix/, acquisition/, reconstruction/, bench/, sim/), Two-area split: acquisition vs reconstruction, Branch and PR policy, Data sync of large run files to GitHub release, Run directory is the only contract between areas, frames.jsonl and steps.jsonl records, metrics.json key set (+2 more)

### Community 35 - "Sensing Modes"
Cohesion: 0.22
Nodes (10): Sensing modes: plan_depth and recon_input axes (ADR-K07), Sensing presets mono, mono_noprior, stereo_plan, stereo_full, oracle_depth, ADR-K03 Monocular rule (superseded), ADR-K07 Sensing mode as experiment axis, capture_at settle and capture sequence, Flight rosbag recording (MCAP), T2/T3 ROS 2 interface subset of bisg contract, Topic whitelist per sensing mode (+2 more)

### Community 36 - "Radiance Field NBV"
Cohesion: 0.38
Nodes (9): ActiveGS, ActiveNeRF (ECCV 2022), ActiveSplat, FisherRF, NARUTO, NeurAR (RA-L 2023), PUN, Four families of radiance-field NBV (offline, online RGB-D, learned object-centric, geometry-prediction) (+1 more)

### Community 37 - "Room Scan Script"
Cohesion: 0.53
Nodes (9): check(), die(), fly(), kx(), latest_run(), say(), sfm(), room_scan.sh script (+1 more)

### Community 38 - "Exploration Tests"
Cohesion: 0.25
Nodes (9): hint_scene(), What the planner and the rig get in exploration mode: the take-off point, an…, explore_cfg(), The live sim run: EKF2 drifted ~1.4 m over an 11 min exploration, the same wall…, Flown 2026-10-27: the east wall mapped a second time 1.4 m out (pose drift);…, test_exploration_survives_ekf_drift_without_false_obstacles(), test_explore_does_not_read_the_scene_files_room(), test_fit_puts_the_wall_on_its_dense_sheet_not_on_a_drifted_copy_beyond_it() (+1 more)

### Community 39 - "Depth and Snapshot Streams"
Cohesion: 0.29
Nodes (6): on_frame(), writer(), ndarray, Path, <run>/depth/<idx>.npy: float32 metres along the optical z, registered to the…, to_bgr()

### Community 40 - "Room Photogrammetry Review"
Cohesion: 0.43
Nodes (7): Plan3D, Acquisition variables: GSD, parallax, overlap, image count, speed, drift, FC-Planner / PredRecon / SOAR, FUEL frontier planner, ICON drone (Zhang, Yang, Zou), Krisshnakumar et al. IROS 2026 time-budgeted 3DGS exploration, Room photogrammetry open problem: fewest images / shortest flight for stated accuracy

### Community 41 - "Proxy Validation (V3)"
Cohesion: 0.33
Nodes (6): main(), Path, {N: (coverage, min overlap of frames 2..N with what came before)} at GT poses., replay(), spearman(), validate()

### Community 42 - "ZED Depth Study"
Cohesion: 0.33
Nodes (7): ZED SVO recording for real-data depth study, ADR-K10 Pools carry real ZED SDK depth, Depth study (ZED SDK vs FoundationStereo vs DA-V2 vs GT), FoundationStereo, Hypothesis H1: stereo helps planning more than final accuracy, Research questions RQ1-RQ5, Stereo hardware does not break monocular plan

### Community 43 - "Benchmarks and Venues"
Cohesion: 0.38
Nodes (6): GenNBV, GenNBV (CVPR 2024, Isaac Gym), Houses3K / OmniObject3D / Objaverse datasets, ObjView-Bench (Bonn, arXiv 2605.10707), Release norms of impactful NBV papers, Discrete view-pool replay protocol

### Community 44 - "Work Plan Phases"
Cohesion: 0.52
Nodes (6): Work plan Phase 1: Foundation (weeks 1-4), Phase 2: Core algorithm (weeks 5-12), Phase 3: Optimization and efficiency (weeks 13-18), Phase 4: Evaluation (weeks 19-24), Phase 5: Paper writing (weeks 25-28), Isaac Sim + Pegasus + PX4 SITL + MAVROS stack

### Community 46 - "Import Boundary Test"
Cohesion: 0.50
Nodes (4): ast, _imports(), Who may import whom (architecture.md §4, collaboration.md): the two areas meet…, test_import_boundaries()

### Community 47 - "Setup and Environment"
Cohesion: 0.40
Nodes (4): Machine .env settings (KINETIX_DATA, CUDA_VISIBLE_DEVICES), Per-phase extras: gsplat, COLMAP Docker, FoundationStereo, Pre-sim-run checklist (stop lc-chat, bisg check/smoke), uv-managed Python environment (uv.lock)

### Community 48 - "Cube Mesh Test"
Cohesion: 0.50
Nodes (4): Path, A 1 m cube in millimetres, outward winding, centred at the origin, as an…, test_build_cube(), write_cube_gltf()

### Community 49 - "Data Prune Workflow"
Cohesion: 0.67
Nodes (3): Run data release assets (data/<run>/run.yaml), data/sync.sh release sync (prune), data prune GitHub workflow

## Knowledge Gaps
- **79 isolated node(s):** `kinetix`, `render_server.sh script`, `SCENES_DIR`, `NBV utility (gain - travel - yaw cost)`, `Liu et al. 2026 (surface-normal NBV)` (+74 more)
  These have ≤1 connection - possible missing edges or undocumented components. (Counts symbols only; 329 node(s) total have ≤1 connection when file, concept and rationale nodes are included.)
- **24 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `OccupancyMap` connect `Occupancy Map` to `Room Tour Planner`, `Rig Adapters and Drift`, `Config and Session Loop`, `Exploration Tests`, `NBV Bench and Explore`, `Exploration Planner`, `Explore Tests`, `Room Fit`, `Geometry and Scorers`?**
  _High betweenness centrality (0.050) - this node is a cross-community bridge._
- **Why does `Pose` connect `Rig Adapters and Drift` to `Room Tour Planner`, `SfM and Bench Harness`, `Coverage Model`, `Core Types and Validation`, `MAVROS Rig`, `Exploration Planner`, `Explore Tests`, `Occupancy Map`, `Proxy Validation (V3)`, `Geometry and Scorers`, `Dry Rig Motion`, `Tour Bench`?**
  _High betweenness centrality (0.046) - this node is a cross-community bridge._
- **Why does `Intrinsics` connect `Config and Session Loop` to `Room Tour Planner`, `SfM and Bench Harness`, `Rig Adapters and Drift`, `Coverage Model`, `Core Types and Validation`, `MAVROS Rig`, `Exploration Planner`, `Explore Tests`, `Occupancy Map`, `Rig Liveness Tests`, `Proxy Validation (V3)`, `Geometry and Scorers`, `Dry Rig Motion`, `Kinetix CLI`, `Tour Bench`?**
  _High betweenness centrality (0.032) - this node is a cross-community bridge._
- **Are the 21 inferred relationships involving `Pose` (e.g. with `OccupancyMap` and `depth_points()`) actually correct?**
  _`Pose` has 21 INFERRED edges - model-reasoned connections that need verification._
- **Are the 15 inferred relationships involving `Intrinsics` (e.g. with `OccupancyMap` and `depth_points()`) actually correct?**
  _`Intrinsics` has 15 INFERRED edges - model-reasoned connections that need verification._
- **Are the 24 inferred relationships involving `CoverageState` (e.g. with `depth_surface()` and `run_tour()`) actually correct?**
  _`CoverageState` has 24 INFERRED edges - model-reasoned connections that need verification._
- **What connects `kinetix`, `render_server.sh script`, `SCENES_DIR` to the rest of the system?**
  _79 weakly-connected nodes found - possible documentation gaps or missing edges._