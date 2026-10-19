# Graph Report - kinetix  (2026-10-20)

## Corpus Check
- 60 files · ~59,249 words
- Verdict: corpus is large enough that graph structure adds value.
- Unclassified: 5 file(s) not represented in the graph (top: (none) 2, .example 1, .xml 1)

## Summary
- 1106 nodes · 3003 edges · 51 communities (45 shown, 6 thin omitted)
- Extraction: 70% EXTRACTED · 30% INFERRED · 0% AMBIGUOUS · INFERRED: 910 edges (avg confidence: 0.88)
- Token cost: 0 input · 0 output

## Community Hubs (Navigation)
- MavrosRig & Rig Config
- Room Capture & SfM Pipeline
- Benchmarks, Venues & Scene Licences
- Planner Budget & Coverage Config
- Constraints & Layered Config
- Aerial Active Reconstruction Literature
- Radiance-Field NBV Literature
- Orbit Baseline & Demo Scenes
- Tour Method (ADR-K12)
- Foundation Models in Planning
- Geometry & Constraints Code
- CLI, Coverage Code & Room Tests
- Module Map & Config Code
- Rig Protocol & Frame Types
- Phases, Todo & Scope Rules
- bisg Platform & MAVROS Rig
- Related Work & ActMVS
- ActiveGS & Splatting Baselines
- NBV Scorers & Open Questions
- Contribution C3: N@q Metric
- Architecture & Closed Loop
- Mono vs Stereo Study
- Baseline Families (FisherRF)
- Claude Instructions & Data Rules
- Contribution C1: SfM-Anchored NBV
- Contribution C2: Depth Study
- Environment Setup & check_env
- Image-Efficiency Literature
- Publication Plan & Benchmark Protocol
- Workspace Sampler & Core Types
- Execution Tiers & View Pools
- Sensing Modes (ADR-K07)
- Mono vs Stereo Decisions
- Learned Reconstructability Papers
- Coverage Path Planning Papers
- Kinetix Idea & Applications
- VGGT & Feed-Forward 3D
- Testing & ReconState
- Stereo Depth Source (Q6)
- Run Log Format
- Offline Pipeline & Densify
- Box Geometry
- NeRF & Gaussian Splatting Papers
- Coverage Tests & Look-At
- Dependency Rule
- Package Root

## God Nodes (most connected - your core abstractions)
1. `MavrosRig` - 53 edges
2. `CoverageState` - 41 edges
3. `Pose` - 38 edges
4. `run_sfm()` - 35 edges
5. `run_session()` - 28 edges
6. `Candidate` - 26 edges
7. `BoxConstraints` - 25 edges
8. `camera_pose()` - 24 edges
9. `Intrinsics` - 23 edges
10. `Field splits into four families: offline view selection in trained NeRF/3DGS; online RGB-D active mapping; learned object NBV policies; classic geometry-prediction NBV` - 23 edges

## Surprising Connections (you probably didn't know these)
- `rig.speed_mps and dry_noise_m: dry-rig time estimate and noise` --references--> `DryRig`  [INFERRED]
  config/default.yaml → src/kinetix/rig/dry.py
- `standby: camera 1.2 m above spawn` --references--> `MavrosRig`  [INFERRED]
  config/scenes/warehouse_room.yaml → src/kinetix/rig/mavros.py
- `run command: python3 -m kinetix run --scene ... --rig mavros` --conceptually_related_to--> `kx()`  [INFERRED]
  docker/compose.yaml → scripts/room_scan.sh
- `Before a sim run (stop lc-chat, bisg check/smoke)` --conceptually_related_to--> `sim_up()`  [INFERRED]
  docs/setup.md → scripts/room_scan.sh
- `K1 Core + T0 skeleton` --references--> `cmd_run()`  [INFERRED]
  docs/roadmap.md → src/kinetix/cli.py

## Import Cycles
- None detected.

## Hyperedges (group relationships)
- **Rig adapters across execution tiers** — docs_architecture_pool_rig, docs_architecture_isaac_render_rig, docs_architecture_mavros_rig, src_kinetix_rig_base_rig [EXTRACTED 1.00]
- **Room tour pipeline (plan, order, fly, reconstruct)** — docs_architecture_tour_viewpoints, docs_architecture_tour_order, docs_architecture_tour_fly, docs_architecture_tour_offline_sfm, src_kinetix_planner_tour [EXTRACTED 1.00]
- **C1 claim, anchoring ablation and K3 go/no-go** — docs_publication_plan_c1, docs_benchmark_anchoring_ablation, docs_roadmap_k3, docs_publication_plan_c1_gonogo, docs_decisions_adr_k08 [INFERRED 0.85]
- **Tour method: viewpoint set cover + TSP order + continuous keyframe capture** — config_room_tour, config_room_tour_cover, config_room_tour_cost, config_room_keyframes, src_kinetix_planner_tour_select_viewpoints, src_kinetix_planner_tour_order_tour, src_kinetix_planner_tour_sample_path [EXTRACTED 0.95]
- **Baseline matrix reviewers expect** — docs_research_reports_kinetix_novelty_search_fixed_patterns, docs_research_reports_kinetix_novelty_search_fvs, docs_research_reports_kinetix_novelty_search_volumetric_ig, docs_research_reports_kinetix_novelty_search_fisherrf, docs_research_reports_kinetix_novelty_search_learned_planner [EXTRACTED 0.95]
- **Sensing 2x2 design: plan_depth x recon_input presets answering RQ2/H1** — docs_research_mono_vs_stereo_plan_depth, docs_research_mono_vs_stereo_recon_input, docs_research_mono_vs_stereo_presets, docs_research_mono_vs_stereo_h1, docs_research_mono_vs_stereo_rq2 [EXTRACTED 0.95]

## Communities (51 total, 6 thin omitted)

### Community 0 - "MavrosRig & Rig Config"
Cohesion: 0.05
Nodes (52): Kinetix defaults (config/default.yaml), rig.auto_world_to_map: measure map-world offset on the ground, rig.bag: MCAP flight recording profile, camera: nominal ZED Mini 720p intrinsics and mount, rig.cruise_mps: carrot speed (VIO-safe), rig.speed_mps and dry_noise_m: dry-rig time estimate and noise, rig.max_lead_m: path clock pauses when setpoint lags, camera.mount_pitch_deg: level forward-looking mount (+44 more)

### Community 1 - "Room Capture & SfM Pipeline"
Cohesion: 0.06
Nodes (63): Rooms: method tour + kinetix sfm, MCAP rosbag per flight, Metric frame via Sim3 alignment to rig poses, Scene kinds: object vs room, recon.pose_source (sfm | prior | gt) ablation axis, Room capture: method tour, Scene file constraints (roi, workspace, no_fly, standoff, altitude), Tour step 3: continuous flight with keyframes (fly_path), Tour step 4: known-pose offline SfM (+55 more)

### Community 2 - "Benchmarks, Venues & Scene Licences"
Cohesion: 0.07
Nodes (56): source: Isaac Simple_Warehouse asset, not redistributable, Bag of Views (RA-L 2023) with the View Planning Toolbox, Experimental contribution (iii): four-tier realism protocol (replay, render, PX4 SITL, real flight), GenNBV (CVPR 2024): depth-based RL NBV in Isaac Gym, sim only, One learned planner: ma_scvp preferred, gennbv as drone-flavoured alternative, MA-SCVP (T-RO 2025): public learned object-NBV baseline of the Bonn group, ObjView-Bench argues deployment constraints change method rankings: Kinetix tiered-protocol argument, but for a tabletop arm; code only on acceptance, Open: tiered photoreal-replay -> render -> PX4 SITL -> real-flight protocol measuring rank shifts (+48 more)

### Community 3 - "Planner Budget & Coverage Config"
Cohesion: 0.06
Nodes (44): planner.budget: image budget (20), coverage: GT-free ROI-proxy coverage model, planner.coverage_target: stop at 95 % triangulated, coverage.depth_range_m: usable depth window, coverage.max_incidence_deg: seen only below this angle to normal, coverage.max_views per proxy point, coverage.novel_deg: extra rays must be novel, planner: NBV loop parameters (+36 more)

### Community 4 - "Constraints & Layered Config"
Cohesion: 0.07
Nodes (47): planner.clearance_m and roi_clearance_m: no-fly and ROI keep-out, Layered config: default < --config files < key=value overrides, planner.rings: ring-stack candidates around ROI vertical axis, coverage.spacing_m: surface sample spacing on ROI box, planner.clearance_m 0.8 around no-fly boxes, planner.grid: GridSampler candidates filling the workspace, Room layer (ADR-K11): applied after default.yaml for kind: room scenes, coverage.spacing_m 0.5: coarser room surface sampling (~8.5 k points) (+39 more)

### Community 5 - "Aerial Active Reconstruction Literature"
Cohesion: 0.09
Nodes (48): COLMAP / Structure-from-motion revisited (Schonberger & Frahm, CVPR 2016) [1], FlyCo: foundation-model drones for 3D structure scanning [36], OpenDroneMap vs Metashape accuracy benchmark (Kostrzewa et al.) [49], Kriegel et al.: next-best-scan planning with robotic arms [32], On-the-fly feedback SfM (Lou et al.) [3], Mostegel et al. 2016: UAV acquisition with MVS confidence prediction [2], OpenDroneMap / WebODM [4], OpenSfM [5] (+40 more)

### Community 6 - "Radiance-Field NBV Literature"
Cohesion: 0.12
Nodes (44): NARUTO (arXiv 2402.18771): neural active reconstruction, RGB-D, Radiance-field / 3DGS-based active view selection and NBV (2022-2026): novelty check for Kinetix, Active Neural Mapping (ICCV 2023): neural variability, RGB-D, ActiveGS (RA-L 2025): hybrid 3DGS + voxel map, RGB-D, real UAV, ActiveNeRF (ECCV 2022): radiance variance, offline candidate pool, ActiveSplat (RA-L 2025): scene-level active Gaussian splatting, AG-SLAM (arXiv 2410.17422): active 3DGS SLAM with Fisher information, Auto3R (arXiv 2512.04528): robot-arm real deployment (+36 more)

### Community 7 - "Orbit Baseline & Demo Scenes"
Cohesion: 0.08
Nodes (39): planner.orbit: orbit baseline ring, Scene simple_room_blocks: first demo object scene in Isaac Simple Room, Scene warehouse_room: whole-room photogrammetry of the Isaac Simple_Warehouse (ADR-K11), service kinetix: image kinetix:jazzy built from repo Dockerfile, random scorer, Component registry by name, Sampler protocol, Scorer protocol (uses_gt flag) (+31 more)

### Community 8 - "Tour Method (ADR-K12)"
Cohesion: 0.10
Nodes (38): keyframe_step_m 0.4 / 6 deg: sequential overlap for SfM, planner.tour (ADR-K12): viewpoints + TSP + continuous capture, tour.yaw_cost_m / path_step / keyframe_step: TSP turn cost and carrot path resolution, tour.coverage_target 0.8 set cover; path fills the rest (dry 0.91), select: gain minus lambda times travel cost, camera_yaw(), Heading of the optical axis projected on the ground plane., wrap() (+30 more)

### Community 9 - "Foundation Models in Planning"
Cohesion: 0.12
Nodes (37): AREA3D (CVPR 2026): active agent on feed-forward 3D perception, Foundation depth / pointmap models inside active reconstruction, NBV and exploration (2023-2026): novelty check for Kinetix, ACT-R (3DV 2026, arXiv 2505.08239): adaptive orbit + video diffusion, virtual view planning, ActMVS (ICRA 2026, arXiv 2606.01367): monocular MVS aerial scene reconstruction, Replica, code TrickyGo/ActMVS, AIM-SLAM (arXiv 2603.05097): keyframe choice by overlap and information gain, AREA3D (CVPR 2026, arXiv 2512.05131): active agent on feed-forward 3D perception + VLM guidance, Auto3R (arXiv 2512.04528): learned viewpoint uncertainty, real robot arm, Depth Anything 3 (arXiv 2511.10647): beats DA2 and VGGT; RANSAC scale-shift alignment to sparse depth (+29 more)

### Community 10 - "Geometry & Constraints Code"
Cohesion: 0.11
Nodes (30): collections, 4-DoF camera pose with fixed mount pitch, Feasibility constraints (constraints.py), Constraint protocol, signal, body_from_camera(), camera_from_body(), camera_pose() (+22 more)

### Community 11 - "CLI, Coverage Code & Room Tests"
Cohesion: 0.12
Nodes (25): argparse, ast, run command: python3 -m kinetix run --scene ... --rig mavros, cmd_run(), main(), `python -m kinetix run ...` (interfaces.md §6; `uv run kinetix` once…, load_scene(), Intrinsics (+17 more)

### Community 12 - "Module Map & Config Code"
Cohesion: 0.13
Nodes (20): copy, dataclasses, Module map (src/kinetix layout), K1 Core + T0 skeleton, json, numpy, pathlib, platform (+12 more)

### Community 13 - "Rig Protocol & Frame Types"
Cohesion: 0.12
Nodes (12): Protocol, Frame, Pose, covis_pairs(), Pairs for a continuous capture (method `tour`): each frame with its `window`…, (image_i, image_j, shared) for frame pairs that co-observe >= min_shared…, video_pairs(), Rig protocol (interfaces.md §3). The planner imports only this module; adapters… (+4 more)

### Community 14 - "Phases, Todo & Scope Rules"
Cohesion: 0.13
Nodes (24): Build phase by phase (K0-K6), do not widen scope, Q4 ground truth for real objects, Stop-and-shoot capture, ADR-K06 Stop-and-shoot capture in v1, RigError kinds (unreachable, timeout, no_image, aborted), Publication plan, C1 go/no-go at G2, C5 Real-flight evaluation and release (+16 more)

### Community 15 - "bisg Platform & MAVROS Rig"
Cohesion: 0.18
Nodes (23): bisg_isaac is the platform: use its contract, never copy code, MavrosRig (T2/T3 flight rig), Q3 scene mount into bisg sim, System context: bisg_isaac platform, ADR-K01 Reuse bisg_isaac through its interface contract only, ADR-K09 ROS 2 pipeline: ROS at the edges, goto action + snapshot service (proposed), capture_at settle sequence, Rig protocol and RigCaps (+15 more)

### Community 16 - "Related Work & ActMVS"
Cohesion: 0.17
Nodes (23): Related Work, ActMVS: monocular active scene reconstruction [35], ActMVS: replacing MVS depth with single-image depth raises Chamfer ~2.4 cm -> ~46 cm, FisherRF: RGB-only view selection via 3DGS Fisher information [48], FrontierNet: frontiers from posed RGB + monocular metric depth [39], Gap 1: scale from the reconstruction itself: anchor relative depth to online SfM, no depth sensor or metric-depth model (C1), Gap 2: what depth is for: one fixed NBV planner swapped between monocular, small-baseline stereo and RGB-D, 2x2 design (C2), Gap 3: images-to-quality: N@q against a dense capture of the same object, geometric F-score (C3) (+15 more)

### Community 17 - "ActiveGS & Splatting Baselines"
Cohesion: 0.14
Nodes (22): ActiveGS (2025), ActiveGS (RA-L 2025): RGB-D active scene reconstruction with 3DGS, ActMVS (ICRA 2026): monocular active scene reconstruction, view factor graph, Bonn group (SCVP, MA-SCVP, ObjView-Bench, DM-OSVP++, ActiveGS) is the likely reviewer pool: frame Kinetix as the aerial, photoreal, monocular-SfM complement, First monocular active (aerial) reconstruction: TAKEN for scenes, First monocular depth foundation model in NBV/exploration: TAKEN, RGB-only object NBV on a real drone: TAKEN (weakly), qualitative, First RGB-only object NBV: TAKEN (+14 more)

### Community 18 - "NBV Scorers & Open Questions"
Cohesion: 0.16
Nodes (21): NBV scorers table, Open questions Q1-Q6, Q1 ZED mount pitch, Q2 object set, Q5 drone pose as a prior in the mono claim, combo scorer (Kinetix method), coverage_oracle scorer (GT upper bound), depth_frontier scorer (+13 more)

### Community 19 - "Contribution C3: N@q Metric"
Cohesion: 0.16
Nodes (21): C3: N@q images-to-quality metric normalised to a dense capture, Kinetix wins on decomposition, not on savings, C3: N@q images-to-quality metric alongside Chamfer, F-score and coverage AUC, C4: four-tier evaluation protocol quantifying rank and saving shifts as realism increases, C5: real-flight evaluation on PX4/Jetson with scanned GT; released view pools, twins, code and baselines, Five claims remain open (absence-of-evidence findings), Conclusion: a measurement paper with a principled monocular method attached, Recommended contribution list C1-C5 for an RA-L submission (+13 more)

### Community 20 - "Architecture & Closed Loop"
Cohesion: 0.13
Nodes (20): Kinetix architecture, Baselines are configurations of the same loop, Closed NBV capture loop, Design goals (one planner, minimal, modular, measurable), Session loop (run_session), Reporting rules, Seeds (3 for K2-K3, 5 for paper), Benchmark directory (matrix.yaml, results.csv, report.md) (+12 more)

### Community 21 - "Mono vs Stereo Study"
Cohesion: 0.16
Nodes (20): ZED Mini stereo camera (63 mm baseline) as comparison sensor, Study: monocular vs stereo in Kinetix, ActMVS (Pu, Han, Lian; ICRA 2026): closest monocular competitor, Depth study: ~50 views, abs-rel / RMSE / % within 1 cm binned by distance and surface; noise model sigma(z) per source; reuses bisg zed_depth_box approach, FlyCo (arXiv 2601.07558): outdoor foundation-model drone scanning, FoundationStereo (CVPR 2025): zero-shot stereo, offline stereo depth source for T0 pools, H1: stereo depth helps planning much more than final accuracy, which multi-view SfM/MVS dominates, Kotlyar close-range study (arXiv 1903.09169): ZED denser than active stereo at 0.6 m but ~2x RMSE (+12 more)

### Community 22 - "Baseline Families (FisherRF)"
Cohesion: 0.13
Nodes (19): FisherRF (ECCV 2024), PUN: half the views [45], Four baseline families reviewers expect; current matrix covers only fixed patterns, 50-80 % fewer images than fixed patterns: TAKEN, Gap: none of the surveyed papers evaluates the selected image set through unmodified COLMAP SfM->MVS against a GT mesh, Delmerico et al.: volumetric information-gain planner family and open ROS framework, FisherRF (ECCV 2024): RGB-only view selection by Fisher information over a candidate pool, Farthest-view sampling baseline (fvs): cheap and strong, 70 views vs 150 random (+11 more)

### Community 23 - "Claude Instructions & Data Rules"
Cohesion: 0.14
Nodes (18): Kinetix instructions for Claude Code, Large data in $KINETIX_DATA, only tests/fixtures in git, Change interfaces.md first, then the code, Pose convention T_map_cam ENU OpenCV, convert in core.geometry, uv Python env, no host ROS, ROS in kinetix image, Workstation B: check nvidia-smi, stop lc-chat, Configuration layering, GPU budget on workstation B (+10 more)

### Community 24 - "Contribution C1: SfM-Anchored NBV"
Cohesion: 0.17
Nodes (18): C1: SfM-anchored monocular NBV (DA-V2 scale-anchored online to SfM, TSDF frontier + SfM uncertainty), Depth Anything V2 relative-depth foundation model, SfM-uncertainty score, TSDF frontier score from scale-anchored depth, Depth Anything: zero-shot depth from 60M+ images [30], Murre: SfM-guided monocular depth fused into TSDF [40], Kinetix SfM scale fit answers the unreliable-depth argument: paper must show anchoring makes the prior usable, C1: monocular NBV anchoring a relative-depth foundation model to SfM scale, TSDF frontier + SfM uncertainty, no depth sensor or metric model (+10 more)

### Community 25 - "Contribution C2: Depth Study"
Cohesion: 0.18
Nodes (18): C2: what is depth for? first controlled mono-vs-stereo (ZED Mini, 63 mm) study in one planner, 2x2 design, Maboudi et al.: review of UAV viewpoint and path planning [12], C2: first controlled study of mono vs small-baseline stereo within one NBV planner (2x2 presets mono/stereo_plan/stereo_full/oracle_depth), Experimental contribution (i): same-planner mono vs 63 mm stereo, depth for planning vs for reconstruction (2x2), H1 prediction: stereo helps planning far more than final accuracy (falsifiable), Next-Best Stereo (BMVC 2016): 3.8-4.4 % of Middlebury views, Open: same-planner mono vs real small-baseline stereo, 2x2 of plan-depth x recon-input, Plan3D (Hepp et al., TOG 2018): GT-depth vs SfM/MVS reconstruction split (+10 more)

### Community 26 - "Environment Setup & check_env"
Cohesion: 0.12
Nodes (7): K0 Foundations, Python environment (uv sync, check_env, uv.lock pins), kinetix, Quick start (uv sync, check_env), Verify the Kinetix Python environment: each library is imported *and* exercised…, shutil, sys

### Community 27 - "Image-Efficiency Literature"
Cohesion: 0.23
Nodes (16): Mono vs stereo vs RGB-D in active reconstruction, and image-set minimisation for SfM/MVS (Kinetix novelty check), ActMVS (arXiv 2606.01367): compared with RGB-D ActiveGS and NARUTO, Baseline choice inflates or deflates the reduction: report against the strongest fixed pattern (dense multi-ring orbit) and a strong adaptive baseline, ActMVS "w/o MVS" is a controlled depth-source ablation inside one planner (Chamfer 2.4 -> 46 cm), but not stereo vs RGB-D, Mono-vs-stereo novelty is experimental-design, not algorithmic; Kinetix targets one object with COLMAP SfM->MVS/3DGS output, Recommended framing: lead with the modality comparison and the depth decomposition; N@q as main efficiency measure; savings as consistent with prior reports, Open: controlled mono vs stereo (ZED Mini 63 mm) study, planning-vs-reconstruction depth 2x2, N@q, indoor single-object small-drone COLMAP + 3DGS output, Knapitsch et al. 2017: F-score@tau (Tanks and Temples) (+8 more)

### Community 28 - "Publication Plan & Benchmark Protocol"
Cohesion: 0.26
Nodes (15): Paper claims C1-C5; never call image saving a novelty, Benchmark protocol, Dense reference F_ref, N@q images-to-quality headline metric, N@q sensitivity to reference density (120/240/360), Comparability with prior tables (coverage AUC, K=5/30), Tier transfer metric (Kendall tau, delta N@q), ADR-K08 Measurement paper positioning, stronger baselines (+7 more)

### Community 29 - "Workspace Sampler & Core Types"
Cohesion: 0.25
Nodes (11): workspace: camera >= 1.5 m from walls, east of racks, 4 m tall, Core types (Pose, Intrinsics, Frame, Candidate, Box, Scene), ReconState API, Candidate, RigCaps, Scene, grid_poses(), GridSampler (+3 more)

### Community 30 - "Execution Tiers & View Pools"
Cohesion: 0.21
Nodes (15): Execution tiers T0-T3, IsaacRenderRig (T1 render rig), PoolRig (T0 replay rig), View pool (T0), Pools rings:4x60 and tammes:128, reachable flag and test split, View pool directory (pool.yaml, views.jsonl, images, right, depth, zed), views.jsonl (one view per line) (+7 more)

### Community 31 - "Sensing Modes (ADR-K07)"
Cohesion: 0.30
Nodes (14): Sensing modes rule (mono method, stereo comparison, never ZED point cloud/odom), sensing.plan_depth axis, sensing.recon_input axis, Sensing modes: monocular method, stereo comparison, Sensing presets (mono, mono_noprior, stereo_plan, stereo_full, oracle_depth), Mono vs stereo measures (planning value, reconstruction value, gap closure), Sensing rows (mono ... oracle_depth, zed_mapping), Decisions (ADRs) (+6 more)

### Community 32 - "Mono vs Stereo Decisions"
Cohesion: 0.23
Nodes (13): ADR-K07: decisions that follow from this study, sensing.plan_depth: depth used for planning (none, mono DA-V2 scale-fit, stereo ZED SDK or FoundationStereo, gt oracle), mono: plan_depth mono, recon left (main method), mono_noprior: does a mono depth prior help planning at all?, oracle_depth: upper bound of depth-assisted planning (gt depth), stereo_full: best the ZED can do with our planner, stereo_plan: how much does stereo help planning? (tests H1), Presets: mono (Kinetix), mono_noprior, stereo_plan, stereo_full, oracle_depth, zed_mapping baseline (+5 more)

### Community 33 - "Learned Reconstructability Papers"
Cohesion: 0.26
Nodes (13): Liu et al. 2022: learned reconstructability, 764 vs 1,610 images at equal F-score, ActiveInitSplat (arXiv 2503.06859): GP surrogate for few-view 3DGS selection, CMVS / Furukawa 2010: redundant-image removal, unverified, DynaView (2026): Transformer reconstructability predictor + online marginal-utility updates, Most aerial planners fix budget and report quality; Kinetix fixed-quality, report-images framing is less common, Hornung, Zeng, Kobbelt CVPR 2008: image selection for improved MVS with proxy coverage, Taken: monocular online-MVS planning, large image reductions, reconstructability planners, offline image selection, classic network design, metric conventions, Liu et al. 2022: 764 images F 34.96 vs Zhou planner 1610 images F 34.55 (~53 % fewer) (+5 more)

### Community 34 - "Coverage Path Planning Papers"
Cohesion: 0.30
Nodes (12): Bahnemann et al.: boustrophedon CPP as generalized TSP, 14 % lower path cost [7], CPP methods are open-loop, model-based and cannot react to reconstruction quality; Kinetix replaces fixed coverage geometry with uncertainty-driven NBV and needs no prior 3D model, Adaptive CPP: explore-then-exploit builds a coarse proxy first, then targets low-quality regions, Galceran & Carreras: survey on coverage path planning [6], Gap 6: CPP vs NBV: CPP plans globally but cannot react; image reduction reported as a result consistent with prior ~53-72 %, Jing et al.: multi-UAV CPP for large structures [9], Liu et al.: ~53 % fewer images at equal F-score [43], Peng & Isler: adaptive view planning for aerial 3D reconstruction, explore-then-exploit [10] (+4 more)

### Community 35 - "Kinetix Idea & Applications"
Cohesion: 0.31
Nodes (10): Kinetix: Adaptive Dynamic Photogrammetry for Indoor Objects with Monocular Drone Imaging, Target applications: museums, warehouses, archaeology, e-commerce product modelling, C4: tiered realism protocol (view-pool replay, continuous render, PX4 SITL, real flight), C5: quantitative real flights: indoor PX4/Jetson drone, scanned GT, unmodified COLMAP -> MVS/3DGS, released pools/twins/code, Unmodified COLMAP -> MVS/3DGS output pipeline, Core problem: fixed paths need hundreds of overlapping images indoors; long flights, energy, processing; generic paths miss optimal angles, Indoor constraints (collision, standoff, fixed camera mount) are part of the method, not a contribution, Active vision framework: minimal seed images, real-time depth/uncertainty/feature analysis predicts the Next Best View, dynamic path and capture parameters (+2 more)

### Community 36 - "VGGT & Feed-Forward 3D"
Cohesion: 0.27
Nodes (10): VGGT (CVPR 2025): feed-forward cameras, depth and points in <1 s; K5 candidate, Feed-forward 3D model + kinematic scale for NBV: TAKEN on an arm, Depth Anything 3: reported to beat DA-V2 and VGGT, Depth Pro: monocular metric depth, FrontierNet (RA-L 2025): frontiers and information gain from posed RGB + Metric3D v2, Classical planners degrade significantly with predicted depth (FrontierNet), GraspView: VGGT + kinematic-scale NBV on an arm, Metric3D v2: monocular metric depth (+2 more)

### Community 37 - "Testing & ReconState"
Cohesion: 0.39
Nodes (9): Tests: uv run pytest (unit + t0), ReconState online reconstruction, Stereo rig reconstruction (pycolmap Rig), Testing, Component tests (t0), Test levels (unit, t0, t1, t2, t3, bench regression), t1 render integration tests, t2 flight integration tests (+1 more)

### Community 38 - "Stereo Depth Source (Q6)"
Cohesion: 0.36
Nodes (9): Q6 stereo depth on T0 pools, Stereo depth source per tier (ZED SDK vs FoundationStereo), zed/ and zed_conf/ SDK depth + confidence maps, ADR-K10 Pools carry real ZED SDK depth rendered through the twin (proposed), DepthSource protocol, Depth confidence for stereo_zed, SDK depth on pools probe (Q6), SVO recording for T2/T3 (+1 more)

### Community 39 - "Run Log Format"
Cohesion: 0.33
Nodes (4): frames.jsonl, _git(), Path, RunLog

### Community 40 - "Offline Pipeline & Densify"
Cohesion: 0.39
Nodes (8): Densify backends (colmap_mvs, gsplat, tsdf_depth), Offline pipeline identical for every method, Export to COLMAP / nerfstudio / gsplat, ADR-K04 pycolmap as the only SfM backend, COLMAP format everywhere, Densifier protocol, K2 Evaluation + benchmark harness, Extras planned per phase (gsplat, colmap, FoundationStereo, kinetix image), Dense reconstruction for real completeness

### Community 41 - "Box Geometry"
Cohesion: 0.29
Nodes (5): _box(), Box, ndarray, For segments o -> P[i] (vectorised slab test): does the segment pass through…, segments_hit_box()

### Community 42 - "NeRF & Gaussian Splatting Papers"
Cohesion: 0.48
Nodes (7): 3D Gaussian Splatting [26], Instant-NGP [25], NeRF [24], pixelNeRF: few-shot view synthesis [27], Monocular 3D reconstruction: NeRF, Instant-NGP, 3DGS, sparse-view methods, depth foundation models, SparseNeRF: depth-ranking distillation for 3-5 views [28], Zero-1-to-3: diffusion novel views from a single image [29]

### Community 43 - "Coverage Tests & Look-At"
Cohesion: 0.57
Nodes (6): look_at_yaw(), frame(), test_coverage_grows_around_the_object(), test_one_view_sees_one_face_and_triangulates_nothing(), test_second_view_from_new_angle_triangulates_and_has_highest_gain(), view()

### Community 44 - "Dependency Rule"
Cohesion: 0.47
Nodes (6): Dependency rule: planner/core never import rclpy, Dependency rule (planner/core never import rclpy), Architecture import-rule test, planner/ and core/ never import rclpy, rig adapters or offline/…, test_dependency_rule(), test_offline_never_imports_the_planner()

## Knowledge Gaps
- **1 isolated node(s):** `gt_mesh null: no GT mesh for the Warehouse`
  These have ≤1 connection - possible missing edges or undocumented components. (Counts symbols only; 130 node(s) total have ≤1 connection when file, concept and rationale nodes are included.)
- **6 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `MavrosRig` connect `MavrosRig & Rig Config` to `Benchmarks, Venues & Scene Licences`, `Kinetix Idea & Applications`, `Constraints & Layered Config`, `Aerial Active Reconstruction Literature`, `Foundation Models in Planning`, `Geometry & Constraints Code`, `CLI, Coverage Code & Room Tests`, `Rig Protocol & Frame Types`, `bisg Platform & MAVROS Rig`, `Related Work & ActMVS`, `Contribution C3: N@q Metric`, `Mono vs Stereo Study`, `Workspace Sampler & Core Types`?**
  _High betweenness centrality (0.133) - this node is a cross-community bridge._
- **Why does `CoverageState` connect `CLI, Coverage Code & Room Tests` to `Room Capture & SfM Pipeline`, `Learned Reconstructability Papers`, `Planner Budget & Coverage Config`, `Testing & ReconState`, `Radiance-Field NBV Literature`, `Tour Method (ADR-K12)`, `Box Geometry`, `Coverage Tests & Look-At`, `Module Map & Config Code`, `Rig Protocol & Frame Types`, `Image-Efficiency Literature`, `Workspace Sampler & Core Types`?**
  _High betweenness centrality (0.113) - this node is a cross-community bridge._
- **Why does `run_sfm()` connect `Room Capture & SfM Pipeline` to `Kinetix Idea & Applications`, `Constraints & Layered Config`, `Aerial Active Reconstruction Literature`, `Radiance-Field NBV Literature`, `Orbit Baseline & Demo Scenes`, `Tour Method (ADR-K12)`, `Foundation Models in Planning`, `CLI, Coverage Code & Room Tests`, `Module Map & Config Code`, `Rig Protocol & Frame Types`, `Related Work & ActMVS`, `Contribution C3: N@q Metric`, `Mono vs Stereo Study`, `Baseline Families (FisherRF)`?**
  _High betweenness centrality (0.090) - this node is a cross-community bridge._
- **Are the 21 inferred relationships involving `MavrosRig` (e.g. with `rig.ns /drone_1 and camera_frame zed_left_camera_frame_optical` and `rig overrides: move_timeout 120 s, cruise_mps 1.0`) actually correct?**
  _`MavrosRig` has 21 INFERRED edges - model-reasoned connections that need verification._
- **Are the 25 inferred relationships involving `CoverageState` (e.g. with `ReconState online reconstruction` and `ReconState API`) actually correct?**
  _`CoverageState` has 25 INFERRED edges - model-reasoned connections that need verification._
- **Are the 12 inferred relationships involving `Pose` (e.g. with `camera_yaw()` and `BoxConstraints`) actually correct?**
  _`Pose` has 12 INFERRED edges - model-reasoned connections that need verification._
- **Are the 13 inferred relationships involving `run_sfm()` (e.g. with `keyframe_step_m 0.4 / 6 deg: sequential overlap for SfM` and `Run directory (run.yaml, frames.jsonl, steps.jsonl, sfm, rosbag)`) actually correct?**
  _`run_sfm()` has 13 INFERRED edges - model-reasoned connections that need verification._