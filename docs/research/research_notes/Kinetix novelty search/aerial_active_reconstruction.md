# Drone / aerial active 3D reconstruction and view/path planning (2022–2026) — novelty check for Kinetix

Scope: systems where a drone (or simulated aerial camera) actively picks views for 3D reconstruction. Kinetix: adaptive NBV photogrammetry of one indoor object (0.3–2 m), monocular RGB + drone pose for planning (ZED Mini stereo as a comparison), output stays compatible with COLMAP -> MVS / 3DGS, claim of 50–80 % fewer images than orbit/grid at equal quality, evaluated in Isaac Sim + PX4 SITL and on a real Jetson/PX4 drone indoors.

Verification legend: **[V]** = checked this session against the arXiv abstract / HTML full text (via fetch) or project text; **[S]** = only from a search-result snippet; **[M]** = from prior knowledge, not re-checked this session (treat as unverified). Searched 2026-10-08.

## Q1. Master list of the closest prior systems (with per-paper fields)

### Takeaway
Most 2022–2026 aerial active-reconstruction work targets buildings/urban scenes or indoor rooms, plans with LiDAR or RGB-D, and is validated only in simulation. Only a handful work at object scale with a real drone. The closest is Hestia (WACV 2026): a real Crazyflie with an RGB camera only, indoor objects, MASt3R-SfM. The older "toy drone" work (Huang et al.) is also close: a monocular Bebop, SfM/MVS, and indoor plus outdoor real flights.

### Cited Findings

**A. Object-scale / small-structure, real drone (closest)**

- **Hestia: Voxel-Face-Aware Hierarchical Next-Best-View Acquisition for Efficient 3D Reconstruction.** C.-Y. Lu, Z. Zhuang, N.T.T. Le, D. Xiao, Y.-C. Chang, T. Do, S. Sridhar, C.-T. Lin. WACV 2026, arXiv 2508.01014. [V] Learned (RL-type, hierarchical) NBV policy. It reports at least 12 % higher coverage ratio than prior methods and better Chamfer distance. — [arXiv abs](https://arxiv.org/abs/2508.01014)
  - Real drone: Crazyflie carrying an **RGB camera only, no depth sensor**. A learned depth predictor turns multi-view RGB into depth maps for the occupancy grid. The planner's inputs are the current grayscale image, the previous pose and that occupancy grid. Reconstruction uses **MASt3R-SfM**. Real-world tests are indoor with naturally sized objects (sim objects scaled to 8 m). The first three viewpoints are set by hand to align the real and virtual frames. Policy inference runs at 25 FPS. Real results are shown qualitatively ("visibly better" point clouds). [V] — [arXiv HTML](https://arxiv.org/html/2508.01014)
  - Localization by four HTC Lighthouse base stations on a Crazyflie 2.1. [S] — [search snippet of 2508.01014](https://arxiv.org/pdf/2508.01014)
  - Gap: no image-count reduction versus orbits was seen in the parts read. Its efficiency metric is coverage per view budget.
- **Active Image-based Modeling with a Toy Drone.** R. Huang, D. Zou, R. Vaughan, P. Tan. arXiv 1705.01010 (2017; published at ICRA 2018 [M]). A real Parrot Bebop streams 640×368 frames over WiFi for planning and stores 1920×1080 images on board. Pipeline: SfM → fast linear MVS → Poisson surface for the online model, then CMP-MVS for the final model. Initial capture is 64–81 images; 101–108 images after 5 iterations. Baselines are hand-designed flight plans (Grid-Downward, Grid-Multi-Angle, Rectangle). Tested in Gazebo, in an indoor Vicon room with cardboard structures, and outdoors (a 15 m gate). Scale comes from GPS outdoors and Vicon indoors. [V] — [ar5iv](https://ar5iv.arxiv.org/html/1705.01010)
  - This is the closest "monocular RGB + external pose → SfM/MVS, real drone, explore-then-refine" precedent, but it predates the 2022 window and is not object-centric NBV.

**B. Monocular-RGB active reconstruction (sim or real)**

- **ActMVS: Active Scene Reconstruction with Monocular Multi-View Stereo.** G. Pu, Y. Han, Z. Lian. arXiv 2606.01367 (June 2026). [V] It claims to be the "first framework for monocular active reconstruction" that builds a view factor graph for MVS depth plus a global depth optimization. — [arXiv pdf](https://arxiv.org/pdf/2606.01367); [HTML](https://arxiv.org/html/2606.01367)
  - RGB-only plus pose: poses come from the simulator, or from odometry on real robots. Metric scale comes from MVS with known camera motion. Tested in Habitat and AirSim (quadrotor). **No real-world deployment.** Scenes are indoor (Replica). Output: OctoMap occupancy, a Gaussian-splatting map and dense depth.
  - Compared with the RGB-D methods ActiveGS and NARUTO: PSNR 26.32–31.96 vs ActiveGS 28.14–34.62, accuracy 1.8–3.3 cm vs 0.90–1.28 cm. Runtime is about 1200 s vs 300 s for ActiveGS. It is competitive with RGB-D but still behind it. [V, numbers as extracted by the fetch tool]
- **Autonomous UAV 3D Reconstruction using Prediction-Based Next Best View.** Z. Wang, B. Alsadik, F. Nex. ISPRS Annals X-2/W2-2025. [V] Evaluates **MACARONS**, a self-supervised NBV method using monocular RGB only, on two bridge models rendered in Blender (synthetic only, no real flight). Reports 88 % coverage vs ground truth, 90 % coverage relative to traditional flight planning, and a "37.5 % efficiency improvement" over predefined flight paths. Output is a point cloud. — [ISPRS Annals](https://isprs-annals.copernicus.org/articles/X-2-W2-2025/207/2025/)
  - MACARONS itself is Guédon et al., CVPR 2023 [M]: monocular RGB, large scenes, simulation.
- **GenNBV: Generalizable Next-Best-View Policy for Active 3D Reconstruction.** X. Chen, Q. Li, T. Wang, T. Xue, J. Pang. CVPR 2024, arXiv 2402.16174. [V] RL policy with a 5D free-space action space for drone views, trained and evaluated in Isaac Gym. Coverage is 98.26 % on Houses3K and 97.12 % on OmniObject3D. The sensor modality is not stated in the abstract (depth-based according to [M]). Simulation only. — [arXiv abs](https://arxiv.org/abs/2402.16174)

**C. Building / outdoor-structure aerial planners (explore-then-exploit and successors)**

- **PredRecon.** C. Feng, H. Li, F. Gao, B. Zhou, S. Shen. ICRA 2023, arXiv 2302.04488. [V] A surface prediction module infers the coarse complete surface from a partial reconstruction. Hierarchical planner: global coverage path → local path optimized for MVS → smooth trajectory for image-pose capture. Validated in simulation; open source. The sensor (LiDAR/depth for mapping, camera for images [M]) is not stated in the abstract. — [arXiv abs](https://arxiv.org/abs/2302.04488)
- **SOAR.** M. Zhang, C. Feng, Z. Li, G. Zheng, Y. Luo, Z. Wang, J. Zhou, S. Shen, B. Zhou. IROS 2024, arXiv 2409.02738. [V] Heterogeneous UAVs: a wide-FoV **LiDAR explorer** plus camera "photographer" UAVs. Uses surface-frontier exploration, incremental viewpoint generation and CMD-mTSP task allocation. Images are rendered in Blender at 2 Hz and reconstructed in **RealityCapture**. **Simulation only (MARSIM)**; the authors list this as a limitation. — [arXiv](https://arxiv.org/abs/2409.02738); [HTML](https://arxiv.org/html/2409.02738)
  - Efficiency is measured by flight time and path length, not image count. Sydney: 196.2 s, 772.8 m, F-score 93.2 %. Pisa: 166.4 s, 724.5 m, F-score 90.1 %. Baselines: SSearchers (multi-robot Star-Searcher) and Multi-EE (explore-then-exploit with a RealityCapture coarse model). [V]
- **FlyCo: Foundation Model-Empowered Drones for Autonomous 3D Structure Scanning in Open-World Environments.** C. Feng, G. Zheng, T. Zhuang et al. arXiv 2601.07558 (Jan 2026). [V] Prompt-driven (text or visual) perception → prediction → planning loop. The drone carries a gimbal-mounted RGB camera and a **3D LiDAR**. The predicted watertight mesh comes from NKSR. **Real-world outdoor** flights at unmapped sites (arch bridge, concert hall, castle gate, brick building). — [arXiv abs](https://arxiv.org/abs/2601.07558); [HTML](https://arxiv.org/html/2601.07558)
  - Versus baselines grouped as pre-defined patterns, two-stage pipelines and exploration-based methods: 1.25–3× lower flight time, 43–56.2 % higher target coverage and 11.6–50.1 % higher success rate. [V, from the first 100k chars of the HTML]
- **FC-Planner: A Skeleton-guided Planning Framework for Fast Aerial Coverage of Complex 3D Scenes.** (Feng et al. [M]) ICRA 2024, arXiv 2309.13882. [V] Skeleton-based space decomposition yields a minimal set of viewpoints, planned hierarchically and in parallel. It is over 10× faster at computing than state-of-the-art methods, with a shorter path and more complete coverage, tested in benchmarks and the real world. It takes a **prior model of the scene** (coverage of a known structure, not NBV). — [arXiv](https://arxiv.org/abs/2309.13882v3)
- **Star-Searcher**: in this session it showed up only as the base of SOAR's "SSearchers" baseline (multi-robot Star-Searcher). I did not verify a separate paper. [M] It is an RA-L 2024 aerial **target-search** system, not a reconstruction planner. — [SOAR HTML](https://arxiv.org/html/2409.02738)
- **Learning Reconstructability for Drone Aerial Path Planning.** Y. Liu, L. Lin, Y. Hu, K. Xie, C.-W. Fu, H. Zhang, H. Huang. SIGGRAPH Asia 2022 (ACM TOG), arXiv 2209.10174. [V] The first learned reconstructability predictor, trained on simulated proxy-based reconstruction. Workflow: build a 3D proxy, then plan from predicted quality and uncertainty. Iterative view planning is shown on synthetic and real urban scenes. The abstract gives no image-count figures. — [arXiv abs](https://arxiv.org/abs/2209.10174)
  - Follow-up: "Aerial Path Online Planning for Urban Scene Updation," SIGGRAPH 2025. [S] — [search result](https://arxiv.org/pdf/2505.01486v3.pdf) (arXiv id 2505.01486 taken from the URL only; not opened)
- **On-the-fly Feedback SfM.** L. Lou, W. Li, W. Gan, Y. Yu, T. Wang, X. Wang, Z. Zhan. arXiv 2512.02375, submitted to IEEE GRSM. [V] Online explore-and-exploit UAV photogrammetry: incremental mesh from sparse SfM points, a mesh-quality indicator and predictive path planning. Claims it "markedly reduces coverage gaps and re-flight costs." The abstract has no image-count numbers. — [arXiv abs](https://arxiv.org/abs/2512.02375)
- **Iterative Hybrid Discrete-Continuous Viewpoint Planning for UAV Photogrammetry.** A. Grech, D. Pisani, A. Grima, C.J. Debono, S. Formosa, D. Seychell. EUVIP 2026, arXiv 2608.05718. [V] Candidate views are scored on photogrammetric metrics (frontality, distance, parallax, multi-view observations) and the set is optimized with CMA-ES clustering. On three synthetic scenes it beats prior UAV path-planning methods on accuracy and completeness. — [arXiv abs](https://arxiv.org/abs/2608.05718)
  - A search snippet says it uses "substantially fewer images than comparison methods". [S] — [pdf](https://arxiv.org/pdf/2608.05718)
- **Quality-Adaptive Multi-UAV 3D Reconstruction with Sparse Workload Redistribution.** arXiv 2607.24233. [S, not opened] — [arXiv HTML](https://arxiv.org/html/2607.24233)
- **Optimized UAV view planning ... using a modified sparrow search algorithm** (Advanced Engineering Informatics 2025). An explore-then-exploit method for buildings that "aim[s] to minimize the number of images required." [S] — [ScienceDirect](https://www.sciencedirect.com/science/article/abs/pii/S147403462500237X)

**D. 3DGS / radiance-field active reconstruction with a quadrotor (mostly indoor scenes, RGB-D, simulation)**

- **GS-Planner.** R. Jin, Y. Gao, H. Lu, F. Gao. arXiv 2405.10142 (May 2024). [V] 3DGS extended to flag unobserved regions, with online quality and completeness evaluation driving sampling-based active reconstruction and 3DGS safety constraints for the quadrotor. Uses **RGB-D** (640×480, 0.5–3 m range). **Simulation only (Unity)**; real deployment is listed as future work. The scene is a 22×14×3.2 m supermarket, covered in 343 s. No peer-reviewed venue is given on arXiv. — [arXiv HTML](https://arxiv.org/html/2405.10142v1)
- **GauSS-MI.** Y. Xie, Y. Cai, Y. Zhang, L. Yang, J. Pan. arXiv 2504.21067 (2025). [V] Shannon mutual information on 3DGS for real-time active view selection, tested on "various simulated and real-world scenes." Sensors and platform are not given in the abstract. — [arXiv abs](https://arxiv.org/abs/2504.21067)
- **Return-to-Home Feasible MAV Exploration for 3DGS Reconstruction.** P. Krisshnakumar, F. Yang, K. Niinuma. IROS 2026, arXiv 2610.04013. [V] An RGB-D MAV builds an indoor online 3DGS map with a flight-time-budgeted NBV. Simulation only (ReplicaCAD, Gibson, HM3D). — [arXiv abs](https://arxiv.org/abs/2610.04013)
- **OccamView.** H. Gao, W. Zhang, Z. Ni, D. Zhu, R. Li, Y. Wang, C. Xu. arXiv 2608.16499 (Aug 2026). [V] Object-conditioned, **frame-budgeted** view selection for active 3DGS. Uses RGB-D and open-vocabulary object memory; tested on Replica and Matterport3D, with the largest gains at small frame budgets. Not a drone and not real. Relevant because it frames the problem as a fixed frame budget. — [arXiv abs](https://arxiv.org/abs/2608.16499)
- Other active-3DGS/NBV work surfaced but not opened [S]: HGS-Planner (ICRA 2025), ActiveSplat (RA-L 2025), MAGICIAN (arXiv 2603.22650), DynActiveGS (2608.01178), SA-ResGS (2601.03024), uncertainty-driven 3DGS active mapping via an anisotropic visibility field (2605.30342), and ObjView-Bench (2605.10707, object-centric view-planning benchmark). — [search results](https://arxiv.org/pdf/2603.22650), [ObjView-Bench](https://arxiv.org/pdf/2605.10707)

**E. Already-known baselines (brief, [M] unless noted)**

- Mostegel et al., CVPRW 2016: UAV image acquisition with MVS-confidence prediction (buildings, outdoor). [M]
- Bircher et al., "Receding Horizon NBV Planner for 3D Exploration," ICRA 2016: volumetric exploration with depth/stereo, RRT-based. [M]
- Smith et al., "Aerial Path Planning for Urban Scene Reconstruction: A Continuous Optimization Method and Benchmark," SIGGRAPH Asia 2018: offline reconstructability-optimized views from a coarse proxy. [M]
- Peng & Isler, "Adaptive View Planning for Aerial 3D Reconstruction," ICRA 2019, arXiv 1805.00506: explore-then-exploit with a coarse model from the first flight. [M]
- Zhou et al., "Offsite Aerial Path Planning for Efficient Urban Scene Reconstruction," SIGGRAPH Asia 2020: plans from 2D maps and satellite data with no pre-flight. [M]

### Inferences
- The table-level picture is that LiDAR or RGB-D is the norm for *planning* in aerial active reconstruction (SOAR, FlyCo, GS-Planner, RTH-MAV, OccamView). RGB-only *planning* appears in Hestia (learned depth), ActMVS (MVS depth, sim), MACARONS (self-supervised, sim) and the toy drone (SfM/MVS online).
- Object-scale real-drone work is rare. Hestia (indoor objects, Crazyflie) and the toy-drone indoor Vicon tests are the only two found.

### Gaps
- I could not open the full texts of GauSS-MI, HGS-Planner, ActiveSplat, MAGICIAN or ObjView-Bench to check sensors and real-robot details.
- Hestia's exact real-world object sizes and quantitative real results were not extracted (only the first 100k chars of the HTML were read).
- PredRecon's sensor suite and numbers were not in the abstract.

## Q2. Which do object-centric / small-structure active reconstruction with a real drone, especially indoors?

### Takeaway
Only two found: **Hestia** (WACV 2026; indoor objects, real Crazyflie, RGB-only, MASt3R-SfM) and **Huang et al. toy drone** (2017/2018; real Bebop, indoor Vicon room with cardboard structures plus outdoor buildings). Everything else that is object-centric (GenNBV on OmniObject3D, ObjView-Bench, OccamView) is simulation and/or not a drone. Everything flown for real outdoors (FlyCo, FC-Planner, Liu et al. 2022) is building-scale and LiDAR- or proxy-based.

### Cited Findings
- Hestia: Crazyflie + RGB camera, no depth, indoor objects, a depth predictor feeds the occupancy grid, MASt3R-SfM, the first 3 views set by hand. — [arXiv HTML](https://arxiv.org/html/2508.01014)
- Toy drone: Bebop, indoor Vicon tests with cardboard structures, outdoor 15 m gate, SfM/MVS pipeline. — [ar5iv](https://ar5iv.arxiv.org/html/1705.01010)
- FlyCo real flights are outdoor architectural targets with LiDAR + gimbal camera. — [arXiv HTML](https://arxiv.org/html/2601.07558)
- GS-Planner and SOAR are simulation only. — [GS-Planner](https://arxiv.org/html/2405.10142v1); [SOAR](https://arxiv.org/html/2409.02738)

### Inferences
- A real PX4 drone running onboard compute (Jetson) indoors, object-centric, with a standard COLMAP/3DGS output, does not appear in the 2022–2026 literature found. Hestia uses a nano-drone with external Lighthouse tracking and MASt3R-SfM, not COLMAP, and gives qualitative real results.

### Gaps
- An IEEE Xplore / Scholar "cited by" sweep was not possible with the tools available. Indoor object-scale drone works in venues not indexed on arXiv may have been missed.

## Q3. Which use only a monocular RGB camera for planning, and how do they get scale?

### Takeaway
RGB-only planners get metric scale from **known camera poses**: simulator poses, Vicon/GPS, Lighthouse, or robot odometry. Depth then comes from MVS or a learned depth/geometry model. None I found uses an uncalibrated scale.

### Cited Findings
- ActMVS: RGB + pose (simulator, or odometry on real robots). Metric depth comes from MVS with known camera motion plus depth-warping alignment. — [arXiv HTML](https://arxiv.org/html/2606.01367)
- Hestia: RGB only, with a depth predictor from multi-view RGB. Poses come from the virtual-real sync (first 3 views by hand) and Lighthouse localization [S]. — [arXiv HTML](https://arxiv.org/html/2508.01014)
- Toy drone: scale from GPS outdoors and Vicon indoors. — [ar5iv](https://ar5iv.arxiv.org/html/1705.01010)
- MACARONS (in the ISPRS 2025 evaluation): "only a monocular RGB sensor", synthetic. — [ISPRS Annals](https://isprs-annals.copernicus.org/articles/X-2-W2-2025/207/2025/)

### Inferences
- Kinetix's "RGB + drone pose (PX4 EKF / mocap)" sits in the same family as ActMVS and the toy drone. A measured stereo-versus-monocular comparison on the same platform was not found in any of these works. ActMVS compares against RGB-D methods, not against stereo on the same drone.

### Gaps
- How GenNBV's and PredRecon's observations are formed (depth vs RGB) is not confirmed from primary text.

## Q4. Which report image-count reduction vs fixed orbit/grid/coverage paths, and by how much?

### Takeaway
Almost none report image count as the headline metric. The 2022–2026 aerial planners report flight time, path length, coverage or F-score (SOAR, FlyCo, FC-Planner, GS-Planner). Image-count framing appears in the photogrammetry-oriented line (toy drone, EUVIP 2026 hybrid planner, sparrow-search building planner) and in frame-budgeted 3DGS (OccamView), but I found **no verified figure of the "50–80 % fewer images vs orbit/grid at equal quality" type**.

### Cited Findings
- Toy drone: 101–108 total images after 5 iterations, compared with hand-designed grid and rectangle plans. — [ar5iv](https://ar5iv.arxiv.org/html/1705.01010)
- FlyCo: 1.25–3× lower flight time than pre-defined patterns, two-stage and exploration methods (time, not images). — [arXiv HTML](https://arxiv.org/html/2601.07558)
- MACARONS on bridges: "37.5 % efficiency improvement" vs a predefined flight path (the metric definition was not visible). — [ISPRS Annals](https://isprs-annals.copernicus.org/articles/X-2-W2-2025/207/2025/)
- EUVIP 2026 hybrid viewpoint planner: "substantially fewer images" [S]; the abstract confirms gains in accuracy and completeness. — [arXiv abs](https://arxiv.org/abs/2608.05718)
- OccamView: gains largest at small frame budgets (RGB-D, not a drone). — [arXiv abs](https://arxiv.org/abs/2608.16499)
- Sparrow-search building planner "aim[s] to minimize the number of images." [S] — [ScienceDirect](https://www.sciencedirect.com/science/article/abs/pii/S147403462500237X)
- Practitioner context (not peer-reviewed): circlegrammetry is claimed to need fewer images than a double grid. [S] — [UgCS manual](https://manuals-ugcs.sphengineering.com/docs/circlegrammetry-area)

### Inferences
- An image-count-at-equal-quality comparison against orbit/grid baselines, at object scale, with COLMAP/3DGS metrics, seems uncommon. It is a defensible framing for Kinetix, but the claim must be benchmarked carefully against a *good* orbit, such as multi-ring orbits at matched GSD.

### Gaps
- Exact image-reduction numbers in the EUVIP 2026 paper, the sparrow-search paper and the On-the-fly Feedback SfM paper were not extracted.

## Q5. Explore-then-exploit / two-stage aerial reconstruction 2022–2026 and 3DGS-based aerial active reconstruction

### Takeaway
The explore-then-exploit line (Peng & Isler 2019, Smith 2018, Zhou 2020) evolved in three ways after 2022. Learned reconstructability (Liu et al. 2022, with a SIGGRAPH 2025 online-update follow-up) is one. Prediction-boosted single-flight planners (PredRecon 2023 → SOAR 2024 → FlyCo 2026, mostly from the HKUST/SYSU groups, LiDAR-based, mostly outdoor) are another. Online SfM feedback (On-the-fly Feedback SfM, 2025) is the third. Active 3DGS planners (GS-Planner, GauSS-MI, HGS-Planner, ActiveSplat, RTH-MAV) are mostly RGB-D, indoor-room scale and simulated.

### Cited Findings
- SOAR uses Multi-EE (multi-UAV explore-then-exploit with a RealityCapture coarse model) as its baseline and beats it on time and quality. — [SOAR HTML](https://arxiv.org/html/2409.02738)
- FlyCo groups its baselines as pre-defined patterns, two-stage pipelines and exploration methods. — [FlyCo HTML](https://arxiv.org/html/2601.07558)
- On-the-fly Feedback SfM frames itself as online explore-and-exploit with SfM-driven incremental meshing. — [arXiv abs](https://arxiv.org/abs/2512.02375)
- The 2022 review of UAV viewpoints and path planning defines explore-then-exploit as a first flight that builds a proxy and a second, optimized flight. — [arXiv 2205.03716](https://arxiv.org/pdf/2205.03716)

### Inferences
- Kinetix's explore (orbit seed) → exploit (NBV) loop at object scale is conceptually a down-scaled Peng & Isler / toy-drone design. Its novelty must therefore rest on the specifics: indoor object scale, monocular-plus-pose online quality estimation, PX4/Jetson real flights, standard COLMAP/3DGS output and an image-count-at-equal-quality evaluation with a measured stereo comparison.

### Gaps
- No primary-source check of HGS-Planner, ActiveSplat, Star-Searcher or the SIGGRAPH 2025 online-update paper.

## Closest 5 works to Kinetix and how Kinetix differs

1. **Hestia (WACV 2026, arXiv 2508.01014).** Real drone, indoor objects, RGB-only, learned NBV, MASt3R-SfM. *Kinetix differs:* a PX4/Jetson drone (not a Crazyflie with Lighthouse), a COLMAP → MVS/3DGS output, image count at equal quality vs orbit/grid as the main metric (Hestia reports coverage and Chamfer distance), and a monocular-vs-stereo measured comparison.
2. **Active Image-based Modeling with a Toy Drone (Huang et al., arXiv 1705.01010).** Monocular Bebop, SfM/MVS, iterative refinement, real indoor and outdoor flights, hand-designed grid baselines. *Kinetix differs:* object scale (0.3–2 m) rather than building scale, a modern 3DGS/COLMAP output, a sim-to-real twin (Isaac + PX4 SITL) and explicit image-reduction targets. It is pre-2022 and so must be cited as the closest classical precedent.
3. **ActMVS (arXiv 2606.01367, 2026).** Monocular RGB + pose active reconstruction with MVS depth and 3DGS output. *Kinetix differs:* object-centric photogrammetry rather than room exploration, real-drone validation (ActMVS is simulation only) and image-count efficiency rather than RGB-D parity.
4. **PredRecon / SOAR / FlyCo line (ICRA 2023 / IROS 2024 / arXiv 2601.07558).** Prediction-boosted, MVS-aware aerial reconstruction planning. FlyCo has real outdoor flights. *Kinetix differs:* no LiDAR, indoor single-object scale, and efficiency in image count rather than flight time or coverage.
5. **GS-Planner (arXiv 2405.10142) and GauSS-MI (arXiv 2504.21067).** 3DGS-driven active view selection with a quadrotor or real scenes. *Kinetix differs:* planning uses RGB + pose, not RGB-D (GS-Planner uses RGB-D in Unity sim). Output stays as standard COLMAP-compatible image sets rather than an online 3DGS map. The target is object scale with a real PX4 drone indoors.

Honorable mention: **OccamView (arXiv 2608.16499)** is the closest *framing* match, with frame-budgeted object completion, but it uses RGB-D, Replica and Matterport3D, and no drone.

**Novelty risk summary:** Each individual ingredient has precedent: monocular active reconstruction (ActMVS, MACARONS), real-drone indoor RGB-only object NBV (Hestia), explore-then-exploit SfM/MVS by drone (toy drone, On-the-fly Feedback SfM), and frame budgets (OccamView). The defensible novelty is in the combination: indoor object scale, a real PX4/Jetson drone, monocular RGB + pose planning with a measured stereo comparison, output that drops into standard COLMAP/MVS/3DGS, and a quantified image-count reduction at equal quality vs orbit/grid baselines. I found no verified prior that reports that last quantity.
