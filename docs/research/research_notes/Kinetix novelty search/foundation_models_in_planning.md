# Foundation depth / pointmap models inside active reconstruction, NBV and exploration (2023-2026), plus monocular-only active reconstruction — novelty check for Kinetix

Scope note: Kinetix = adaptive NBV photogrammetry of one indoor object by a small drone. The primary pipeline is monocular RGB plus drone pose: Depth Anything V2 depth is scale-fit to SfM sparse points, fused into a coarse TSDF and used for a frontier/completeness score, alongside an SfM-uncertainty score. ZED Mini stereo is the comparison, and the output comes from COLMAP -> MVS/3DGS.
Search date: 2026-10-08. About 20 search/fetch calls. Where I checked an abstract or page in this session, it is cited. Anything marked **[unverified]** comes from prior knowledge and was not re-checked here.

## Is "monocular depth prior (e.g. Depth Anything) fused for NBV of an object" already published? By whom, how close?

### Takeaway
I found no paper that does exactly what Kinetix does. That is: Depth Anything (or another relative-depth foundation model), scale-fitted to SfM sparse points, fused into a TSDF to drive object-centric NBV on a drone, with SfM uncertainty as a second score. Four nearby works each cover one or two of these parts:
- **Next Best Sense** (Strong et al. 2024): Depth Anything V2 / Metric3D V2 in an NBV loop for objects, on a robot arm. It aligns scale against a real depth sensor, not against SfM.
- **FrontierNet** (RA-L 2025): Metric3D v2 monocular depth feeds frontier detection and information gain. It explores scenes with a Spot robot.
- **GraspView** (2025): VGGT with NBV and metric alignment from robot kinematics, for tabletop grasping.
- **ActMVS** (ICRA 2026): monocular MVS for active scene reconstruction on UAVs. Simulation only (Replica).

### Cited Findings
**Closest: monocular foundation depth in an object/scene NBV loop**
- **Next Best Sense: Guiding Vision and Touch with FisherRF for 3D Gaussian Splatting.** Matthew Strong, Boshu Lei, Aiden Swann, Wen Jiang, Kostas Daniilidis, Monroe Kennedy III. arXiv 2410.04680 (Oct 2024).
  - It supports **DepthAnythingV2 and Metric3DV2** as depth sources and does "semantic depth alignment" with SAM2: a scale and offset per object and for the background. — [arXiv html](https://arxiv.org/html/2410.04680v1)
  - The alignment is fitted against **metric depth from a real depth camera**, so this is not a monocular-only system. — [arXiv html](https://arxiv.org/html/2410.04680v1)
  - It plans next-best-view and next-best-touch with FisherRF (3DGS Fisher information), on a Kinova Gen3 arm. Real objects: Stanford Bunny, toy car, mirror, prism, ridged objects; 8 initial views, then online active selection. — [arXiv html](https://arxiv.org/html/2410.04680v1)
  - Metrics: PSNR, SSIM, LPIPS and depth MAE (D-ABS). — [arXiv html](https://arxiv.org/html/2410.04680v1)
  - Closeness to Kinetix: high on "foundation depth prior + object NBV". It differs on scale source (depth sensor, not SfM), representation (3DGS + FisherRF, not TSDF frontier), and platform (arm, not drone).
- **FrontierNet: Learning Visual Cues to Explore.** ETH Zurich / TUM / Bonn / Microsoft. IEEE RA-L 2025, arXiv 2501.04597.
  - It detects frontiers and predicts their information gain "from posed RGB images enhanced by monocular depth priors". — [arXiv](https://arxiv.org/pdf/2501.04597)
  - The depth prior is **Metric3D v2**, a metric model, so the paper has no explicit scale recovery. — [arXiv html v2](https://arxiv.org/html/2501.04597v2)
  - Real robot: Boston Dynamics Spot, 640x480 RGB at 3 Hz, ~5 Hz inference on a laptop with a 3080Ti. — [arXiv html v2](https://arxiv.org/html/2501.04597v2)
  - Metrics: Vox@25/50/100% (volumetric coverage) and success rate. About 15-16% gain in early exploration. — [arXiv html v2](https://arxiv.org/html/2501.04597v2)
  - **Monocular vs perfect-depth comparison:** the baselines (classic frontier, NBVP, SEER) used perfect simulator depth and "degrade significantly with predicted depth", while FrontierNet stays robust with monocular depth. — [arXiv html v2](https://arxiv.org/html/2501.04597v2)
  - Ablation: frontier detection relies mostly on depth cues; information-gain estimation uses both RGB and depth. — [arXiv](https://arxiv.org/html/2501.04597v1)
  - Task: scene exploration, not object reconstruction.
- **GraspView: Active Perception Scoring and Best-View Optimization for Robotic Grasping in Cluttered Environments.** Wang, Dai, Su, Liu, Chen, Wu, Lin. arXiv 2511.04199 (Nov 2025).
  - RGB only, with **VGGT** reconstruction. An "online metric alignment module calibrates VGGT predictions against robot kinematics".
  - A "render-and-score" active perception strategy picks next-best-views that reveal occluded regions.
  - Real tabletop objects. Reported to beat both **RGB-D and single-view RGB baselines**, including on transparent objects. — [arXiv](https://arxiv.org/abs/2511.04199)
  - Closeness to Kinetix: same idea of a feed-forward model plus known pose for scale, with NBV. The goal is grasping in clutter, not complete photogrammetric object reconstruction, and the platform is an arm.
- **ActMVS: Active Scene Reconstruction with Monocular Multi-View Stereo.** Guo Pu, Yixuan Han, Zhouhui Lian. ICRA 2026, arXiv 2606.01367 (31 May 2026).
  - Claims to be "the first monocular scene reconstruction framework that enables robust autonomous aerial reconstruction without depth sensors".
  - Method: a view factor graph, voxel-frame visibility modeling, informed MVS depth prediction and global depth optimization. The result is an occupancy map for safe planning.
  - Evaluated on Replica (synthetic). Reported "competitive with RGB-D methods". Code at TrickyGo/ActMVS. — [arXiv](https://arxiv.org/pdf/2606.01367), [abs](https://arxiv.org/abs/2606.01367)
  - The abstract page does not say how scale is resolved. Learned MVS with known poses implies metric scale from the poses (inference). Scene-level, not object-level.

**Feed-forward 3D foundation models as the perception backbone of an active reconstruction agent**
- **AREA3D: Active Reconstruction Agent with Unified Feed-Forward 3D Perception and Vision-Language Guidance.** Tianling Xu, Shengzhe Gan, Leslie Gu, Yuelei Li, Fangneng Zhan, Hanspeter Pfister. CVPR 2026, arXiv 2512.05131.
  - An active viewpoint-selection agent built on feed-forward 3D perception. Uncertainty modeling is decoupled from the reconstructor, with VLM semantic guidance.
  - Evaluated on scene- and object-level benchmarks and strong in sparse-view settings. — [arXiv](https://arxiv.org/abs/2512.05131)
  - The abstract does not name the backbone (VGGT, DA3, etc.), the scale handling or the platform **[backbone unverified]**.
- **Auto3R: Automated 3D Reconstruction and Scanning via Data-driven Uncertainty Quantification.** Shen, Zheng, Wu, Feng, Fei, Mei, Hanwen Jiang, Xiangru Huang. arXiv 2512.04528 (Dec 2025).
  - Learned uncertainty over candidate viewpoints, used for iterative scanning of objects and scenes, including non-Lambertian and specular ones.
  - Real deployment on a **robot arm with a camera**. — [arXiv](https://arxiv.org/abs/2512.04528)
  - The abstract names no specific depth foundation model.

**Single-image / generative-prior NBV for objects (RGB only)**
- **Peering into the Unknown: Active View Selection with Neural Uncertainty Maps (UPNet).** Zhengquan Zhang, Feng Xu, Mengmi Zhang. ICLR 2026, arXiv 2506.14856.
  - A feed-forward net maps one RGB image of an object to uncertainty over candidate views.
  - About half the views for comparable quality, up to 400x speedup, generalizes to novel categories. — [arXiv](https://arxiv.org/abs/2506.14856)
- **DM-OSVP++: One-Shot View Planning Using 3D Diffusion Models for Active RGB-Based Object Reconstruction.** Sicong Pan, Liren Jin, Xuying Huang, Cyrill Stachniss, Marija Popović, Maren Bennewitz. arXiv 2504.11674 (Apr 2025).
  - A 3D diffusion model, conditioned on the initial RGB views, produces a coarse object prior. The prior drives one-shot view planning.
  - Simulation and real-world experiments. — [arXiv](https://arxiv.org/abs/2504.11674)
  - The predecessor, "Exploiting Priors from 3D Diffusion Models for RGB-Based One-Shot View Planning" (arXiv 2403.16803), builds a mesh from one RGB image with a 3D diffusion model and then plans the shortest global path through the views. — [arXiv](https://arxiv.org/html/2403.16803)
  - Platform: robot arm **[unverified for ++; the 2024 paper used a UR arm from prior knowledge]**.
- **ACT-R: Adaptive Camera Trajectories for Single View 3D Reconstruction.** Yizhi Wang, Mingrui Zhao, Hao Zhang. 3DV 2026, arXiv 2505.08239.
  - Computes an adaptive orbit that maximises visibility of occluded regions, generates views along it with a **video diffusion model**, then runs standard multi-view reconstruction.
  - Inference only. This is **virtual** view planning for generation; no robot. — [arXiv](https://arxiv.org/abs/2505.08239)
- **Hestia** (WACV 2026, arXiv 2508.01014): RL hierarchical, face-aware NBV for object reconstruction. At least 4% coverage gain and 50% less Chamfer distance; "feasible for real-world application". It uses no foundation depth model. — [arXiv](https://arxiv.org/abs/2508.01014)
- **GenNBV** (CVPR 2024, arXiv 2402.16174): a generalizable RL NBV policy for active 3D reconstruction in a free drone-like action space. — [arXiv](https://arxiv.org/pdf/2402.16174). Its inputs are depth-based, not a monocular prior **[unverified detail]**.

### Inferences
- The "foundation depth -> volumetric map -> NBV" pattern is published: FrontierNet for scenes, Next Best Sense for objects with 3DGS. Kinetix cannot claim "first to use a monocular depth foundation model for NBV".
- What stays distinct is the combination: (i) **SfM sparse points** as the scale anchor for a relative-depth model during flight, (ii) a coarse TSDF completeness/frontier score (iii) fused with SfM-uncertainty, (iv) object-centric photogrammetry on a **drone** indoors, (v) a final COLMAP/MVS/3DGS deliverable. None of the papers found combine (i), (iv) and (v).
- Next Best Sense is the paper Kinetix most needs to cite and contrast with: same prior (DA-V2), object NBV, but sensor-aligned scale on an arm.

### Gaps
- I did not verify AREA3D's backbone (VGGT, DA3 or other) from the full PDF.
- I did not check the GenNBV, DM-OSVP++ and Hestia full texts for sensor modality or platform.
- Google Scholar "cited-by" for Depth Anything V2 / VGGT was not reachable via WebSearch. More close works may exist, especially 2026 workshop papers.

## Are DUSt3R/MASt3R/VGGT used for active view selection or online reconstruction on drones/robots?

### Takeaway
Yes, for **online reconstruction / SLAM**: MASt3R-SLAM, VGGT-SLAM, VGGT-Motion, AIM-SLAM, VGGT-GS SLAM, and On-the-Fly3R for UAVs. For **active view selection**, the uses found are GraspView (VGGT + NBV, arm) and AREA3D (a feed-forward 3D perception agent). I found no published drone NBV/exploration system that closes the loop with DUSt3R, MASt3R or VGGT.

### Cited Findings
- **MASt3R-SLAM**: "the first real-time dense monocular SLAM leveraging a reconstruction prior". It uses fixed two-view input. — [AIM-SLAM related work](https://arxiv.org/html/2603.05097v1). Murai et al., CVPR 2025; arXiv id 2412.12392 **[from memory, unverified]**. Passive, with no planning.
- **VGGT-SLAM / VGGT-SLAM 2.0** handle the 15-DoF projective (homography) ambiguity of uncalibrated submaps and process fixed chunks of keyframes. — [AIM-SLAM](https://arxiv.org/html/2603.05097v1). Passive.
- **AIM-SLAM** (arXiv 2603.05097) chooses a variable number of keyframes by viewpoint overlap and **information gain**. This is view selection among already-captured frames, not motion planning. — [arXiv html](https://arxiv.org/html/2603.05097v1)
- **VGGT-Motion** (arXiv 2602.05508): calibration-free monocular SLAM. — [arXiv](https://arxiv.org/html/2602.05508v1)
- **VGGT-GS SLAM** (arXiv 2609.19628): uncalibrated monocular 3DGS SLAM with feed-forward priors. — [arXiv](https://arxiv.org/pdf/2609.19628)
- **"Accelerating Transformer-Based Monocular SLAM via Geometric Utility Scoring"** (arXiv 2604.08718). — [arXiv](https://arxiv.org/pdf/2604.08718)
- **On-the-Fly3R** (arXiv 2609.00923): online 3D reconstruction with feed-forward 3R models for large UAV scenes. — [arXiv](https://arxiv.org/pdf/2609.00923). Online reconstruction, not active **[contents unverified beyond title]**.
- **GraspView** (2511.04199): VGGT + render-and-score NBV + metric alignment to robot kinematics, on a real arm. — [arXiv](https://arxiv.org/abs/2511.04199)
- **AREA3D** (CVPR 2026): an active reconstruction agent on feed-forward 3D perception. — [arXiv](https://arxiv.org/abs/2512.05131)
- **Uncertainty for pointmaps** is now a research line of its own, a prerequisite for using them in NBV:
  - **Trust3R** (arXiv 2605.19539): evidential NIW head giving Student-t per-point uncertainty. — [arXiv](https://arxiv.org/html/2605.19539)
  - **ReVar3R** (arXiv 2610.07883): evidential uncertainty head on frozen MASt3R. — [arXiv](https://arxiv.org/html/2610.07883)
  - Both note that MASt3R's confidence is "a learned reliability weight ... rather than a predictive uncertainty with a clear probabilistic interpretation". — [Trust3R](https://arxiv.org/html/2605.19539)
- **FlyCo** (Feng, ..., Shaojie Shen, Boyu Zhou; arXiv 2601.07558, Jan 2026): foundation-model-empowered drones for autonomous 3D **structure scanning** in open-world settings. Uses **vision-language** foundation models for target grounding and tracking, with real-world and simulation experiments. — [arXiv](https://arxiv.org/abs/2601.07558). The abstract does not name a geometric foundation model or the sensors **[sensors unverified; the HKUST/SYSU aerial-scanning line typically uses LiDAR]**.
- **Depth Anything 3** (arXiv 2511.10647): reported to beat DA2 on monocular depth and VGGT on multi-view depth and pose. It aligns dense pseudo-depth to sparse/noisy depth with RANSAC scale-shift. — [GitHub](https://github.com/ByteDance-Seed/Depth-Anything-3), [arXiv](https://arxiv.org/html/2511.10647v1)

### Inferences
- "VGGT/MASt3R/DA3 as the online mapper inside a drone NBV loop for an object" appears unpublished as of 2026-10-08. Feed-forward SLAM is passive, and the active uses are on arms or benchmarks.
- A Kinetix ablation that swaps DA-V2+SfM-scale for DA3 or VGGT multi-view depth (scale from drone pose) would be a timely comparison and not yet done on a drone.

### Gaps
- I could not confirm whether any 2026 workshop paper (e.g. RSS/ICRA aerial workshops) uses MASt3R-SLAM/VGGT-SLAM for active exploration on a drone. Search returned nothing.
- I did not open On-the-Fly3R's full text.

## Monocular active reconstruction / exploration systems (RGB without a depth sensor)

### Takeaway
Monocular-only active systems exist across several sub-families:
- **Self-supervised RGB NBV**: MACARONS (CVPR 2023).
- **Learned-MVS active scene reconstruction**: ActMVS (ICRA 2026, sim).
- **Monocular depth-network navigation**: MonoNav (ISER 2023, Crazyflie).
- **Monocular-depth-prior exploration**: FrontierNet (RA-L 2025, Spot).
- **Sparse-SLAM monocular UAV exploration**: MonoSpheres (RA-L 2026, real outdoor). It explicitly avoids learned depth.
- **High-altitude monocular semantic exploration**: HALO (2025).
- **Generative/single-image object view planners**: DM-OSVP(++), UPNet, ACT-R.

None does photogrammetric single-object NBV on an indoor drone.

### Cited Findings
- **MACARONS** (Guédon, Monnier, Monasse, Lepetit; CVPR 2023; arXiv 2303.03315). Exploration and 3D reconstruction of large scenes **from color images only**. A self-supervised "volume occupancy field" predicts NBV coverage gain, and it beats depth-dependent methods on 3D scene datasets. — [arXiv](https://arxiv.org/abs/2303.03315). Simulated scenes; the depth module is learned self-supervised, not a foundation model.
- **ActMVS** (ICRA 2026), see above: monocular MVS, aerial, Replica (sim), reported competitive with RGB-D. — [arXiv](https://arxiv.org/abs/2606.01367)
- **MonoNav** (Nathaniel Simon, Anirudha Majumdar; ISER 2023; arXiv 2311.14100).
  - Pre-trained monocular depth plus poses goes into a "metrically accurate 3D reconstruction", and motion primitives are planned on it. The depth network is ZoeDepth **[unverified detail]**.
  - Metric scale comes from odometry poses.
  - Crazyflie (37 g), real indoor flights at 0.5 m/s. Collision rate 4x lower than an end-to-end baseline, goal completion 22% lower. — [arXiv](https://arxiv.org/abs/2311.14100)
  - This is navigation, not NBV, but it is the closest small-drone "monocular foundation depth -> TSDF" precedent. MonoNav fuses into Open3D TSDF **[unverified]**.
- **MonoSpheres** (Tomáš Musil, Matěj Petrlík, Martin Saska; RA-L, arXiv 2511.17299).
  - Depth comes from **sparse OpenVINS SLAM points**. It deliberately avoids learned monocular depth, saying "current depth estimation models still struggle to provide real-time onboard performance and reliable generalization".
  - Metric scale comes from VIO (camera + IMU).
  - Real outdoor UAV runs: 8 min at a ~70x10x8 m farmhouse, 4 min in an orchard.
  - Metrics: explored area and mapped volume, compared with a ray-tracing occupancy baseline. No stereo or RGB-D comparison. — [arXiv html](https://arxiv.org/html/2511.17299)
- **HALO** (Tao, Ong, Cladera, Hughes, Taylor, Chaudhari, Vijay Kumar; arXiv 2511.17497). Monocular camera + GPS + IMU on a quadrotor. Real-time dense reconstruction at long range, language-conditioned exploration. Real runs up to 24,600 m² at 40 m altitude, with 68% better competitive ratio in distance travelled. — [arXiv](https://arxiv.org/abs/2511.17497)
- **FrontierNet**, see above. — [arXiv](https://arxiv.org/html/2501.04597v2)
- **NARUTO** (arXiv 2402.18771): neural active reconstruction from uncertain target observations. Uses RGB-D **[unverified that it is RGB-D; from prior knowledge]**. — [arXiv](https://arxiv.org/pdf/2402.18771)
- Offline, not active: **Murre**, "Multi-view Reconstruction via SfM-guided Monocular Depth Estimation" (Guo et al., CVPR 2025). It conditions monocular depth on SfM sparse points and fuses the result with TSDF, comparing against Depth Anything v1/v2. — [CVF](https://openaccess.thecvf.com/content/CVPR2025/papers/Guo_Multi-view_Reconstruction_via_SfM-guided_Monocular_Depth_Estimation_CVPR_2025_paper.pdf). It is the closest prior art for the "SfM points -> monocular depth -> TSDF" part of Kinetix, but it is offline and does not plan views.

### Inferences
- "Monocular-only active reconstruction" alone is not novel. ActMVS, MACARONS and MonoSpheres claim it for scenes. ActMVS explicitly claims "first monocular ... autonomous aerial reconstruction", so Kinetix should not repeat that claim for scenes.
- Kinetix's monocular claim must be scoped to object-centric photogrammetric NBV on a real indoor drone, with SfM-anchored foundation depth.
- MonoSpheres argues learned depth is unreliable onboard. Kinetix can answer this directly: SfM-scale-fitting tests reliability, and the stereo comparison measures it.

### Gaps
- I did not verify ActMVS's scale mechanism, real-world flights (abstract mentions Replica only) or exact metrics.
- I could not check MonoNav's exact depth model in this session.

## Do any of them compare monocular vs stereo vs RGB-D inputs within the same planner?

### Takeaway
Only partially. FrontierNet compares predicted monocular depth with perfect simulated depth. GraspView and ActMVS compare against RGB-D *methods*, not the same planner fed different sensors. I found **no paper that runs the same NBV planner on monocular prior depth vs stereo vs RGB-D on the same platform**, and none with a real stereo camera such as the ZED.

### Cited Findings
- FrontierNet: baselines run on perfect simulator depth degrade with predicted depth; FrontierNet stays robust with monocular (Metric3D v2) depth. — [arXiv html v2](https://arxiv.org/html/2501.04597v2)
- GraspView beats RGB-D and single-view RGB baselines. These are different methods, not one planner with swapped inputs. — [arXiv](https://arxiv.org/abs/2511.04199)
- ActMVS is "competitive with RGB-D methods" (Replica). — [arXiv](https://arxiv.org/abs/2606.01367)
- MonoSpheres has no stereo or RGB-D comparison. — [arXiv html](https://arxiv.org/html/2511.17299)
- Next Best Sense uses the depth sensor to align the monocular prior. It does not compare them as alternative planner inputs. — [arXiv html](https://arxiv.org/html/2410.04680v1)

### Inferences
- A controlled ablation is a clear, defensible contribution: same planner and scoring, with the depth source swapped between DA-V2+SfM-scale, ZED Mini stereo, sim ground-truth depth and optionally DA3/VGGT multi-view. Report the effect on NBV path length, coverage and the final COLMAP/MVS/3DGS quality.

### Gaps
- An exhaustive check of RA-L/IROS 2025-26 aerial view-planning papers for sensor ablations was not possible in the tool budget.

## Which foundation-model-in-the-loop claims are still open for Kinetix

Summary from the evidence above (inference, to be re-checked before submission):

1. **Open (no counter-example found):** relative-depth foundation model (DA-V2) **scale-fitted online to the growing SfM sparse model** and fused into a TSDF to drive **object-centric NBV**. Closest work: Next Best Sense scales with a depth sensor; Murre scales with SfM but offline; FrontierNet uses metric Metric3D without SfM.
2. **Open:** a **joint score of TSDF frontier/completeness from foundation depth and SfM-model uncertainty** for photogrammetry-oriented NBV. No paper found fuses the two.
3. **Open:** **indoor small-drone, single-object** NBV with a monocular foundation prior and **real-world validation**. Drone works found are scenes or structures (ActMVS sim, MonoSpheres outdoor, HALO outdoor, FlyCo structures with VLMs). Object-level works use arms (Next Best Sense, GraspView, Auto3R, DM-OSVP).
4. **Open:** **same-planner comparison of monocular-prior vs real stereo (ZED Mini) vs RGB-D/GT depth**, scored by downstream COLMAP->MVS/3DGS quality.
5. **Not open / must be scoped:**
   - "First monocular active (aerial) reconstruction" is already claimed by ActMVS for scenes.
   - "First to use monocular depth foundation models in exploration or NBV" is taken by FrontierNet and Next Best Sense.
   - "First RGB-only object NBV" is taken by MACARONS, DM-OSVP(++), UPNet and ACT-R (virtual).
   - "Feed-forward 3D model with kinematic scale for NBV" is taken by GraspView (VGGT, arm).
6. **Timely optional extension:** replacing DA-V2 with DA3, VGGT or MASt3R pointmaps plus a calibrated uncertainty (Trust3R/ReVar3R-style) inside the drone NBV loop. AREA3D (CVPR 2026) is the nearest. It is benchmark-level, and its backbone and platform are unverified.
