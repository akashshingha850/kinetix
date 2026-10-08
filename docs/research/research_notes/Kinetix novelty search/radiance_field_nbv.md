# Radiance-field / 3DGS-based active view selection and NBV (2022–2026): novelty check for Kinetix

Scope note: about 20 search/fetch calls. Items marked **[verified]** had title, arXiv id and core claims checked against arXiv, CVF, ECVA or RSS pages during this session. Items marked **[partly verified]** had only the id/title or abstract checked. Items marked **[unverified]** rest on prior knowledge and must be checked before anyone cites them. Per-paper numbers (budgets, savings) are given only where a source stated them.

## Q0. Catalogue: method, uncertainty signal, sensing, platform, metrics, budgets

### Takeaway
The field splits into four families. (a) Offline view selection inside a trained NeRF/3DGS (ActiveNeRF, FisherRF, POp-GS, NeRF Director), scored mainly with PSNR. (b) Online active mapping/exploration of scenes with RGB-D (NARUTO, Active Neural Mapping, ActiveGS, ActiveSplat, ActiveGAMER, AG-SLAM, RT-GuIDE, GS-Planner, GauSS-MI). (c) Learned, generalizable object-centric NBV policies (GenNBV, VIN-NBV, Hestia, PUN, SO-NeRF, Auto3R), which often report Chamfer/coverage. (d) Classic geometry-prediction NBV (MAP-NBV). None found combines all four: RGB-only, object-centric, a real drone, and COLMAP/MVS output.

### Cited Findings

**ActiveNeRF: Learning Where to See with Uncertainty Estimation** (Pan et al., Tsinghua/CMU). ECCV 2022, arXiv 2209.08546 **[verified]**
- Models each radiance as a Gaussian, so NeRF predicts its own variance. It adds the views that most reduce that uncertainty, in an active-learning loop with a "constrained input budget". Works offline on a candidate image pool, with no robot. RGB-only. Requires training the NeRF in the loop. Metric is rendering quality (PSNR/SSIM/LPIPS) — [ECVA](https://www.ecva.net/papers/eccv_2022/papers_ECCV/html/7175_ECCV_2022_paper.php); [arXiv](https://arxiv.org/abs/2209.08546)

**NeurAR: Neural Uncertainty for Autonomous 3D Reconstruction with Implicit Neural Representations** (Ran et al., Zhejiang U.). RA-L 2023, arXiv 2207.10985 **[verified]**
- Learns a PSNR proxy (neural uncertainty) jointly with the implicit network and uses it to plan a view path. Reports improvements in both "rendered image quality and geometry quality of reconstructed 3D models" over TSDF baselines and over no view planning. Requires online training. Scene/object in simulation; depth use not verified — [arXiv](https://arxiv.org/abs/2207.10985)

**Efficient View Path Planning for Autonomous Implicit Reconstruction** (Zeng, Li, Ran, Li, Gao, …, Ye; Zhejiang U./NetEase). arXiv 2209.13159 (Sep 2022; reportedly ICRA 2023, venue unverified) **[verified id]**
- A neural network approximates an "information gain field", combined with a coarse volumetric map and a graph-based informative path planner. **Deployed on a real UAV**. This is likely the "Zeng et al." work the brief refers to — [arXiv](https://arxiv.org/pdf/2209.13159); [code](https://github.com/small-zeng/EVPP)
- Gap: whether it needs RGB-D, and its exact metrics, were not checked.

**Uncertainty Guided Policy for Active Robotic 3D Reconstruction using Neural Radiance Fields** (Lee, Chen, Wang, Liniger, Kumar, Yu; ETH). RA-L 2022, arXiv 2209.08409 **[partly verified: id/title]**
- Object-centric. Uses a ray-based volumetric uncertainty (entropy of the ray weight distribution) from NeRF to choose the next view of a robot-mounted camera. RGB-only and online-trained. Platform and metric details **[unverified]** — [arXiv](https://arxiv.org/pdf/2209.08409)

**Active Implicit Object Reconstruction using Uncertainty-guided Next-Best-View Optimization** (Yan, Liu, Quan, Chen, Fu). RA-L 2023 (accepted Jul 2023, not ICCV), arXiv 2303.16739 **[verified]**
- Uses an implicit occupancy field as a geometry proxy. The NBV is optimized *continuously* by gradient ascent on a differentiable uncertainty (entropy). Object-centric — [arXiv](https://arxiv.org/abs/2303.16739)
- Note: the brief attributes this to "Zeng et al."; the paper is by Yan et al. The Zeng et al. work is 2209.13159 above.

**NeU-NBV: Next Best View Planning Using Uncertainty Estimation in Image-Based Neural Rendering** (Jin, Chen, Rückin, Popović; Bonn). IROS 2023, arXiv 2303.01284 **[partly verified]**
- Uses an image-based (PixelNeRF-style), generalizable rendering network that predicts per-pixel rendering uncertainty at novel views. It needs **no online per-scene training** and no 3D map ("mapless"), and its uncertainty is "an informative proxy for rendering quality at novel views". RGB-only. Metric is mainly rendering quality — [ar5iv](https://ar5iv.labs.arxiv.org/html/2303.01284)
- Platform and real-robot validation **[unverified]**; I believe it was evaluated in simulation on object-centric datasets.

**Density-aware NeRF Ensembles** (Sünderhauf, Abou-Chakra, Miller; QUT). ICRA 2023, arXiv 2209.08718 **[unverified id]**
- Uncertainty from ensembles plus density-awareness. Proposed as a signal for NBV, but it is an uncertainty-quantification paper, not an NBV system. No title "Active Implicit Reconstruction" from the Sünderhauf group was found; the brief's item may mix up this paper with Yan et al. 2303.16739, or with Popović-group work (NeU-NBV, ActiveGS).

**SO-NeRF: Active View Planning for NeRF using Surrogate Objectives** (K. Lee, Gupta, Kim, Makwana, C. Chen, C. Feng; NYU). arXiv 2312.03266 (Dec 2023) **[verified]**
- The SOAR scores are surface coverage, geometric complexity, textural complexity and ray diversity. SOARNet predicts them, selecting views "in mere seconds instead of hours" with **no radiance-field training during planning** (about 80x speed-up). Model-agnostic across NeRF and explicit representations — [arXiv](https://arxiv.org/pdf/2312.03266)

**Bayes' Rays: Uncertainty Quantification for Neural Radiance Fields** (Goli, Reading, Sellán, Jacobson, Tagliasacchi). CVPR 2024, arXiv 2309.03185 **[unverified, from prior knowledge]**
- Post-hoc spatial uncertainty from a Laplace approximation over a perturbation field on any pretrained NeRF. Used for artifact cleanup and uncertainty maps. **Not an NBV planner**, but often used as a baseline uncertainty signal.

**Active Neural Mapping** (Yan, Yang, Zha). ICCV 2023, arXiv 2308.16246 **[verified]**
- Uncertainty is "neural variability": the variance of predictions under random weight perturbations of the continually learned field. Online active mapping in Gibson and MP3D simulation, scene-level, RGB-D. Requires online training — [CVF](https://openaccess.thecvf.com/content/ICCV2023/html/Yan_Active_Neural_Mapping_ICCV_2023_paper.html)

**NARUTO: Neural Active Reconstruction from Uncertain Target Observations** (Feng et al.). CVPR 2024, arXiv 2402.18771 **[verified]**
- Hash-grid neural map with a learned uncertainty module and an uncertainty-aggregation goal search. Uses Replica and MP3D (scene-level, RGB-D, simulation) and reports surface-reconstruction metrics — [CVF](https://openaccess.thecvf.com/content/CVPR2024/html/Feng_NARUTO_Neural_Active_Reconstruction_from_Uncertain_Target_Observations_CVPR_2024_paper.html)

**GenNBV: Generalizable Next-Best-View Policy for Active 3D Reconstruction** (Chen et al.). CVPR 2024, arXiv 2402.16174 **[verified]**
- Reinforcement-learning policy with a 5-D free-space action space (drone-like camera). Benchmarked in Isaac Gym on Houses3K and OmniObject3D: "98.26% and 97.12% coverage ratios" on unseen objects. The metric is coverage, not PSNR. Simulation only — [CVF](https://openaccess.thecvf.com/content/CVPR2024/html/Chen_GenNBV_Generalizable_Next-Best-View_Policy_for_Active_3D_Reconstruction_CVPR_2024_paper.html)

**NeRF Director: Revisiting View Selection in Neural Volume Rendering** (Xiao et al.). CVPR 2024, arXiv 2406.08839 **[verified]**
- Compares Farthest View Sampling (FVS) and Information-Gain Sampling. Reports that "FVS can reach 33 dB of PSNR using only 70 training views, whereas random sampling would require 150 views". This is an explicit **views-to-reach-a-quality-level** measurement (about 53% fewer views), in PSNR terms only — [arXiv](https://arxiv.org/abs/2406.08839v1)

**FisherRF: Active View Selection and Mapping with Radiance Fields using Fisher Information** (Jiang, Lei, Daniilidis; UPenn). ECCV 2024 (oral), arXiv 2311.17874 **[verified]**
- Uses the Fisher information of the radiance-field parameters (diagonal Hessian approximation) to maximize expected information gain. Runs at "70 fps" with a 3DGS backend. Tasks are view selection, active mapping and uncertainty quantification. RGB-only for view selection, which is offline over a candidate pool. Its active-mapping variant ran in simulated scenes. Main metric is novel-view PSNR/SSIM/LPIPS. Requires an online-trained NeRF/3DGS — [ECVA](https://www.ecva.net/papers/eccv_2024/papers_ECCV/html/2130_ECCV_2024_paper.php); [arXiv](https://arxiv.org/abs/2311.17874v1)

**POp-GS: Next Best View in 3D-Gaussian Splatting with P-Optimality** (Wilson et al.). CVPR 2025, arXiv 2503.07819 **[verified]**
- Extends FisherRF. Derives the 3DGS covariance and applies optimal-experimental-design criteria; D- and T-optimality "lead to significant improvements" over FisherRF's criterion. Offline view selection, rendering metrics — [CVF](https://openaccess.thecvf.com/content/CVPR2025/html/Wilson_POp-GS_Next_Best_View_in_3D-Gaussian_Splatting_with_P-Optimality_CVPR_2025_paper.html)

**PUP 3D-GS: Principled Uncertainty Pruning for 3D Gaussian Splatting** (Hanson et al., UMD). CVPR 2025, arXiv 2406.10219 **[verified]**
- Computes a Hessian/Fisher-style spatial-sensitivity score per Gaussian and prunes 88.44% of Gaussians for a 2.65x render speed-up. It is a **pruning** method, not an NBV method. No "PUP-based view selection" paper was found, though the per-Gaussian sensitivity score could be reused as an NBV signal (inference) — [CVF](https://openaccess.thecvf.com/content/CVPR2025/html/Hanson_PUP_3D-GS_Principled_Uncertainty_Pruning_for_3D_Gaussian_Splatting_CVPR_2025_paper.html)

**GS-Planner: A Gaussian-Splatting-based Planning Framework for Active High-Fidelity Reconstruction** (Zhejiang U.). arXiv 2405.10142 (May 2024; IROS 2024 per prior knowledge, unverified) **[verified id]**
- 3DGS extended to mark unobserved regions, with online evaluation of quality and completeness and a sampling-based planner. Its **quadrotor** platform uses 3DGS-based safety constraints. Experiments are in "highly realistic simulation scenes" only, with no real flight reported. Improves "geometric and textural quality" — [arXiv](https://arxiv.org/abs/2405.10142v1)

**RT-GuIDE: Real-Time Gaussian Splatting for Information-Driven Exploration** (KumarRobotics, UPenn). arXiv 2409.18122 **[verified]**
- GPU-accelerated information-driven planning on an onboard Gaussian map. In real-world experiments it beats the state of the art by "at least 0.8 dB higher PSNR and more than 16% higher geometric reconstruction accuracy". It is a scene-exploration system on a real **ground** robot (by prior knowledge, unverified) and uses RGB-D — [arXiv](https://arxiv.org/pdf/2409.18122); [code](https://github.com/KumarRobotics/RT-GuIDE)

**AG-SLAM: Active Gaussian Splatting SLAM** (Jiang et al., UPenn). arXiv 2410.17422 **[verified]**
- Billed as "the first active SLAM system utilizing 3DGS". Uses Fisher information to trade off information gain against localization risk. Evaluated on Gibson and HM3D simulation, scene-level — [arXiv](https://arxiv.org/html/2410.17422v1)

**ActiveSplat: High-Fidelity Scene Reconstruction through Active Gaussian Splatting** (Li et al.). RA-L 2025, arXiv 2410.21955 **[partly verified: id + venue via search]**
- Scene-level active 3DGS mapping (RGB-D, per prior knowledge) — [arXiv](https://export.arxiv.org/pdf/2410.21955)

**ActiveGS: Active Scene Reconstruction Using Gaussian Splatting** (Jin, Zhong, Pan, Behley, Stachniss, Popović; Bonn). RA-L 2025, arXiv 2412.17769 **[verified]**
- Hybrid map of 3DGS plus a coarse voxel map, with **confidence modelling per Gaussian** to find under-reconstructed areas. Uses an **RGB-D camera**. "Demonstrates applicability in the real world using an unmanned aerial vehicle" — [arXiv](https://arxiv.org/pdf/2412.17769); [Lamarr](https://lamarr-institute.org/publication/activegs-active-scene-reconstruction-using-gaussian-splatting/)

**ActiveGAMER: Active GAussian Mapping through Efficient Rendering** (Chen et al.). CVPR 2025, arXiv 2501.06897 **[partly verified: id + venue]**
- Rendering-based information gain for active 3DGS mapping, scene-level, RGB-D (per prior knowledge) — [CVF PDF](https://openaccess.thecvf.com/content/CVPR2025/papers/Chen_ActiveGAMER_Active_GAussian_Mapping_through_Efficient_Rendering_CVPR_2025_paper.pdf)

**GauSS-MI: Gaussian Splatting Shannon Mutual Information for Active 3D Reconstruction** (Xie, Cai, Zhang, Yang, Pan; HKU). RSS 2025, arXiv 2504.21067 **[verified]**
- A per-Gaussian probabilistic model of *visual* uncertainty, with Shannon mutual information as a real-time view criterion. Explicitly argues that prior work "evaluat[es] geometric completeness without direct evaluation of the visual uncertainty". Real-world validation is claimed on the project/code page (not re-checked here) — [arXiv](https://arxiv.org/abs/2504.21067); [RSS](https://roboticsproceedings.org/rss21/p030.html); [code](https://github.com/JohannaXie/GauSS-MI)

**VIN-NBV: A View Introspection Network for Next-Best-View Selection** (N. Frahm, Zhao, Dunn Beltran, Alterovitz, J.-M. Frahm, Oliva, Sengupta; UNC). arXiv 2505.06219 (preprint, v3) **[verified]**
- A lightweight network predicts the **Relative Reconstruction Improvement (RRI)** of a candidate view without acquiring it. Gives about a 30% quality gain over a coverage criterion and about 40% over the RL baselines Scan-RL and GenNBV — [arXiv](https://arxiv.org/abs/2505.06219)
- **Assumes RGB-D** acquisitions back-projected to point clouds. The paper notes depth "can come from monocular estimation or MVS". Object-centric (OmniObject3D). **Simulated drone only**, no real robot. Metrics are **Chamfer distance**, coverage and F1, under a fixed capture budget (e.g. 20 captures) — [arXiv HTML v3](https://arxiv.org/html/2505.06219v3)
- Not radiance-field-based: it uses a point-cloud reconstruction.

**Hestia: Voxel-Face-Aware Hierarchical Next-Best-View Acquisition for Efficient 3D Reconstruction** (C.-Y. Lu et al., UTS/Brown). WACV 2026, arXiv 2508.01014 **[verified]**
- Learned hierarchical 5-DoF NBV with "close-greedy" and face-aware design. Reports "92% CR with only 5 acquisitions, whereas prior work requires 15 images", "reducing Chamfer Distance by 50%", and at least 12% better coverage at a 5-image budget. Budgets tested were 5, 15 and 30 images — [arXiv HTML v3](https://arxiv.org/html/2508.01014v3); [WACV 2026](https://wacv.thecvf.com/virtual/2026/poster/662)
- Trained on RGB-D in Isaac Lab with a Crazyflie drone model. **Real-world test uses a drone with an RGB camera plus a learned depth predictor (MASt3R/DUSt3R)**, so it is effectively RGB-only on hardware. Object-centric. Hardware validation is described as limited — [arXiv HTML v3](https://arxiv.org/html/2508.01014v3)

**PUN, "Peering into the Unknown: Active View Selection with Neural Uncertainty Maps for 3D Reconstruction"** (Zhengquan Zhang, Feng Xu, Mengmi Zhang; Fudan/NTU). ICLR 2026, arXiv 2506.14856 **[verified]**
- UPNet predicts an uncertainty map over all candidate viewpoints **from a single image**, with no NeRF/3DGS trained during selection. Reaches "comparable reconstruction accuracy" to the upper bound "using half of the viewpoints", runs up to 400x faster and cuts CPU/RAM/GPU use by more than 50%. Object-centric; RGB input — [arXiv](https://arxiv.org/pdf/2506.14856); [ICLR](https://iclr.cc/virtual/2026/poster/10008352)

**Auto3R: Automated 3D Reconstruction and Scanning via Data-driven Uncertainty Quantification** (Shen et al.). arXiv 2512.04528 **[partly verified: abstract]**
- A learned uncertainty distribution over candidate viewpoints. Objects and scenes, including specular materials. **Real deployment on a robot-arm camera** — [arXiv](https://arxiv.org/abs/2512.04528)

**ObjSplat: Geometry-Aware Gaussian Surfels for Active Object Reconstruction** (Y. Li, Jia, Y. Zhang, Hao, S. Zhang). IEEE T-ASE, arXiv 2601.06997 **[partly verified: abstract]**
- Gaussian-surfel object model scored by back-face visibility and occlusion-aware covisibility, with a multi-step next-best-*path* planner. **Real-world tests on cultural artifacts**. Reports fidelity, completeness, scan time and path length. Sensor (RGB vs RGB-D) and platform are not stated in the abstract — [arXiv](https://arxiv.org/abs/2601.06997)

**R3-RECON: Radiance-Field-Free Active Reconstruction via Renderability** (Jin, Frosi, Guo, Matteucci; PoliMi). arXiv 2601.07484 **[partly verified]**
- A voxel "renderability field" correlated with image-space error, queried in milliseconds with **no online radiance-field training**. Scene-level (Replica), no real robot — [arXiv](https://arxiv.org/abs/2601.07484)

**MAP-NBV: Multi-agent Prediction-guided NBV Planning for Active 3D Object Reconstruction**. arXiv 2307.04004 **[verified]**
- Predicts the unobserved geometry; multi-agent (UAVs in AirSim, ShapeNet); 19% better than non-predictive multi-agent planning. **Not radiance-field based**, and it uses point-cloud geometry — [arXiv](https://arxiv.org/abs/2307.04004v1)

**Other 2025–2026 items surfaced, not examined** **[title/id only]**
- AREA3D (feed-forward 3D perception plus vision-language guidance), arXiv 2512.05131 — [arXiv](https://arxiv.org/pdf/2512.05131)
- MAGICIAN (imagined Gaussians for long-horizon active mapping), arXiv 2603.22650 — [arXiv](https://arxiv.org/pdf/2603.22650)
- DynActiveGS (dynamic scenes), arXiv 2608.01178 — [arXiv](https://arxiv.org/pdf/2608.01178)
- NBV for semantic and dynamic 3DGS, arXiv 2512.22771 — [arXiv](https://arxiv.org/pdf/2512.22771)
- DAV-GSWT, arXiv 2602.15355 — [arXiv](https://arxiv.org/pdf/2602.15355)
- "Active3D: Active High-Fidelity 3D Reconstruction via Hierarchical Uncertainty Quantification" (Li, Li, Lee, 2025), cited in a search snippet; no arXiv id found.
- Neural Visibility Field (Xue et al., visibility-based uncertainty, CVPR 2024 per prior knowledge) appeared in search snippets; id not verified.

### Inferences
- Fisher/Hessian-based signals (FisherRF, POp-GS, AG-SLAM, and PUP's sensitivity score) are the dominant principled 3DGS uncertainty family since 2024. All of them require the 3DGS to be trained online during capture.
- Learned "predict-the-gain" methods (SO-NeRF, NeU-NBV, VIN-NBV, PUN, Auto3R, Hestia) remove online radiance-field training. That makes them the closest competitors to a lightweight on-drone NBV loop.

### Gaps
- No paper titled "Active3DGS" was found. The brief's name may refer to ActiveGS (2412.17769), ActiveSplat or Active3D. **[unverified]**
- The Google Scholar "cited-by" pages for FisherRF and NeU-NBV were not crawled, since Scholar is not reachable with these tools. Coverage of 2025–2026 citers is therefore partial.

## Q1. Which methods select views RGB-only and are validated on a real robot? On a real drone?

### Takeaway
On a real drone, Hestia is the closest match: RGB camera plus a learned depth predictor, object-centric, real-world feasibility shown. Zeng et al. (2209.13159) also flew a real UAV, but its sensing was not verified. ActiveGS flew a real UAV but uses RGB-D and targets scenes. GS-Planner targets a quadrotor in simulation only.

### Cited Findings
- Hestia's real-world deployment uses "a drone equipped with an RGB camera" and "a depth predictor to convert multi-view RGB images into depth maps" (MASt3R/DUSt3R); hardware evaluation is limited — [Hestia arXiv v3](https://arxiv.org/html/2508.01014v3)
- Zeng et al. "deployed on a real UAV" — [arXiv 2209.13159](https://arxiv.org/pdf/2209.13159)
- ActiveGS uses an RGB-D camera and real-world UAV validation — [arXiv 2412.17769](https://arxiv.org/pdf/2412.17769)
- GS-Planner targets a quadrotor but reports simulation only — [arXiv 2405.10142](https://arxiv.org/abs/2405.10142v1)
- VIN-NBV used a simulated drone only and assumes RGB-D — [arXiv 2505.06219v3](https://arxiv.org/html/2505.06219v3)
- RT-GuIDE has real-world experiments (platform: ground robot, from prior knowledge) — [arXiv 2409.18122](https://arxiv.org/pdf/2409.18122)
- Real arm-based, object-level work:
  - Auto3R uses a robot arm — [arXiv 2512.04528](https://arxiv.org/abs/2512.04528)
  - ObjSplat was tested on real cultural artifacts, platform not stated — [arXiv 2601.06997](https://arxiv.org/abs/2601.06997)

### Inferences
- RGB-only selection methods with no real robot are common: ActiveNeRF, FisherRF, POp-GS, NeRF Director, SO-NeRF, NeU-NBV and PUN all work offline on image pools.
- RGB-only *and* real-drone *and* object-centric is essentially Hestia alone, and Hestia replaces true monocular reasoning with a learned depth predictor.

### Gaps
- GauSS-MI's real-world platform was not verified.
- Whether NeU-NBV or Lee et al. 2022 had real-robot runs was not verified.

## Q2. Which evaluate geometry (Chamfer, F-score, completeness) rather than only novel-view PSNR?

### Takeaway
The geometric evaluators are mainly the mapping/learned-policy line: NeurAR, NARUTO, GS-Planner, RT-GuIDE, VIN-NBV, Hestia, GenNBV (coverage), ObjSplat (completeness) and MAP-NBV. The Fisher-information view-selection line (FisherRF, POp-GS) and ActiveNeRF/NeRF Director report rendering metrics. GauSS-MI explicitly targets visual (not geometric) uncertainty.

### Cited Findings
- VIN-NBV reports Chamfer distance, coverage and F1 — [arXiv](https://arxiv.org/html/2505.06219v3)
- Hestia reports coverage ratio, Chamfer distance (-50%) and AUC — [arXiv](https://arxiv.org/html/2508.01014v3)
- NeurAR reports "geometry quality of reconstructed 3D models" — [arXiv](https://arxiv.org/abs/2207.10985)
- RT-GuIDE reports "16% higher geometric reconstruction accuracy" — [arXiv](https://arxiv.org/pdf/2409.18122)
- GenNBV reports coverage ratio — [CVF](https://openaccess.thecvf.com/content/CVPR2024/html/Chen_GenNBV_Generalizable_Next-Best-View_Policy_for_Active_3D_Reconstruction_CVPR_2024_paper.html)
- GauSS-MI positions itself against methods that evaluate "geometric completeness" — [arXiv](https://arxiv.org/abs/2504.21067)
- NeRF Director reports PSNR — [arXiv](https://arxiv.org/abs/2406.08839v1)

### Inferences
- None of the geometric evaluators found scores a **COLMAP SfM→MVS** product against a ground-truth mesh. They score their own point cloud (back-projected depth) or their own neural/3DGS surface.

### Gaps
- Exact per-paper F-score thresholds were not extracted.

## Q3. Which explicitly measure "number of images needed to reach a quality level"?

### Takeaway
Only a few methods report views-to-target. Most report quality at a fixed budget (5, 10 or 20 views) or as curves.

### Cited Findings
- NeRF Director: 70 vs 150 views to reach 33 dB PSNR (FVS vs random) — [arXiv](https://arxiv.org/abs/2406.08839v1)
- Hestia: 92% coverage with 5 acquisitions vs 15 for prior work — [arXiv](https://arxiv.org/html/2508.01014v3)
- PUN: comparable accuracy with "half of the viewpoints" of the upper bound — [arXiv](https://arxiv.org/pdf/2506.14856)
- VIN-NBV: fixed-budget comparisons, e.g. 20 captures — [arXiv](https://arxiv.org/html/2505.06219v3)

### Inferences
- Kinetix's planned metric ("images needed to reach 95% of a dense capture's F-score") does not appear verbatim in any paper found. NeRF Director's PSNR-threshold analysis is the closest precedent. The 50–80% savings Kinetix claims overlap the reported ranges: about 53% for NeRF Director, about 67% for Hestia's coverage, and about 50% for PUN.

### Gaps
- Budgets for FisherRF, ActiveGS and GauSS-MI were not extracted from the full papers.

## Q4. COLMAP/MVS-compatible output vs a bespoke NeRF/3DGS?

### Takeaway
Almost all methods in this catalogue require their own online NeRF/3DGS (or their own depth-fused point cloud) both to plan and to report results. Methods whose selection is decoupled from the final reconstruction are the closest to COLMAP compatibility: SO-NeRF ("model-agnostic"), PUN, NeU-NBV, R3-RECON ("radiance-field-free") and Hestia/VIN-NBV, which are point-cloud based. None reports a COLMAP SfM→MVS evaluation (not found in the abstracts checked).

### Cited Findings
- SO-NeRF: "without … training any radiance field during planning", "model-agnostic" — [arXiv](https://arxiv.org/pdf/2312.03266)
- PUN: no NeRF/3DGS is learned during selection — [arXiv](https://arxiv.org/pdf/2506.14856)
- R3-RECON: "radiance-fields-free" — [arXiv](https://arxiv.org/abs/2601.07484)
- FisherRF, POp-GS, ActiveGS, GauSS-MI and AG-SLAM compute their criterion on the online 3DGS — [FisherRF](https://arxiv.org/abs/2311.17874v1); [POp-GS](https://arxiv.org/abs/2503.07819v1); [ActiveGS](https://arxiv.org/pdf/2412.17769); [GauSS-MI](https://arxiv.org/abs/2504.21067)
- Most online systems assume known poses from simulation or SLAM.

### Inferences
- A method whose plan uses a cheap RGB-only signal, and whose captured set is then handed to unmodified COLMAP for evaluation, sits in a gap. The questions this raises (SfM registration success, baseline/triangulation angle, feature overlap) are not addressed by radiance-field uncertainty methods.

### Gaps
- Whether any 2025–2026 paper evaluates NBV-selected sets through COLMAP MVS could not be confirmed. A targeted search on "next-best-view COLMAP MVS photogrammetry drone 2025" is recommended.

## What remains open for an RGB-only, object-centric, drone-validated, COLMAP-compatible NBV method

### Takeaway
No surveyed method combines all of the following:
- monocular RGB plus drone pose as the only planning input (no depth sensor, no learned depth prior required);
- a single object;
- real-drone flights;
- planning not tied to an online-trained NeRF/3DGS;
- output evaluated through standard COLMAP SfM→MVS/3DGS against a ground-truth mesh (Chamfer/F-score);
- savings reported as images needed to reach a fraction of dense-capture quality.

The nearest neighbours each miss at least two of these:
- **Hestia** uses learned depth, its output is a point cloud rather than COLMAP, and its hardware validation is limited.
- **ActiveGS** uses RGB-D, targets scenes, and requires its own 3DGS.
- **VIN-NBV** uses RGB-D and was tested only on a simulated drone.
- **FisherRF/POp-GS** select offline, report PSNR, and have no drone.
- **GS-Planner** was tested in simulation only.
- **PUN/SO-NeRF** have no robot and no COLMAP evaluation.

### Cited Findings
- See the per-method citations in Q0–Q4: [Hestia](https://arxiv.org/html/2508.01014v3), [ActiveGS](https://arxiv.org/pdf/2412.17769), [VIN-NBV](https://arxiv.org/html/2505.06219v3), [FisherRF](https://arxiv.org/abs/2311.17874v1), [GS-Planner](https://arxiv.org/abs/2405.10142v1), [PUN](https://arxiv.org/pdf/2506.14856), [SO-NeRF](https://arxiv.org/pdf/2312.03266)

### Inferences
- Kinetix's defensible novelty lies in the *combination*:
  - a monocular SfM-aware NBV signal, such as covisibility/triangulation-angle/registrability rather than radiance uncertainty;
  - evaluation through unmodified COLMAP MVS with geometric metrics;
  - a "views-to-95%-of-dense F-score" protocol;
  - real indoor small-drone validation;
  - stereo as an ablation.
- Novelty would be weak if Kinetix claimed a new uncertainty signal for 3DGS (crowded: FisherRF, POp-GS, GauSS-MI, ActiveGS) or simply "fewer images than dense capture". NeRF Director, Hestia and PUN already report about 50–67% reductions.
- Kinetix should benchmark against FisherRF (RGB-only, open source), Hestia (drone, object, RGB with learned depth) and a coverage/FVS baseline, since NeRF Director shows that FVS is a strong baseline.

### Gaps
- Unchecked items: GauSS-MI's real-world platform, NeU-NBV's real-robot status, the venue of Zeng et al. 2209.13159, the arXiv id for "Active3D", and full-text budgets for ActiveGS and FisherRF.
- Google Scholar cited-by lists for FisherRF and NeU-NBV were not crawled. 2026 drone-specific works may be missing.
