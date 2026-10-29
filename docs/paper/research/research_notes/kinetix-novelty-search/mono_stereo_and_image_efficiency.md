# Mono vs stereo vs RGB-D in active reconstruction, and image-set minimisation for SfM/MVS (Kinetix novelty check)

Scope: (A) sensing-modality comparisons inside active reconstruction / NBV planners, including "depth for planning" vs "depth for reconstruction"; (B) view / image-subset selection and capture planning for SfM-MVS, with the reduction numbers reported; (C) "images-to-quality" metrics. Search date 2026-10-08. Numbers marked "(computed)" are my arithmetic on the paper's tables. Items marked "[not verified this session]" come from background knowledge; I did not re-check them against a source in this search.

---

## Q1. Is a controlled mono-vs-stereo (or mono-vs-RGB-D) comparison inside one NBV planner already published? Is "depth for planning" separated from "depth for reconstruction"?

### Takeaway
I found no published work that keeps one NBV planner fixed, swaps only the sensing modality (monocular RGB vs small-baseline stereo vs RGB-D), and then separately varies whether depth is used for planning and whether it is used for reconstruction. The nearest work falls into three groups:
- monocular active systems benchmarked against *different* RGB-D planners (ActMVS 2026);
- stereo-pair selection that builds on a monocular NBV (Next-Best Stereo, BMVC 2016);
- capture planners that use RGB only and run online MVS to drive the NBV (Huang et al., toy drone).

The 2×2 factorial (planning-depth × reconstruction-depth) that Kinetix proposes looks open.

### Cited Findings
- **ActMVS (Pu, Han, Lian; arXiv 2606.01367, 31 May 2026)** calls itself "the first framework for monocular active reconstruction". Because no monocular baseline existed, the authors compare it with two RGB-D active methods, **ActiveGS** and **NARUTO**. These are different planners and representations, so this is not a controlled swap inside one planner. Setup: Habitat + Replica, 90° FoV, 512×512. Metrics are Accuracy, Completion and Chamfer (cm), plus PSNR/SSIM/LPIPS over 1000 random novel views. Example (Room0): ActiveGS (RGB-D) Chamfer 1.180 cm, NARUTO (RGB-D) 1.763 cm, ActMVS (RGB) 2.397 cm. Office1: ActiveGS 0.814 vs ActMVS 3.409. The authors say ActMVS "necessitates denser camera trajectories": runtime is about 1200 s against 300 s for ActiveGS. — [ActMVS arXiv PDF](https://arxiv.org/pdf/2606.01367)
- Within its own planner, ActMVS ablates the depth source. **"w/o MVS"** replaces the MVS depth with a state-of-the-art monocular metric-depth network, and Chamfer degrades from about 2.4 cm to about 46 cm on Room0. This is a controlled *depth-source* ablation inside one planner (MVS-from-monocular vs single-image depth). It does not cover stereo vs RGB-D, and in ActMVS the depth serves both planning (occupancy map) and reconstruction. — [ActMVS arXiv PDF](https://arxiv.org/pdf/2606.01367)
- ActMVS states that existing active-reconstruction systems "predominantly rely on depth sensors" for occupancy-map updates. — [ActMVS arXiv PDF](https://arxiv.org/pdf/2606.01367)
- **Next-Best Stereo (Mendez, Hadfield, Pugeault, Bowden; BMVC 2016)**. Part 1 is a monocular NBV that optimises accuracy and coverage from partial geometry. Part 2 selects *variable-baseline stereo pairs* by jointly optimising baseline and vergence. Each step costs 0.8 ms and 0.3 ms per pose. On Middlebury, the front and back of the models are reconstructed from at most **14 images**, which is **3.8% (dino) and 4.4% (temple)** of the views, with results "comparable to state-of-the-art batch approaches". This is stereo built from a moving monocular camera (wide, variable baseline), not a fixed small-baseline rig against monocular. — [BMVC 2016 paper PDF](https://www.bmva-archive.org.uk/bmvc/2016/papers/paper065/paper065.pdf); [BMVA archive page](https://bmva-archive.org.uk/bmvc/2016/papers/paper065/index.html)
- **Active Image-based Modeling with a Toy Drone (Huang, Zou, Vaughan, Tan; arXiv 1705.01010)**. A monocular Parrot Bebop runs fast online MVS on low-resolution frames (640×368) to plan NBVs, then shoots 1920×1080 frames for offline reconstruction. So low-quality monocular depth is used for planning and high-resolution images for reconstruction, which is a "planning vs reconstruction" split, but only within the monocular modality. (Venue reported elsewhere as ICRA 2018 [not verified this session].) — [ar5iv](https://ar5iv.arxiv.org/html/1705.01010)
- **Mostegel et al. (CVPR 2016 workshops)** use already-acquired images to predict confidence in dense MVS output *without running MVS*, and use that prediction to drive autonomous UAV image acquisition. This is an RGB-only planning signal for MVS quality. — [CVPRW 2016 PDF](https://openaccess.thecvf.com/content_cvpr_2016_workshops/w3/papers/Mostegel_UAV-Based_Autonomous_Image_CVPR_2016_paper.pdf); [arXiv 1605.01923](https://arxiv.org/pdf/1605.01923)
- **Forster et al. (RSS 2014)**, active monocular dense reconstruction: choose motion to minimise expected depth uncertainty given scene texture. This is monocular planning only, with no modality comparison. — [RSS14 PDF](https://infoscience.epfl.ch/record/203672/files/RSS14_Forster.pdf?version=1)
- **Plan3D (Hepp et al., ACM TOG 2018)** separates planning input from reconstruction pipeline in a different way. It reports F-score with the planner's output reconstructed from *ground-truth depth maps* and, separately, from the *full SfM/MVS pipeline*. Grasslands-small, Plan3D: 79.22 (GT depth) vs 76.02 (SfM/MVS). Roberts 2017: 76.11 vs 72.39. This splits "ideal depth" from "photogrammetric reconstruction" at the reconstruction stage, not the planning stage. — [Hepp et al. arXiv 1705.09314](https://arxiv.org/abs/1705.09314); [project page](https://niessnerlab.org/projects/hepp2018plan3d.html)
- **Multi-sensor NBV as matroid-constrained submodular maximisation (arXiv 2007.02084)** plans views for several sensors jointly. From the search snippet, it concerns multiple cameras or sensors, not a monocular vs stereo vs RGB-D accuracy comparison [content not read in detail]. — [arXiv 2007.02084](https://arxiv.org/pdf/2007.02084)
- A 2022 UAV view/path-planning review groups sensors by type. It notes that RGB-D and stereo give dense real-time depth for TSDF/volumetric maps, and that LiDAR is accurate but sparse. It does not cite a controlled same-planner modality study. — [Review arXiv 2205.03716](https://arxiv.org/pdf/2205.03716)

### Inferences
- The prior-art pattern is: RGB-D/LiDAR planners (ActiveGS, NARUTO, SEE, volumetric NBV), monocular planners that must manufacture their own depth (ActMVS, Huang 2018, Forster 2014), and cross-paper comparisons between the two. A *same-planner, same-scene* comparison of monocular RGB vs a real small-baseline stereo (ZED Mini, 63 mm) vs RGB-D, with planning-depth and reconstruction-depth varied independently, was not found. This is likely Kinetix's clearest novel axis, but it is an *experimental-design* novelty, not an algorithmic one.
- ActMVS (May 2026) is the closest and most recent competitor for "monocular active reconstruction". Kinetix must cite it and position against it. ActMVS targets *scenes*, uses a 3DGS/occupancy representation, and uses learned MVS depth. Kinetix targets a single *object* with COLMAP SfM→MVS/3DGS output and an images-to-quality metric.
- Next-Best Stereo shows the intuition that wide, variable-baseline pairs from a moving monocular camera beat fixed geometry. This sets up Kinetix's question of whether a 63 mm baseline adds anything beyond what motion parallax gives a monocular drone.

### Gaps
- No study found that quantifies small-baseline stereo (≈6 cm) depth against wide-baseline multi-view SfM/MVS for *object* reconstruction accuracy at 1–2 m, in a drone or NBV context. Generic stereo error theory (σ_Z ∝ Z²/(f·B)) is textbook knowledge; I did not fetch a specific comparative paper.
- I did not verify whether ActiveGS or NARUTO report RGB-only ablations.
- GenNBV, MACARONS (Guédon et al., CVPR 2023, RGB-only self-supervised depth for NBV) and NeU-NBV (RGB-only, mapless) are relevant monocular-NBV works [not verified this session]. Check whether any of them runs an RGB vs RGB-D ablation inside its own planner.

---

## Q2. What reduction factors (images or path length) do adaptive / optimised methods report against fixed patterns? Which 50–80% claims already exist?

### Takeaway
Large image-reduction claims already exist. Next-Best Stereo reached comparable results with 3.8–4.4% of the Middlebury views. Huang et al.'s toy drone used 108 adaptive views against 390 for a dense rectangle pattern, about 72% fewer, at similar accuracy. Liu et al. 2022 matched a planner's F-score with about 53% fewer images (computed). Many aerial planners (Roberts 2017, Hepp 2018) instead hold the budget fixed (flight time or images) and report quality gains. A bare "50–80% fewer images than orbit/grid" claim is therefore *not* novel by itself. The novelty has to come from the threshold-referenced metric (N@q) and the object-scale, monocular-vs-stereo setting.

### Cited Findings
- **Huang et al., toy drone (arXiv 1705.01010)**, simulated Table I:

  | Plan | Views | Mean error (in) | RMS (in) | Completeness @1 in (%) | Completeness @2 in (%) |
  |---|---|---|---|---|---|
  | Automated (NBV) | 108 | 1.89 | 2.93 | 35.8 | 76.1 |
  | Rectangle-Dense | 390 | 1.79 | 2.62 | 28.6 | 80.6 |
  | Rectangle-Sparse | 130 | 2.24 | 3.05 | 15.8 | 64.2 |
  | Grid-Multi-Angle | 858 | 3.05 | 3.81 | 5.3 | 31.9 |
  | 2-Circles | 105 | 3.65 | 4.50 | 2.8 | 20.3 |
  | Grid-Downward | 143 | 3.55 | 4.52 | 4.7 | 30.2 |

  108 vs 390 views means **about 72% fewer images (computed)** than the dense pattern at similar accuracy. An outdoor test used 101 views against an exhaustive 328 (**about 69% fewer**, computed). — [ar5iv](https://ar5iv.arxiv.org/html/1705.01010)
- **Next-Best Stereo (BMVC 2016)**: Middlebury dino/temple, at most 14 images, **3.8% / 4.4% of the views**. Results are "comparable to state-of-the-art batch approaches", measured with Middlebury-style accuracy (error at 80/90/99% of points, mm) and completeness (%, at 1.25 mm). The paper's own dataset modes are full (~300 images), ring (~50) and sparse ring (~15). — [BMVC 2016 PDF](https://www.bmva-archive.org.uk/bmvc/2016/papers/paper065/paper065.pdf)
- **Smith et al. 2018, "Aerial Path Planning for Urban Scene Reconstruction: A Continuous Optimization Method and Benchmark" (ACM TOG 37(6):183, doi 10.1145/3272127.3275010)**:
  - On Old-City, the optimised plan reaches **>96% completeness with ≥204 views**, whereas "none of the other methods [nadir grid; oblique grid] reaches 93% even with significantly more images". It is also "more than twice as accurate".
  - On GOTH-1 the comparison is at roughly equal image count: OURS (588) completeness at 0.075 m 58.09% vs NBV (607) 44.41% and SUB = Roberts (605) 49.24%.
  - On CA-1, OURS with 743 images reaches 64.92% vs about 51–53% for the others; 728 images with a 4 m standoff still match or beat them.
  - Metrics: error at the 90%/95% point quantile (m), and completeness (%) at 0.075 / 0.05 / 0.02 m.
  - Smith states plainly: "accurate and complete reconstructions using significantly fewer images".

  — [Smith 2018 PDF (TU Darmstadt)](https://download.hrz.tu-darmstadt.de/media/FB20/GCC/paper/Smith-2018-APP.pdf); [KAUST summary](https://cemse.kaust.edu.sa/articles/2018/10/16/aerial-path-planning-urban-scene-reconstruction-continuous-optimization-method)
- **Roberts et al., ICCV 2017 (submodular trajectory optimisation)**:
  - Controls flight time, battery, *number of images* and reconstruction settings, so the comparison is at equal image count, not "fewer images".
  - Synthetic Grasslands: Overhead 170.2 / 583.8 / 7.1; Random 126.5 / 557.2 / 4.4; Next-Best-View 122.8 / 330.7 / 3.6; Ours 115.2 / 323.3 / 3.3 (accuracy, completeness, per-pixel visual error %; lower is better; units of the first two columns not captured in my extract).
  - Ours gets **20% more submodular reward than NBV** at a 960 m (8 min) budget.

  — [arXiv 1705.00703](https://arxiv.org/abs/1705.00703v1); [CVF open access](https://openaccess.thecvf.com/content_iccv_2017/html/Roberts_Submodular_Trajectory_Optimization_ICCV_2017_paper.html)
- **Plan3D (Hepp, Nießner, Hilliges; ACM TOG 2018)**, at an equal travel budget (Grasslands-large, 1500 m), F-score:

  | Plan | F-score |
  |---|---|
  | Plan3D | **70.49** |
  | Small hemisphere | 64.24 |
  | Large hemisphere | 62.29 |
  | Roberts 2017 | 63.76 |
  | Greedy NBV | 59.74 |
  | Small circle | 22.64 |
  | Large circle | 7.44 |
  | Meanders | ~27.6–27.95 |

  Reported as a quality gain at fixed path length, not as an image reduction. — [arXiv 1705.09314](https://arxiv.org/abs/1705.09314)
- **Liu et al. 2022, "Learning Reconstructability for Drone Aerial Path Planning" (ACM TOG 41(6):197, SIGGRAPH Asia 2022)**:
  - F-score at a 10 cm threshold (Knapitsch/Tanks-and-Temples convention).
  - Table 6, Box proxy: their planner **764 images, F 34.96** vs Zhou et al. planner **1610 images, F 34.55**, about **53% fewer images at equal F** (computed).
  - Table 3, Box proxy with the Smith planner: learned predictor 1,610 images, F 34.55 vs Smith heuristic 1,770 images, F 31.85.
  - Accuracy and completeness are also reported at 70/80/90/95% quantiles.

  — [arXiv 2209.10174](https://arxiv.org/pdf/2209.10174)
- **Zhou et al. 2020** (cited by Liu 2022 and the Smith follow-ups) uses MAXIMIN optimisation to maximise reconstructability under a fixed number of viewpoints. — via [Liu 2022](https://arxiv.org/pdf/2209.10174) [primary not fetched]
- **DynaView (2026)** combines a Transformer reconstructability predictor (offline) with online marginal-utility viewpoint updates. — [Nottingham Ningbo record](https://research.nottingham.edu.cn/en/publications/dynaview-bridging-offline-learning-and-online-adaptation-for-uav-/) [numbers not read]
- **Hornung, Zeng, Kobbelt, CVPR 2008, "Image Selection for Improved Multi-View Stereo"**: incremental image selection that guarantees proxy coverage without redundancy, plus extra views for low photo-consistency regions such as cavities. Tested with four Middlebury MVS methods, it gives better quality than uniformly distributed views and "reduc[es] processing time considerably". — [RWTH PDF](https://www.graphics.rwth-aachen.de/media/papers/hornung_2008_cvpr_011.pdf); [mlanthology](https://mlanthology.org/cvpr/2008/hornung2008cvpr-image/)
- **Koch, Körner, Fraundorfer 2019, Remote Sensing 11(13):1550, doi 10.3390/rs11131550**: semantically-aware 3D UAV flight planning after an initial flight. It extracts the target object and restricted airspace, and trades flight length against resolution and photogrammetric constraints. — [MDPI](https://www.mdpi.com/2072-4292/11/13/1550/reprints); [TUM PDF](https://mediatum.ub.tum.de/doc/1520062/document.pdf) [reduction numbers not read]
- **Photogrammetric network design (classic)**:
  - Fraser (1984), "Network design considerations for non-topographic photogrammetry", PE&RS.
  - Mason (1995), "Expert system-based design of close-range photogrammetric networks", ISPRS J. P&RS; Mason 1995/1997 design a generic four-camera network per planar primitive of a CAD model.
  - Olague & Mohr (1998), INRIA report on optimal camera placement.
  - Olague (2002): genetic-algorithm network design optimising accuracy (Grafarend–Sansò design orders), PE&RS May 2002 pp. 423–431.

  — [ISPRS Archives XXXV comm5 paper 531](https://www.isprs.org/proceedings/XXXV/congress/comm5/papers/531.pdf); [ASPRS PE&RS 2002 PDF](https://www.asprs.org/wp-content/uploads/pers/2002journal/may/2002_may_423-431.pdf); [FIG Saadatseresht et al.](https://fig.net/resources/proceedings/fig_proceedings/athens/papers/ts26/TS26_7_Saadatseresht_et_al.pdf)
- **3DGS / NeRF view selection**:
  - ActiveInitSplat (Polyzos et al., arXiv 2503.06859, v2 Dec 2025) actively selects a small training set with a GP surrogate (density + occupancy objective) against random, standard and FVS baselines. Bonsai example: PSNR 22.60 vs 18.81 (passive-standard). Results are reported at an equal image count (T = 30–70), not as an images-to-quality reduction. — [arXiv html](https://arxiv.org/html/2503.06859v2)
  - FisherRF / Next Best Sense (depth and colour uncertainty for NBV in 3DGS) and frequency-based view selection (arXiv 2409.16470) also address few-view 3DGS. — [Next Best Sense](https://arxiv.org/html/2410.04680v3); [Frequency-based view selection](https://arxiv.org/html/2409.16470v1)
- **Practitioner image-count studies**: an OpenScan turntable study finds mesh-quality metrics (RMS, SD, normal consistency) plateau around 292–333 photos, with about 150 "great" and 300 a practical maximum. This is a non-peer-reviewed blog. — [OpenScan blog](https://blog.openscan.eu/posts/optimizing-3d-scans-how-many-photos-do-you-really-need/)
- **UAV image-overlap studies**: the best results come with >95% overlap. — [Remote Sensing 10(6):912](https://www.mdpi.com/2072-4292/10/6/912)

### Inferences
- Reductions already claimed or computable against fixed patterns or competing planners:
  - **~70% fewer images** at similar accuracy (Huang 2018, object/building scale, monocular drone);
  - **~96% fewer** (Next-Best Stereo, against the full Middlebury ring, at "comparable" quality);
  - **~53% fewer** (Liu 2022 vs Zhou 2020 planner, equal F-score);
  - qualitative "significantly fewer images" (Smith 2018, against nadir and oblique grids).

  Kinetix's 50–80% target sits inside the range already reported. Reviewers will ask why it is different.
- Most aerial-planning papers (Roberts, Hepp, Smith's GOTH/CA tables, ActiveInitSplat) fix the budget and report quality, rather than fixing quality and reporting images. Kinetix's fixed-quality-target framing is the less common one.
- Baseline choice inflates or deflates the reduction. In Huang 2018, "Grid-Multi-Angle" used 858 views at *worse* quality than the 108-view NBV, so a naive "88% fewer images" could be claimed against it. Kinetix should report against the strongest fixed pattern (a dense multi-ring orbit) and against a strong adaptive baseline.

### Gaps
- **Furukawa et al., CVPR 2010, "Towards Internet-scale Multi-view Stereo" (CMVS)** clusters and removes redundant images under coverage and size constraints [not verified this session]. It is a classic citation for offline image-subset selection and should be added with its own source.
- Further items not fetched: Schmid/Fraundorfer UAV view planning, Mauro et al. "unified framework for content-aware view selection and planning" (2014), Hoppe et al. 2012 online feedback for SfM image acquisition, Bircher et al. 2016 / Song & Jo object-NBV.
- No 2024–2026 paper found that reports "images needed for 3DGS to reach X% of the full-capture PSNR or F-score" for an active capture policy. The search was not exhaustive.

---

## Q3. Does any work use an "images-to-quality" metric (images needed to reach a quality threshold)? Which metric conventions are standard?

### Takeaway
Close relatives of N@q exist, but I found no exact match:
- views (or time, or distance) needed to reach X% coverage, common in NBV/SEE-style object scanning;
- AUC of coverage against the number of views (SCONE);
- quality-vs-#views curves (Smith 2018 Fig. 11).

I did not find "images needed to reach q = 95% of the dense-capture F-score" defined *relative to a dense-capture reference*. The standard geometric metrics are F-score@τ (Tanks-and-Temples / Knapitsch 2017), precision/recall, accuracy/completeness at percentile thresholds (Middlebury), and Chamfer distance. NBV papers add surface-coverage ratio and its AUC.

### Cited Findings
- **SCONE (Guédon et al., arXiv 2208.10449)** evaluation: start from a random pose, run 9 NBV iterations (10 views), and score the **AUC of surface coverage over the sequence**. This "evaluates both the quality of final surface coverage and the convergence speed". — [SCONE arXiv](https://arxiv.org/pdf/2208.10449) (as summarised by search)
- **Surface coverage** in NBV benchmarking is the ratio of observed model points to total model points, where a point counts as observed if a measurement lies within a registration distance. Other evaluations report "explored volume ≥90% with 2–3 fewer views per camera" or final coverage after a fixed number of scans (e.g., 15). — search summary of [SEE arXiv 1802.08617](https://arxiv.org/pdf/1802.08617) / [Proactive occlusion NBV arXiv 2009.04515](https://arxiv.org/pdf/2009.04515) / [MuTr07 benchmark](https://pub.inf-cv.uni-jena.de/pdf/MuTr07) [individual attributions not verified]
- **SEE (Border & Gammell, IJRR 43(10):1506–1532, 2024)** reports surface coverage against observation time and travel distance, "similar or better surface coverage with less observation time and travel distance than evaluated volumetric approaches". — [arXiv 2207.13684](https://arxiv.org/abs/2207.13684)
- An NBV benchmarking paper (Jena, "MuTr07") notes that NBV methods are hard to compare because each uses its own objects and quality measures. — [Benchmarking 3D Reconstructions from NBV Planning](https://pub.inf-cv.uni-jena.de/pdf/MuTr07)
- **Smith 2018 Fig. 11** plots accuracy and completeness against the number of views for nadir-grid, oblique and optimised plans. The paper reads off threshold statements ("completeness >96% for ≥204 views; others never reach 93%"), which is effectively an images-to-quality reading but is not formalised as a metric. — [Smith 2018 PDF](https://download.hrz.tu-darmstadt.de/media/FB20/GCC/paper/Smith-2018-APP.pdf)
- **F-score@τ** (Knapitsch et al. 2017), with τ = 10 cm for urban drone scenes, is used by Liu 2022. Plan3D reports precision, recall and F-score on dense point clouds. — [Liu 2022](https://arxiv.org/pdf/2209.10174); [Hepp 2018](https://arxiv.org/abs/1705.09314)
- **Percentile accuracy and threshold completeness** (Middlebury style): Smith 2018 uses error at 90%/95% and completeness at 0.075/0.05/0.02 m. Next-Best Stereo uses error at 80/90/99% and completeness at 1.25 mm. Huang 2018 uses mean/RMS error and completeness at 1 and 2 inches. — sources above
- **Chamfer / Accuracy / Completion (cm)** plus PSNR/SSIM/LPIPS is the convention in neural active reconstruction (ActMVS, ActiveGS, NARUTO comparisons). — [ActMVS](https://arxiv.org/pdf/2606.01367)

### Inferences
- **N@q** (images needed to reach 95% of a dense capture's F-score) seems to be a new *formalisation*. It inverts the usual quality-at-budget table and normalises by a per-object dense-capture ceiling, which handles object-to-object difficulty. It should be presented alongside conventional metrics (F-score@τ, Chamfer, coverage AUC), and the AUC-of-quality-vs-images convention from SCONE should be credited as the nearest prior.
- Normalising to "95% of dense capture F" rather than an absolute F makes N@q depend on how dense the reference capture is. That reference (image count, pattern) has to be fixed and reported.

### Gaps
- I did not find whether 3DGS active-view papers (FisherRF, ActiveNeRF, ActiveGS, AGILE-GS) report "views to reach a PSNR threshold". The search was not exhaustive.
- The exact AUC definition in GenNBV, MACARONS and PC-NBV was not checked.

---

## Implications for Kinetix's claims (what is taken, what is open)

**Taken (prior art Kinetix must cite and cannot claim):**
1. **Adaptive NBV with a monocular drone camera using online MVS to plan, then high-resolution images to reconstruct**: Huang et al. (toy drone, 2017/18), Mostegel 2016, Forster 2014. Monocular active *scene* reconstruction competitive with RGB-D: **ActMVS (May 2026)**, which claims to be "the first monocular active reconstruction framework".
2. **Large image reductions against fixed patterns**: about 70% fewer images than a dense rectangle pattern at similar accuracy (Huang); comparable results with 3.8–4.4% of views (Next-Best Stereo); about 53% fewer images at equal F (Liu 2022 vs Zhou 2020); "significantly fewer images" than nadir and oblique grids (Smith 2018). A 50–80% reduction claim is **not novel in itself**.
3. **Capture planning for MVS with reconstructability heuristics or learned predictors**: Roberts 2017, Hepp 2018, Smith 2018, Koch 2019, Zhou 2020, Liu 2022, DynaView 2026. Offline image selection: Hornung 2008, CMVS [verify]. Classic network design: Fraser 1984, Mason 1995/97, Olague 2002.
4. **Metric conventions**: F-score@τ, accuracy/completeness at thresholds, Chamfer, coverage AUC over the view sequence (SCONE).

**Open (where Kinetix can claim novelty, given the evidence found):**
1. **A controlled, same-planner comparison of monocular RGB vs small-baseline stereo (ZED Mini, 63 mm)**, ideally also RGB-D, for *object* NBV photogrammetry. No such study was found. ActMVS compares across different planners; Next-Best Stereo uses wide variable baselines from a moving monocular camera.
2. **A factorial split of "depth for planning" vs "depth for reconstruction"** (mono-plan/mono-recon, stereo-plan/mono-recon, mono-plan/stereo-fused-recon, stereo/stereo). No published 2×2 was found. Huang 2018 and Plan3D's GT-depth vs SfM/MVS split are partial precedents and should be cited.
3. **N@q, a threshold-referenced images-to-quality metric normalised to a dense capture of the same object.** Related metrics exist (coverage AUC, views-to-coverage-threshold, Smith's quality-vs-#views curves), but this exact definition was not found. Present it as a formalisation and report standard metrics next to it.
4. **Indoor, single-object, small-drone scale with a COLMAP SfM→MVS and 3DGS dual output.** Most capture-planning work is urban/building scale (Roberts, Hepp, Smith, Liu, Koch), and active 3DGS work is scene scale (ActMVS, ActiveGS) or manipulator scale (Next Best Sense).

**Recommended framing:** do not lead with "50–80% fewer images". Lead with (i) the controlled modality comparison and (ii) the planning-vs-reconstruction depth decomposition. Report N@q as the main efficiency measure, against the strongest fixed pattern and an adaptive baseline, and frame the reduction as *consistent with* prior reports (Huang ~70%, Liu ~53%) rather than as a first.
