# Room photogrammetry with a camera-only indoor drone: what is taken, what is open

Scoping review, 2026-10-28. Question: what has been published on autonomous image acquisition for room-scale photogrammetry by a drone
under the Kinetix constraints, and which problem is still open? Constraints: **good accuracy, no GNSS, as few images as possible,
stereo and monocular RGB, no LiDAR but stereo depth (ZED Mini), as short a flight as possible.** It complements the object-scale
[Kinetix novelty search](kinetix-novelty-search.md) (2026-10-08) and [related_work.md](../related_work.md).

**Bottom line.** Every constraint has been met somewhere, but no paper meets all of them together. The open problem, worded as a research question:

> *For an autonomous, camera-only indoor drone (stereo RGB + stereo depth + VIO, no GNSS, no LiDAR), which acquisition variables
> (distance/GSD, parallax, overlap, image count, speed, pose drift) govern photogrammetric accuracy of a room, and what is the
> fewest-images / shortest-flight acquisition that reaches a stated accuracy?*

The closest system, the **ICON drone** (Zhang, Yang & Zou, BuildSys 2024 and Data-Centric Engineering 2025), already does "autonomous indoor
drone, VIO, RGB-D camera, RGB images → COLMAP/3DGS, scored against a terrestrial laser scan" in real rooms. It plans for *exploration*
(the FUEL frontier planner), not for photogrammetry. It does not minimise images or flight time, and it does not vary the acquisition geometry. Fast
coverage with a viewpoint set, an ordering and a continuous trajectory is taken outdoors (FC-Planner, PredRecon, SOAR), but with LiDAR or a known
map. Reconstructability heuristics built on GSD and parallax are taken outdoors with GNSS poses (Smith 2018, Plan3D, Roberts 2017, Liu
2022). Drift-aware active SLAM is taken for *maps* (RAVEN 2026, Zhang 2022), not for photogrammetric accuracy. Time-budgeted indoor
3DGS exploration appeared in October 2026 (Krisshnakumar et al., IROS 2026), but only in simulation and without pose-error modelling.

## Method

- **Form:** scoping review (chosen by the author). ARS has no scoping mode, so this followed the lit-review workflow. PRISMA-ScR items
  were not all applied: there was one screener (AI-assisted), no registered protocol, and no critical appraisal, which is normal for scoping.
- **Sources:** web search (Brave-backed, 23 queries, all run 2026-10-28) over arXiv, IEEE, ACM, Elsevier, Cambridge, MDPI and ISPRS. Each reference was
  checked against the arXiv API (20 IDs) and Crossref (DOIs). The abstracts of three indoor papers were read through Semantic Scholar. No
  Scopus, Web of Science or IEEE Xplore database export was made.
- **Search concepts:** (indoor | GNSS-denied | room | building interior) × (UAV | MAV | drone) × (photogrammetry | SfM | MVS | 3DGS |
  active reconstruction | coverage / view planning | NBV), plus one search per constraint: VIO pose priors in SfM, motion blur vs
  speed, keyframe / image-subset selection, stereo / RGB-D depth-aided reconstruction, minimum-time scanning, explore-then-exploit,
  localisation-aware planning.
- **Inclusion:** 2015–2026; planning or acquisition for 3D reconstruction by a mobile robot (preferably aerial), or an acquisition
  variable measured against reconstruction quality. Exclusion: pure navigation, pure SLAM benchmarks, outdoor surveying without
  planning content.
- **Read depth:** almost all charting is from **abstracts and search snippets**. The full text was read for ICON (DCE 2025) only, and that read
  stopped at 100 k characters. Everything marked "not stated" below means "not in the abstract", not "absent from the paper".

## Charting: each work against the six constraints

✓ = meets it, ✗ = does not, ? = not stated in what was read. "Accuracy vs GT" means geometric error against a scan or mesh, not PSNR.

| Work | Indoor, no GNSS | Few images | Short flight | Camera only (no LiDAR) | Depth used | Photogrammetric output | Real flight | Accuracy vs GT |
|---|---|---|---|---|---|---|---|---|
| ICON drone, BuildSys 2024 / DCE 2025 [1,2] | ✓ VIO | ✗ (video) | ✗ (exploration only) | ✓ D435i + GoPro | RGB-D for nav; mono depth prior in 3DGS | COLMAP + planar 3DGS | ✓ 3 rooms | ✓ vs TLS: 5–7 cm mean, F1 0.58–0.82 |
| FlyMeThrough, UIST 2025 [3] | ✓ | ✗ | ✗ (piloted) | ✓ RGB video | ✗ | SfM | ✓ 12 spaces | ? |
| Gao et al., AutCon 2023 [4] | ✓ two-step SLAM | ? | explore-then-exploit, two UAVs | ? | ? | ? | ✓ | ? |
| Pan et al., arXiv 2024 [5] | ✓ 4,000 m² facility | ? | ? | ? | ? | neural recon + 3DGS | ✓ | ✗ (tracking error only) |
| Jin et al. 2025 / ASIP-Planner, IROS 2026 [6,7] | ✓ partially known indoor | ? | ? | ? | ? | surface inspection | ? | ? |
| Return-to-Home MAV 3DGS, IROS 2026 [8] | ✓ (sim) | ? | ✓ flight-time budget | ? | TSDF for collision | 3DGS | ✗ sim | coverage + error (sim) |
| ActiveGS, RA-L 2025 [9] | indoor scenes | ? | ? | ✗ RGB-D | ✓ | 3DGS | ✓ UAV demo | ? |
| ActiveSplat RA-L 2025 [10], HGS-Planner ICRA 2025 [11], GauSS-MI RSS 2025 [12] | indoor scenes (sim) | ? | ? | RGB-D | ✓ | 3DGS | ? | rendering / mesh metrics |
| ActMVS, ICRA 2026 [13] | indoor (sim) | ? | ? | ✓ mono | MVS depth | occupancy + 3DGS | ✗ | Chamfer (sim) |
| FC-Planner ICRA 2024 [14], PredRecon ICRA 2023 [15], SOAR IROS 2024 [16] | ✗ outdoor structures | ✓ minimal viewpoints | ✓ fast coverage | ✗ LiDAR / known map | — | MVS / mesh | ✓ | ✓ (sim GT) |
| Smith 2018 [17], Plan3D 2018 [18], Roberts 2017 [19], Zhou 2020 [20], Liu 2022 [21] | ✗ outdoor, GNSS | ✓ | ✓ path length | ✓ RGB | ✗ | MVS | ✓ | ✓ |
| Schmid 2020 [22], two-stage NBV 2017 [23], FUEL 2021 [24] | ✓ indoor | — | ✓ fast exploration | ✗ depth sensor | ✓ | TSDF / map | ✓ | map metrics |
| Schmid 2021 [25], Zhang 2022 [26], RAVEN 2026 [27] | ✓ drift-aware | — | ✓ | varies | ✓ | map | ✓ | trajectory / map |
| Tankasala 2022 [28] | ✗ outdoor | — | ✓ minimum time, blur-limited speed | ✓ | ✗ | ODM | ✓ PX4 | ? |
| MP-SfM CVPR 2025 [29], DN-Splatter WACV 2025 [30] | indoor data, offline | ✓ few images (MP-SfM) | — | ✓ | mono depth / normal priors | SfM / 3DGS | — | ✓ benchmarks |

## Seven claims are taken

| Claim a room paper cannot make | Held by |
|---|---|
| "Viewpoint set + tour (TSP) + continuous capture for fast aerial reconstruction" | FC-Planner [14], PredRecon [15], SOAR [16]; Kinetix's own ADR-K12 says so |
| "GSD- / parallax-aware (reconstructability) view planning for photogrammetry" | Smith [17], Plan3D [18], Roberts [19], Liu [21] (outdoor, GNSS) |
| "Explore-then-exploit / two-stage capture of an unknown space" | Peng & Isler 2019 (related_work [10]), two-stage NBV [23], Gao [4] (indoor) |
| "First autonomous indoor drone that turns its images into a photogrammetric model scored against TLS" | ICON drone [1,2] |
| "Active 3DGS reconstruction of indoor scenes" (incl. a real UAV) | ActiveGS [9], ActiveSplat [10], HGS-Planner [11], GauSS-MI [12] |
| "Flight-time-budgeted indoor exploration for 3DGS" | Krisshnakumar et al. [8] (sim) |
| "Drift-aware (loop-closure-seeking) aerial exploration" | Schmid 2021 [25], Zhang 2022 [26], RAVEN [27] |

## Five gaps remain open

1. **Acquisition geometry vs accuracy, indoors, under VIO drift.** The reconstructability heuristics [17–21] were derived outdoors with
   GNSS poses, long ranges and textured facades. No study found measures how distance/GSD, parallax and overlap drive SfM accuracy in a
   room, where walls are textureless, ranges are short, the ceiling is far and the pose prior drifts. ICON [1,2] reports one acquisition
   per room, with no ablation. Kinetix data already shows this mechanism: flight `163032` triangulated 84 % of its points from more than 9 m away,
   with a median parallax of 7.5°, and reached only 33 % wall completeness (docs/acquisition/todo.md).
2. **The fewest images and shortest flight for a stated accuracy, camera-only, in real flight.** Fast-coverage planners [14–16] minimise
   time but need LiDAR or a known model. The time-budgeted indoor planner [8] is sim-only. Outdoor planners [17–21] minimise path, not
   images at an accuracy target. No work reports an images × flight-time × accuracy trade-off (a Pareto front) for an indoor
   drone without LiDAR or GNSS. Kinetix's N@q metric (C3) transfers directly to this setting.
3. **What stereo depth is for in room photogrammetry.** ICON uses RGB-D for navigation and *monocular* depth priors to fill weak texture
   [2]. ActiveGS and its relatives use RGB-D [9–12], and ActMVS uses monocular depth [13]. No controlled study keeps one room planner fixed and varies
   (a) depth for planning and (b) depth as reconstruction input (ZED depth in SfM / MVS / 3DGS) against mono. This is C2 moved from
   objects to rooms; it stays open at room scale.
4. **How VIO drift reaches photogrammetric accuracy, and whether planning can limit it.** Active SLAM work [25–27] plans revisits to
   keep the *map* or *trajectory* consistent. No paper found asks whether loop-closing revisits, or SfM pose priors with a known
   sigma, improve the *photogrammetric* result (ATE of the SfM cameras, cloud-to-mesh error) per image or per second flown.
5. **Speed vs blur indoors.** Blur-limited speed has been studied only outdoors, at survey altitudes [28]. Indoors (dim, short range, a rolling-shutter ZED)
   it is unquantified, and the Kinetix sim renders no motion blur (flight `163032`: Laplacian variance vs speed r = −0.02). So a sim result here
   is a lower bound until blur is simulated or flown for real.

## Recommended problem statement

Lead with gaps 1 + 2 and keep 3 as the sensing study:

- **Problem:** autonomous room photogrammetry with a camera-only drone (stereo RGB + ZED depth + VIO), at the fewest images and the shortest
  flight that reach a stated accuracy.
- **Method contribution (must survive a sweep):** an accuracy-targeted acquisition planner for rooms. It turns a target accuracy into a
  distance/GSD bound and a parallax requirement per surface, with drift in the cost through a pose-prior sigma and revisits.
  Exploration with ZED depth gives the room model.
- **Measurement contributions:** (i) how distance, parallax, overlap, speed and drift drive SfM accuracy in real rooms, against a mesh
  and a TLS reference; (ii) N@q and an images × flight-time × accuracy Pareto front against fixed patterns, exploration-only capture
  (the ICON style) and fast coverage (the FC-Planner style); (iii) mono vs stereo-depth for planning and for reconstruction.
- **Wording discipline:** do not claim the tour, explore-then-exploit, reconstructability or "first autonomous indoor photogrammetry
  drone". Every "first" carries "to our knowledge" and is re-checked at G2/G4/G5.

## Limitations

- The review is abstract-level, so some "?" cells may be filled in the full texts. **ICON [1,2], Gao [4], the Jin / ASIP papers [6,7] and the
  RTH planner [8] must be read in full before claims are fixed.** They are the closest prior work.
- Only English-language web search was used, with no structured database export. Chinese-language construction-robotics venues are likely
  under-sampled.
- Preprints from the last 3 months appear fast (RTH 2026-10-02, RAVEN 2026-10-27). Repeat the search before G2.
- The ISPRS motion-blur papers (2013–2015, cited by search results) were not verified, so they are not listed. Find and verify them before
  citing.
- AI-assisted: searches, screening and charting were done by Claude (Anthropic). The author must check the charted cells against the papers.

## References

Checked 2026-10-28: arXiv IDs against the arXiv API, DOIs against Crossref.

1. H.-X. Zhang, Y. Yang, Z. Zou, "ICON drone: Autonomous indoor exploration using unmanned aerial vehicle for semantic 3D reconstruction," *Proc. ACM BuildSys*, 2024. DOI: [10.1145/3671127.3698172](https://doi.org/10.1145/3671127.3698172)
2. H.-X. Zhang, Y. Yang, Z. Zou, "Autonomous unmanned aerial vehicles exploration for semantic indoor reconstruction using 3D Gaussian splatting," *Data-Centric Engineering*, vol. 6, e38, 2025. DOI: [10.1017/dce.2025.10017](https://doi.org/10.1017/dce.2025.10017)
3. X. Su, R. Chen, J. Ma, C. Li, J. E. Froehlich, "FlyMeThrough: Human-AI collaborative 3D indoor mapping with commodity drones," *Proc. ACM UIST*, 2025. arXiv: [2508.20034](https://arxiv.org/abs/2508.20034)
4. C.-X. Gao, X. Wang, R. Wang, Z. Zhao, et al., "A UAV-based explore-then-exploit system for autonomous indoor facility inspection and scene reconstruction," *Automation in Construction*, 2023. DOI: [10.1016/j.autcon.2023.104753](https://doi.org/10.1016/j.autcon.2023.104753)
5. P. Pan et al., "Developing smart MAVs for autonomous inspection in GPS-denied constructions," arXiv: [2408.06030](https://arxiv.org/abs/2408.06030), 2024.
6. H. Jin et al., "Adaptive planning framework for UAV-based surface inspection in partially unknown indoor environments," arXiv: [2504.09294](https://arxiv.org/abs/2504.09294), 2025.
7. H. Jin et al., "ASIP-Planner: Adaptive planning for UAV surface inspection in partially known indoor environments," *IROS* 2026 (accepted). arXiv: [2605.11119](https://arxiv.org/abs/2605.11119)
8. P. Krisshnakumar, F. Yang, K. Niinuma, "Return-to-home feasible micro-aerial vehicle exploration for 3D Gaussian splatting reconstruction," *IROS* 2026. arXiv: [2610.04013](https://arxiv.org/abs/2610.04013)
9. L. Jin, X. Zhong, Y. Pan, J. Behley, C. Stachniss, M. Popović, "ActiveGS: Active scene reconstruction using Gaussian splatting," *IEEE RA-L*, vol. 10, no. 5, pp. 4866–4873, 2025. DOI: [10.1109/LRA.2025.3555149](https://doi.org/10.1109/LRA.2025.3555149) · arXiv: [2412.17769](https://arxiv.org/abs/2412.17769)
10. Y. Li et al., "ActiveSplat: High-fidelity scene reconstruction through active Gaussian splatting," *IEEE RA-L*, 2025. DOI: [10.1109/LRA.2025.3580331](https://doi.org/10.1109/LRA.2025.3580331) · arXiv: [2410.21955](https://arxiv.org/abs/2410.21955)
11. Z. Xu et al., "HGS-Planner: Hierarchical planning framework for active scene reconstruction using 3D Gaussian splatting," *ICRA*, 2025. DOI: [10.1109/ICRA55743.2025.11127649](https://doi.org/10.1109/ICRA55743.2025.11127649) · arXiv: [2409.17624](https://arxiv.org/abs/2409.17624)
12. Y. Xie et al., "GauSS-MI: Gaussian splatting Shannon mutual information for active 3D reconstruction," *RSS*, 2025. DOI: [10.15607/RSS.2025.XXI.030](https://doi.org/10.15607/RSS.2025.XXI.030) · arXiv: [2504.21067](https://arxiv.org/abs/2504.21067)
13. G. Pu, Y. Han, Z. Lian, "ActMVS: Active scene reconstruction with monocular multi-view stereo," *ICRA*, 2026. DOI: [10.1109/ICRA57385.2026.11696006](https://doi.org/10.1109/ICRA57385.2026.11696006)
14. C. Feng et al., "FC-Planner: A skeleton-guided planning framework for fast aerial coverage of complex 3D scenes," *ICRA*, 2024. DOI: [10.1109/ICRA57147.2024.10610621](https://doi.org/10.1109/ICRA57147.2024.10610621) · arXiv: [2309.13882](https://arxiv.org/abs/2309.13882)
15. C. Feng et al., "PredRecon: A prediction-boosted planning framework for fast and high-quality autonomous aerial reconstruction," *ICRA*, 2023. DOI: [10.1109/ICRA48891.2023.10160933](https://doi.org/10.1109/ICRA48891.2023.10160933)
16. M. Zhang et al., "SOAR: Simultaneous exploration and photographing with heterogeneous UAVs for fast autonomous reconstruction," *IROS*, 2024. DOI: [10.1109/IROS58592.2024.10801474](https://doi.org/10.1109/IROS58592.2024.10801474)
17. N. Smith et al., "Aerial path planning for urban scene reconstruction: A continuous optimization method and benchmark," *ACM TOG*, vol. 37, no. 6, 2018. DOI: [10.1145/3272127.3275010](https://doi.org/10.1145/3272127.3275010)
18. B. Hepp, M. Nießner, O. Hilliges, "Plan3D: Viewpoint and trajectory optimization for aerial multi-view stereo reconstruction," *ACM TOG*, vol. 38, no. 1, 2018. DOI: [10.1145/3233794](https://doi.org/10.1145/3233794) · arXiv: [1705.09314](https://arxiv.org/abs/1705.09314)
19. M. Roberts et al., "Submodular trajectory optimization for aerial 3D scanning," *ICCV*, 2017. DOI: [10.1109/ICCV.2017.569](https://doi.org/10.1109/ICCV.2017.569)
20. X. Zhou et al., "Offsite aerial path planning for efficient urban scene reconstruction," *ACM TOG*, vol. 39, no. 6, 2020. DOI: [10.1145/3414685.3417791](https://doi.org/10.1145/3414685.3417791)
21. Y. Liu et al., "Learning reconstructability for drone aerial path planning," *ACM TOG*, vol. 41, no. 6, 2022. DOI: [10.1145/3550454.3555433](https://doi.org/10.1145/3550454.3555433)
22. L. Schmid et al., "An efficient sampling-based method for online informative path planning in unknown environments," *IEEE RA-L*, vol. 5, no. 2, 2020. DOI: [10.1109/LRA.2020.2969191](https://doi.org/10.1109/LRA.2020.2969191)
23. Z. Meng et al., "A two-stage optimized next-view planning framework for 3-D unknown environment exploration, and structural reconstruction," *IEEE RA-L*, vol. 2, no. 3, pp. 1680–1687, 2017. DOI: [10.1109/LRA.2017.2655144](https://doi.org/10.1109/LRA.2017.2655144)
24. B. Zhou et al., "FUEL: Fast UAV exploration using incremental frontier structure and hierarchical planning," *IEEE RA-L*, 2021. DOI: [10.1109/LRA.2021.3051563](https://doi.org/10.1109/LRA.2021.3051563)
25. L. Schmid et al., "A unified approach for autonomous volumetric exploration of large scale environments under severe odometry drift," *IEEE RA-L*, vol. 6, no. 3, 2021. DOI: [10.1109/LRA.2021.3068954](https://doi.org/10.1109/LRA.2021.3068954)
26. Y. Zhang et al., "Exploration with global consistency using real-time re-integration and active loop closure," *ICRA*, 2022. DOI: [10.1109/ICRA46639.2022.9811892](https://doi.org/10.1109/ICRA46639.2022.9811892)
27. X. Chen et al., "RAVEN: A unified active SLAM system for rapid autonomous visual exploration with aerial robots under localization uncertainty," *IEEE T-RO* (conditionally accepted). arXiv: [2610.37111](https://arxiv.org/abs/2610.37111), 2026.
28. S. Tankasala, C. Pehlivanturk, M. Pryor, "Minimum time trajectory generation for surveying using UAVs," arXiv: [2202.11297](https://arxiv.org/abs/2202.11297), 2022.
29. Z. Pataki et al., "MP-SfM: Monocular surface priors for robust structure-from-motion," *CVPR*, 2025. DOI: [10.1109/CVPR52734.2025.02039](https://doi.org/10.1109/CVPR52734.2025.02039)
30. M. Turkulainen et al., "DN-Splatter: Depth and normal priors for Gaussian splatting and meshing," *WACV*, 2025. DOI: [10.1109/WACV61041.2025.00241](https://doi.org/10.1109/WACV61041.2025.00241)
31. (sensor context) "Depth accuracy analysis of the ZED 2i stereo camera in an indoor environment," *Robot. Auton. Syst.*, 2024. DOI: [10.1016/j.robot.2024.104753](https://doi.org/10.1016/j.robot.2024.104753)
