# Related Work

## Traditional Photogrammetry

- **Structure-from-Motion (SfM)**: COLMAP [\[1\]](#ref1), Agisoft Metashape, RealityCapture. Rely on dense image sets from manual or grid-based paths.
- **Drone-based Systems**: Commercial solutions (DJI Terra, Pix4D) use pre-planned missions with high overlap (70–90%). Quality-driven autonomous re-capture for UAVs using multi-view-stereo confidence prediction [\[2\]](#ref2) represents an early online feedback loop, recently extended to full on-the-fly SfM with predictive path planning [\[3\]](#ref3).
- **Open-Source Pipelines**: OpenDroneMap (ODM/WebODM) [\[4\]](#ref4) is a widely used open-source photogrammetry pipeline covering SfM, MVS, meshing, and orthophoto generation, with accuracy recently benchmarked against commercial software [\[49\]](#ref49); it internally uses OpenSfM [\[5\]](#ref5) for the SfM stage. OpenSfM is a Python/C++ incremental SfM library with a modular design. Both assume fully passive, pre-planned image sets and have no adaptive replanning capability.

Limitations: Inefficient for indoor objects; do not adapt to scene geometry; no object-centric view selection.

## Coverage Path Planning (CPP)

Coverage path planning — finding a path that visits all points of an area or volume — is the dominant flight-planning paradigm for commercial UAV photogrammetry [\[6\]](#ref6). Understanding its strengths and limitations is essential for motivating Kinetix's adaptive approach.

- **Classic 2D Coverage (Boustrophedon / Lawnmower)**: Parallel back-and-forth sweeps over a polygon, implemented by DJI Terra, Pix4DCapture, and most commercial planners. A reformulation as a Generalized Traveling Salesman Problem [\[7\]](#ref7) minimizes transition costs between cells, achieving 14% lower path cost. These methods are efficient on flat terrain but are open-loop — they do not respond to reconstruction quality and generate far more images than necessary.
- **3D Coverage for Terrain & Structures**: CPP has been extended to 3D terrain [\[8\]](#ref8), generating UAV paths that minimize battery consumption while achieving photogrammetric coverage of complex surfaces, and to large structures such as buildings via multi-UAV CPP [\[9\]](#ref9). These approaches require a known 3D model as input — they are model-based and cannot handle unknown or partially known indoor objects.
- **Adaptive CPP**: An explore-then-exploit approach for aerial 3D reconstruction [\[10\]](#ref10) first builds a coarse proxy from an initial exploratory flight, then targets low-quality regions with subsequent exploitation flights. A separate line of work adapts UAV altitude and resolution online based on incoming semantic segmentation quality [\[11\]](#ref11). These works demonstrate that adaptive replanning substantially reduces flight time and image count.
- **Survey**: A comprehensive review of model-free and model-based viewpoint and path-planning algorithms for UAV-based 3D reconstruction [\[12\]](#ref12) covers both CPP and NBV paradigms and documents the shift toward adaptive, scene-aware methods.

Collectively, CPP methods represent the state of practice. Kinetix departs from CPP by replacing fixed-coverage geometry with uncertainty-driven NBV selection, requiring no pre-existing 3D model, and optimizing directly for reconstruction quality rather than area coverage.

## Active Vision and Next Best View (NBV)

- **Next Best View Planning**: Classic problem formalized in early work [\[13\]](#ref13) and surveyed comprehensively [\[14\]](#ref14).
- **Uncertainty-based NBV**: Methods using volumetric uncertainty or information gain, e.g., receding-horizon voxel exploration [\[15\]](#ref15), a foundational information-gain formulation [\[16\]](#ref16), and a rigorous comparison of information-gain metrics for object reconstruction [\[17\]](#ref17). These have been extended to object-level reconstruction with explicit positioning uncertainty [\[18\]](#ref18).
- **Learning-based View Planning**: A supervised 3D-CNN predicts the NBV from partial point clouds [\[19\]](#ref19), while reinforcement learning (Scan-RL) learns a view policy for drone-based scanning [\[20\]](#ref20). More recently, NeurAR [\[21\]](#ref21) and Neu-NBV [\[22\]](#ref22) integrate NBV selection with neural-rendering uncertainty estimation.
- **Frontier-Based Exploration**: Frontier-based exploration [\[23\]](#ref23) — navigating toward boundaries between known free space and unknown space — has become a standard baseline for autonomous exploration planners including [\[15\]](#ref15).

## Monocular 3D Reconstruction

- **Neural Radiance Fields (NeRF)** [\[24\]](#ref24) and accelerated variants such as Instant-NGP [\[25\]](#ref25) and 3D Gaussian Splatting [\[26\]](#ref26) achieve high quality but typically require dense, calibrated views.
- **Sparse-view methods**: pixelNeRF [\[27\]](#ref27) uses a learned scene prior for few-shot novel-view synthesis. SparseNeRF [\[28\]](#ref28) distills monocular depth-rank supervision to enable NeRF training from 3–5 views. Zero-1-to-3 [\[29\]](#ref29) demonstrates diffusion-based novel-view synthesis from a single image.
- **Monocular depth foundation models**: Depth Anything [\[30\]](#ref30) provides robust zero-shot monocular depth estimation trained on 60M+ images, offering a scalable depth prior for systems lacking metric sensors.

## Drone-Specific Adaptive Photogrammetry

- **Autonomous Exploration**: Environment-level exploration with UAVs using receding-horizon NBV [\[15\]](#ref15) and autonomous volumetric exploration of large-scale environments under severe odometry drift [\[31\]](#ref31) have both been demonstrated.
- **Object-centric Scanning**: Efficient next-best-scan planning for object surface reconstruction using robotic arms [\[32\]](#ref32) offers principles that are applicable but not directly transferable to drone platforms.
- **Prediction-Boosted Reconstruction**: PredRecon [\[33\]](#ref33) uses a surface prediction module to complete partial reconstructions on the fly, then plans hierarchical coverage paths to fill gaps; achieves high-quality reconstruction in a single flight over outdoor structures.
- **Heterogeneous Multi-UAV Reconstruction**: SOAR [\[34\]](#ref34) employs a LiDAR-equipped explorer and camera-equipped photographers cooperating via frontier-based scheduling; demonstrates simultaneous exploration and high-quality photographing for fast autonomous reconstruction.
- **Recent Works**:
  - Online SfM feedback with quality-aware replanning [\[3\]](#ref3).
  - Neural-uncertainty-driven NBV validated on real robots [\[21\]](#ref21), [\[22\]](#ref22).
  - Adaptive CPP with semantic-quality feedback [\[11\]](#ref11).
  - Monocular active scene reconstruction: ActMVS [\[35\]](#ref35) plans views for scene-level reconstruction from monocular multi-view stereo (view factor graph + global depth optimisation), evaluated in simulation, and reports performance competitive with RGB-D methods.
  - Foundation-model-guided structure scanning with drones: FlyCo [\[36\]](#ref36).

## Closest Recent Work (2024–2026)

Found by the novelty search ([reports/Kinetix novelty search.md](reports/Kinetix%20novelty%20search.md)). Each of these takes one of the claims Kinetix originally planned:

- **RGB-only object NBV on a real drone**: Hestia [\[37\]](#ref37) is a learned hierarchical NBV policy trained in Isaac Lab and deployed on a Crazyflie with an RGB camera and a learned depth predictor (MASt3R/DUSt3R). Real-world results are qualitative, localisation is external (Lighthouse), and reconstruction is MASt3R-SfM.
- **Foundation depth inside NBV/exploration**: Next Best Sense [\[38\]](#ref38) uses Depth Anything V2 / Metric3D V2 for object NBV on a robot arm, with scale aligned to a real depth camera. FrontierNet [\[39\]](#ref39) detects frontiers and information gain from posed RGB plus monocular metric depth on a Spot robot, and shows that classical planners degrade with predicted depth. Murre [\[40\]](#ref40) fuses SfM-guided monocular depth into a TSDF, but offline and with no view planning.
- **Monocular active reconstruction**: ActMVS [\[35\]](#ref35) (scenes, simulation only). Replacing its MVS depth with single-image depth raises Chamfer from ~2.4 cm to ~46 cm. MACARONS [\[41\]](#ref41) uses self-supervised RGB-only NBV.
- **Image savings already reported**: ~72 % fewer images for a monocular toy drone [\[42\]](#ref42), ~53 % at equal F-score for learned aerial planning [\[43\]](#ref43), 70 vs 150 views with farthest-view sampling [\[44\]](#ref44), and half the views in PUN [\[45\]](#ref45).
- **Benchmark protocols**: ObjView-Bench [\[46\]](#ref46) (128-view Tammes sphere, 5,691 Objaverse++ objects, budgets K=5/30, reachability on a real arm) argues that deployment constraints change method rankings, but for a tabletop arm with coverage metrics. MA-SCVP [\[47\]](#ref47) is the public learned object-NBV baseline of the same group. FisherRF [\[48\]](#ref48) selects views RGB-only from a candidate pool via 3DGS Fisher information.

Common pattern: every geometric evaluator above scores its *own* reconstruction (back-projected depth, online 3DGS, neural surface). None evaluates the selected image set through an unmodified COLMAP SfM→MVS pipeline against a GT mesh.

## Gaps This Work Addresses

- **Scale from the reconstruction itself**: prior foundation-depth NBV takes scale from a depth sensor [\[38\]](#ref38) or uses a metric-depth model directly [\[39\]](#ref39). Naive monocular depth is unreliable for planning [\[35\]](#ref35). Kinetix anchors relative depth to the online SfM model and combines it with SfM uncertainty, so planning needs no depth sensor and no metric-depth model (C1).
- **What depth is for**: no work keeps one NBV planner fixed and swaps monocular, small-baseline stereo and RGB-D input. Cross-paper comparisons mix planners and representations [\[35\]](#ref35). Kinetix separates depth for planning from depth as reconstruction input in a 2×2 design on one drone (C2).
- **Images-to-quality**: efficiency is reported as coverage AUC, coverage at a budget, or views to a PSNR level [\[44\]](#ref44). Kinetix formalises N@q against a dense capture of the same object, with geometric F-score (C3).
- **Realism tiers**: object NBV benchmarks are depth-rendered view spheres [\[46\]](#ref46) or simulators without flight dynamics. No protocol measures how rankings shift from photoreal replay to PX4 SITL to real indoor flight (C4).
- **Photogrammetry-compatible, quantitative real flights**: drone object NBV with real flights reports qualitative results [\[37\]](#ref37) or predates modern pipelines [\[42\]](#ref42). Kinetix reports Chamfer/F-score/N@q on real flights through unmodified COLMAP → MVS/3DGS (C5).
- **CPP vs NBV**: CPP methods [\[8\]](#ref8), [\[7\]](#ref7) plan globally but cannot react to local reconstruction quality. The image-count reduction over fixed patterns is reported as a result consistent with [\[42\]](#ref42)–[\[45\]](#ref45), measured against the strongest baseline.

## References

Cross-checked on 2026-10-08 against Crossref (publisher DOIs) and the arXiv API. Every entry has a DOI link except
[4] and [5], for which no DOI exists (FIG proceedings abstract; software repository). Papers published only on
arXiv use their arXiv DOI (`10.48550/arXiv.*`). Where an arXiv version exists it is linked as well.

<a id="ref1"></a>[1] J. L. Schönberger and J.-M. Frahm, "Structure-from-motion revisited," in *Proc. IEEE Conf. Comput. Vis. Pattern Recognit. (CVPR)*, 2016, pp. 4104–4113. DOI: [10.1109/CVPR.2016.445](https://doi.org/10.1109/CVPR.2016.445)

<a id="ref2"></a>[2] C. Mostegel, M. Rumpler, F. Fraundorfer, and H. Bischof, "UAV-based autonomous image acquisition with multi-view stereo quality assurance by confidence prediction," in *Proc. IEEE Conf. Comput. Vis. Pattern Recognit. Workshops (CVPRW)*, 2016, pp. 1–10. DOI: [10.1109/CVPRW.2016.8](https://doi.org/10.1109/CVPRW.2016.8) · arXiv: [1605.01923](https://arxiv.org/abs/1605.01923)

<a id="ref3"></a>[3] L. Lou, W. Li, W. Gan, Y. Yu, T. Wang, X. Wang, and Z. Zhan, "On-the-fly feedback structure from motion: Online explore-and-exploit unmanned aerial vehicle photogrammetry with incremental mesh quality-aware indicator and predictive path planning," *IEEE Geosci. Remote Sens. Mag.*, vol. 14, no. 4, pp. 219–239, 2026. DOI: [10.1109/MGRS.2026.3687205](https://doi.org/10.1109/MGRS.2026.3687205) · arXiv: [2512.02375](https://arxiv.org/abs/2512.02375)

<a id="ref4"></a>[4] G. Vacca, "WEB Open Drone Map (WebODM): A software open source for photogrammetry process," in *FIG Working Week 2020: Smart Surveyors for Land and Water Management*, Amsterdam, 2020. *No DOI exists (FIG proceedings).* [Online]. Available: [fig.net/.../TS01B_vacca_10728_abs.pdf](https://www.fig.net/resources/proceedings/fig_proceedings/fig2020/papers/ts01b/TS01B_vacca_10728_abs.pdf)

<a id="ref5"></a>[5] Mapillary, "OpenSfM: Open source structure-from-motion," software, 2014. *No DOI exists (software).* [Online]. Available: [github.com/mapillary/OpenSfM](https://github.com/mapillary/OpenSfM)

<a id="ref6"></a>[6] E. Galceran and M. Carreras, "A survey on coverage path planning for robotics," *Robot. Auton. Syst.*, vol. 61, no. 12, pp. 1258–1276, 2013. DOI: [10.1016/j.robot.2013.09.004](https://doi.org/10.1016/j.robot.2013.09.004)

<a id="ref7"></a>[7] R. Bähnemann, N. Lawrance, J. J. Chung, M. Pantic, R. Siegwart, and J. Nieto, "Revisiting boustrophedon coverage path planning as a generalized traveling salesman problem," in *Field and Service Robotics (FSR 2019)*, Springer Proc. Adv. Robot., 2021, pp. 277–290. DOI: [10.1007/978-981-15-9460-1_20](https://doi.org/10.1007/978-981-15-9460-1_20) · arXiv: [1907.09224](https://arxiv.org/abs/1907.09224)

<a id="ref8"></a>[8] M. Torres, D. A. Pelta, J. L. Verdegay, and J. C. Torres, "Coverage path planning with unmanned aerial vehicles for 3D terrain reconstruction," *Expert Syst. Appl.*, vol. 55, pp. 441–451, 2016. DOI: [10.1016/j.eswa.2016.02.007](https://doi.org/10.1016/j.eswa.2016.02.007)

<a id="ref9"></a>[9] W. Jing, D. Deng, Y. Wu, and K. Shimada, "Multi-UAV coverage path planning for the inspection of large and complex structures," in *Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst. (IROS)*, 2020, pp. 1480–1486. DOI: [10.1109/IROS45743.2020.9341089](https://doi.org/10.1109/IROS45743.2020.9341089) · arXiv: [2007.13065](https://arxiv.org/abs/2007.13065)

<a id="ref10"></a>[10] C. Peng and V. Isler, "Adaptive view planning for aerial 3D reconstruction," in *Proc. Int. Conf. Robot. Autom. (ICRA)*, 2019, pp. 2981–2987. DOI: [10.1109/ICRA.2019.8793532](https://doi.org/10.1109/ICRA.2019.8793532) · arXiv: [1805.00506](https://arxiv.org/abs/1805.00506)

<a id="ref11"></a>[11] F. Stache, J. Westheider, F. Magistri, C. Stachniss, and M. Popović, "Adaptive path planning for UAVs for multi-resolution semantic segmentation," *Robot. Auton. Syst.*, vol. 159, Art. 104288, 2023. DOI: [10.1016/j.robot.2022.104288](https://doi.org/10.1016/j.robot.2022.104288) · arXiv: [2203.01642](https://arxiv.org/abs/2203.01642)

<a id="ref12"></a>[12] M. Maboudi, M. Homaei, S. Song, S. Malihi, M. Saadatseresht, and M. Gerke, "A review on viewpoints and path planning for UAV-based 3-D reconstruction," *IEEE J. Sel. Topics Appl. Earth Observ. Remote Sens.*, vol. 16, pp. 5026–5048, 2023. DOI: [10.1109/JSTARS.2023.3276427](https://doi.org/10.1109/JSTARS.2023.3276427) · arXiv: [2205.03716](https://arxiv.org/abs/2205.03716)

<a id="ref13"></a>[13] C. Connolly, "The determination of next best views," in *Proc. IEEE Int. Conf. Robot. Autom. (ICRA)*, vol. 2, 1985, pp. 432–435. DOI: [10.1109/ROBOT.1985.1087372](https://doi.org/10.1109/ROBOT.1985.1087372)

<a id="ref14"></a>[14] W. R. Scott, G. Roth, and J.-F. Rivest, "View planning for automated three-dimensional object reconstruction and inspection," *ACM Comput. Surv.*, vol. 35, no. 1, pp. 64–96, 2003. DOI: [10.1145/641865.641868](https://doi.org/10.1145/641865.641868)

<a id="ref15"></a>[15] A. Bircher, M. Kamel, K. Alexis, H. Oleynikova, and R. Siegwart, "Receding horizon 'next-best-view' planner for 3D exploration," in *Proc. IEEE Int. Conf. Robot. Autom. (ICRA)*, 2016, pp. 1462–1468. DOI: [10.1109/ICRA.2016.7487281](https://doi.org/10.1109/ICRA.2016.7487281)

<a id="ref16"></a>[16] S. Isler, R. Sabzevari, J. Delmerico, and D. Scaramuzza, "An information gain formulation for active volumetric 3D reconstruction," in *Proc. IEEE Int. Conf. Robot. Autom. (ICRA)*, 2016, pp. 3477–3484. DOI: [10.1109/ICRA.2016.7487527](https://doi.org/10.1109/ICRA.2016.7487527)

<a id="ref17"></a>[17] J. Delmerico, S. Isler, R. Sabzevari, and D. Scaramuzza, "A comparison of volumetric information gain metrics for active 3D object reconstruction," *Auton. Robots*, vol. 42, no. 2, pp. 197–208, 2018. DOI: [10.1007/s10514-017-9634-0](https://doi.org/10.1007/s10514-017-9634-0)

<a id="ref18"></a>[18] J. I. Vasquez-Gomez, L. E. Sucar, and R. Murrieta-Cid, "View/state planning for three-dimensional object reconstruction under uncertainty," *Auton. Robots*, vol. 41, no. 1, pp. 89–109, 2017. DOI: [10.1007/s10514-015-9531-3](https://doi.org/10.1007/s10514-015-9531-3)

<a id="ref19"></a>[19] M. Mendoza, J. I. Vasquez-Gomez, H. Taud, L. E. Sucar, and C. Reta, "Supervised learning of the next-best-view for 3D object reconstruction," *Pattern Recognit. Lett.*, vol. 133, pp. 224–231, 2020. DOI: [10.1016/j.patrec.2020.02.024](https://doi.org/10.1016/j.patrec.2020.02.024) · arXiv: [1905.05833](https://arxiv.org/abs/1905.05833)

<a id="ref20"></a>[20] D. Peralta, J. Casimiro, A. M. Nilles, J. A. Aguilar, R. Atienza, and R. Cajote, "Next-best view policy for 3D reconstruction," in *Computer Vision – ECCV 2020 Workshops*, Lecture Notes in Computer Science, Springer, 2020, pp. 558–573. DOI: [10.1007/978-3-030-66823-5_33](https://doi.org/10.1007/978-3-030-66823-5_33) · arXiv: [2008.12664](https://arxiv.org/abs/2008.12664)

<a id="ref21"></a>[21] Y. Ran, J. Zeng, S. He, J. Chen, L. Li, Y. Chen, G. Lee, and Q. Ye, "NeurAR: Neural uncertainty for autonomous 3D reconstruction with implicit neural representations," *IEEE Robot. Autom. Lett.*, vol. 8, no. 2, pp. 1125–1132, 2023. DOI: [10.1109/LRA.2023.3235686](https://doi.org/10.1109/LRA.2023.3235686) · arXiv: [2207.10985](https://arxiv.org/abs/2207.10985)

<a id="ref22"></a>[22] L. Jin, X. Chen, J. Rückin, and M. Popović, "NeU-NBV: Next best view planning using uncertainty estimation in image-based neural rendering," in *Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst. (IROS)*, 2023, pp. 11305–11312. DOI: [10.1109/IROS55552.2023.10342226](https://doi.org/10.1109/IROS55552.2023.10342226) · arXiv: [2303.01284](https://arxiv.org/abs/2303.01284)

<a id="ref23"></a>[23] B. Yamauchi, "A frontier-based approach for autonomous exploration," in *Proc. IEEE Int. Symp. Comput. Intell. Robot. Autom. (CIRA)*, 1997, pp. 146–151. DOI: [10.1109/CIRA.1997.613851](https://doi.org/10.1109/CIRA.1997.613851)

<a id="ref24"></a>[24] B. Mildenhall, P. P. Srinivasan, M. Tancik, J. T. Barron, R. Ramamoorthi, and R. Ng, "NeRF: Representing scenes as neural radiance fields for view synthesis," in *Computer Vision – ECCV 2020*, Lecture Notes in Computer Science, Springer, 2020, pp. 405–421. DOI: [10.1007/978-3-030-58452-8_24](https://doi.org/10.1007/978-3-030-58452-8_24) · arXiv: [2003.08934](https://arxiv.org/abs/2003.08934)

<a id="ref25"></a>[25] T. Müller, A. Evans, C. Schied, and A. Keller, "Instant neural graphics primitives with a multiresolution hash encoding," *ACM Trans. Graph.*, vol. 41, no. 4, Art. 102, 2022. DOI: [10.1145/3528223.3530127](https://doi.org/10.1145/3528223.3530127)

<a id="ref26"></a>[26] B. Kerbl, G. Kopanas, T. Leimkühler, and G. Drettakis, "3D Gaussian splatting for real-time radiance field rendering," *ACM Trans. Graph.*, vol. 42, no. 4, Art. 139, 2023. DOI: [10.1145/3592433](https://doi.org/10.1145/3592433)

<a id="ref27"></a>[27] A. Yu, V. Ye, M. Tancik, and A. Kanazawa, "pixelNeRF: Neural radiance fields from one or few images," in *Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit. (CVPR)*, 2021, pp. 4576–4585. DOI: [10.1109/CVPR46437.2021.00455](https://doi.org/10.1109/CVPR46437.2021.00455)

<a id="ref28"></a>[28] G. Wang, Z. Chen, C. C. Loy, and Z. Liu, "SparseNeRF: Distilling depth ranking for few-shot novel view synthesis," in *Proc. IEEE/CVF Int. Conf. Comput. Vis. (ICCV)*, 2023, pp. 9031–9042. DOI: [10.1109/ICCV51070.2023.00832](https://doi.org/10.1109/ICCV51070.2023.00832) · arXiv: [2303.16196](https://arxiv.org/abs/2303.16196)

<a id="ref29"></a>[29] R. Liu, R. Wu, B. Van Hoorick, P. Tokmakov, S. Zakharov, and C. Vondrick, "Zero-1-to-3: Zero-shot one image to 3D object," in *Proc. IEEE/CVF Int. Conf. Comput. Vis. (ICCV)*, 2023, pp. 9264–9275. DOI: [10.1109/ICCV51070.2023.00853](https://doi.org/10.1109/ICCV51070.2023.00853) · arXiv: [2303.11328](https://arxiv.org/abs/2303.11328)

<a id="ref30"></a>[30] L. Yang, B. Kang, Z. Huang, X. Xu, J. Feng, and H. Zhao, "Depth anything: Unleashing the power of large-scale unlabeled data," in *Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit. (CVPR)*, 2024, pp. 10371–10381. DOI: [10.1109/CVPR52733.2024.00987](https://doi.org/10.1109/CVPR52733.2024.00987) · arXiv: [2401.10891](https://arxiv.org/abs/2401.10891)

<a id="ref31"></a>[31] L. Schmid, V. Reijgwart, L. Ott, J. Nieto, R. Siegwart, and C. Cadena, "A unified approach for autonomous volumetric exploration of large scale environments under severe odometry drift," *IEEE Robot. Autom. Lett.*, vol. 6, no. 3, pp. 4504–4511, 2021. DOI: [10.1109/LRA.2021.3068954](https://doi.org/10.1109/LRA.2021.3068954) · arXiv: [2010.09859](https://arxiv.org/abs/2010.09859)

<a id="ref32"></a>[32] S. Kriegel, C. Rink, T. Bodenmüller, and M. Suppa, "Efficient next-best-scan planning for autonomous 3D surface reconstruction of unknown objects," *J. Real-Time Image Process.*, vol. 10, no. 4, pp. 611–631, 2015. DOI: [10.1007/s11554-013-0386-6](https://doi.org/10.1007/s11554-013-0386-6)

<a id="ref33"></a>[33] C. Feng, H. Li, F. Gao, B. Zhou, and S. Shen, "PredRecon: A prediction-boosted planning framework for fast and high-quality autonomous aerial reconstruction," in *Proc. IEEE Int. Conf. Robot. Autom. (ICRA)*, 2023, pp. 1207–1213. DOI: [10.1109/ICRA48891.2023.10160933](https://doi.org/10.1109/ICRA48891.2023.10160933) · arXiv: [2302.04488](https://arxiv.org/abs/2302.04488)

<a id="ref34"></a>[34] M. Zhang, C. Feng, Z. Li, G. Zheng, Y. Luo, Z. Wang, J. Zhou, S. Shen, and B. Zhou, "SOAR: Simultaneous exploration and photographing with heterogeneous UAVs for fast autonomous reconstruction," in *Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst. (IROS)*, 2024, pp. 10975–10982. DOI: [10.1109/IROS58592.2024.10801474](https://doi.org/10.1109/IROS58592.2024.10801474) · arXiv: [2409.02738](https://arxiv.org/abs/2409.02738)

<a id="ref35"></a>[35] G. Pu, Y. Han, and Z. Lian, "ActMVS: Active scene reconstruction with monocular multi-view stereo," in *Proc. IEEE Int. Conf. Robot. Autom. (ICRA)*, 2026, pp. 18735–18742. DOI: [10.1109/ICRA57385.2026.11696006](https://doi.org/10.1109/ICRA57385.2026.11696006) · arXiv: [2606.01367](https://arxiv.org/abs/2606.01367)

<a id="ref36"></a>[36] C. Feng, G. Zheng, T. Zhuang, Y. Wu, F. He, H. Li, J. Zheng, S. Shen, and B. Zhou, "FlyCo: Foundation model-empowered drones for autonomous 3D structure scanning in open-world environments," *arXiv preprint* arXiv:2601.07558, 2026. DOI: [10.48550/arXiv.2601.07558](https://doi.org/10.48550/arXiv.2601.07558)

<a id="ref37"></a>[37] C.-Y. Lu, Z. Zhuang, N. T. T. Le, D. Xiao, Y.-C. Chang, T. Do, S. Sridhar, and C.-T. Lin, "Hestia: Voxel-face-aware hierarchical next-best-view acquisition for efficient 3D reconstruction," in *Proc. IEEE/CVF Winter Conf. Appl. Comput. Vis. (WACV)*, 2026, pp. 5302–5312. DOI: [10.1109/WACV61042.2026.00514](https://doi.org/10.1109/WACV61042.2026.00514) · arXiv: [2508.01014](https://arxiv.org/abs/2508.01014)

<a id="ref38"></a>[38] M. Strong, B. Lei, A. Swann, W. Jiang, K. Daniilidis, and M. Kennedy, "Next best sense: Guiding vision and touch with FisherRF for 3D Gaussian splatting," in *Proc. IEEE Int. Conf. Robot. Autom. (ICRA)*, 2025, pp. 3204–3210. DOI: [10.1109/ICRA55743.2025.11127233](https://doi.org/10.1109/ICRA55743.2025.11127233) · arXiv: [2410.04680](https://arxiv.org/abs/2410.04680)

<a id="ref39"></a>[39] B. Sun, H. Chen, S. Leutenegger, C. Cadena, M. Pollefeys, and H. Blum, "FrontierNet: Learning visual cues to explore," *IEEE Robot. Autom. Lett.*, vol. 10, no. 7, pp. 6576–6583, 2025. DOI: [10.1109/LRA.2025.3569122](https://doi.org/10.1109/LRA.2025.3569122) · arXiv: [2501.04597](https://arxiv.org/abs/2501.04597)

<a id="ref40"></a>[40] H. Guo, H. Zhu, S. Peng, H. Lin, Y. Yan, T. Xie, W. Wang, X. Zhou, and H. Bao, "Multi-view reconstruction via SfM-guided monocular depth estimation," in *Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit. (CVPR)*, 2025, pp. 5272–5282. DOI: [10.1109/CVPR52734.2025.00497](https://doi.org/10.1109/CVPR52734.2025.00497)

<a id="ref41"></a>[41] A. Guédon, T. Monnier, P. Monasse, and V. Lepetit, "MACARONS: Mapping and coverage anticipation with RGB online self-supervision," in *Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit. (CVPR)*, 2023, pp. 940–951. DOI: [10.1109/CVPR52729.2023.00097](https://doi.org/10.1109/CVPR52729.2023.00097) · arXiv: [2303.03315](https://arxiv.org/abs/2303.03315)

<a id="ref42"></a>[42] R. Huang, D. Zou, R. Vaughan, and P. Tan, "Active image-based modeling with a toy drone," in *Proc. IEEE Int. Conf. Robot. Autom. (ICRA)*, 2018. DOI: [10.1109/ICRA.2018.8460673](https://doi.org/10.1109/ICRA.2018.8460673) · arXiv: [1705.01010](https://arxiv.org/abs/1705.01010)

<a id="ref43"></a>[43] Y. Liu, L. Lin, Y. Hu, K. Xie, C.-W. Fu, H. Zhang, and H. Huang, "Learning reconstructability for drone aerial path planning," *ACM Trans. Graph.*, vol. 41, no. 6, Art. 197, 2022. DOI: [10.1145/3550454.3555433](https://doi.org/10.1145/3550454.3555433) · arXiv: [2209.10174](https://arxiv.org/abs/2209.10174)

<a id="ref44"></a>[44] W. Xiao, R. Santa Cruz, D. Ahmedt-Aristizabal, O. Salvado, C. Fookes, and L. Lebrat, "NeRF Director: Revisiting view selection in neural volume rendering," in *Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit. (CVPR)*, 2024, pp. 20742–20751. DOI: [10.1109/CVPR52733.2024.01960](https://doi.org/10.1109/CVPR52733.2024.01960) · arXiv: [2406.08839](https://arxiv.org/abs/2406.08839)

<a id="ref45"></a>[45] Z. Zhang, F. Xu, and M. Zhang, "Peering into the unknown: Active view selection with neural uncertainty maps for 3D reconstruction," in *Proc. Int. Conf. Learn. Represent. (ICLR)*, 2026. DOI (arXiv version; ICLR does not assign DOIs): [10.48550/arXiv.2506.14856](https://doi.org/10.48550/arXiv.2506.14856) · [ICLR 2026 poster page](https://iclr.cc/virtual/2026/poster/10008352)

<a id="ref46"></a>[46] S. Pan, H. Hu, X. Huang, B. Wingender, and M. Bennewitz, "ObjView-Bench: Rethinking difficulty and deployment for object-centric view planning," *arXiv preprint* arXiv:2605.10707, 2026. DOI: [10.48550/arXiv.2605.10707](https://doi.org/10.48550/arXiv.2605.10707)

<a id="ref47"></a>[47] S. Pan, H. Hu, H. Wei, N. Dengler, T. Zaenker, M. Dawood, and M. Bennewitz, "Integrating one-shot view planning with a single next-best view via long-tail multiview sampling," *IEEE Trans. Robot.*, vol. 41, pp. 394–414, 2025. DOI: [10.1109/TRO.2024.3507993](https://doi.org/10.1109/TRO.2024.3507993) · Code (MA-SCVP): [github.com/psc0628/MA-SCVP](https://github.com/psc0628/MA-SCVP)

<a id="ref48"></a>[48] W. Jiang, B. Lei, and K. Daniilidis, "FisherRF: Active view selection and mapping with radiance fields using Fisher information," in *Computer Vision – ECCV 2024*, Lecture Notes in Computer Science, Springer, 2025, pp. 422–440. DOI: [10.1007/978-3-031-72624-8_24](https://doi.org/10.1007/978-3-031-72624-8_24) · arXiv: [2311.17874](https://arxiv.org/abs/2311.17874)

<a id="ref49"></a>[49] A. Kostrzewa, A. Płatek-Żak, P. Banat, and Ł. Wilk, "Open-source vs. commercial photogrammetry: Comparing accuracy and efficiency of OpenDroneMap and Agisoft Metashape," *Int. Arch. Photogramm. Remote Sens. Spatial Inf. Sci.*, vol. XLVIII-1/W4-2025, pp. 65–72, 2025. DOI: [10.5194/isprs-archives-XLVIII-1-W4-2025-65-2025](https://doi.org/10.5194/isprs-archives-XLVIII-1-W4-2025-65-2025)
