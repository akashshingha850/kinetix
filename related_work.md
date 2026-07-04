# Related Work

## Traditional Photogrammetry

- **Structure-from-Motion (SfM)**: COLMAP [\[1\]](#ref1), Agisoft Metashape, RealityCapture. Rely on dense image sets from manual or grid-based paths.
- **Drone-based Systems**: Commercial solutions (DJI Terra, Pix4D) use pre-planned missions with high overlap (70–90%). Quality-driven autonomous re-capture for UAVs using multi-view-stereo confidence prediction [\[2\]](#ref2) represents an early online feedback loop, recently extended to full on-the-fly SfM with predictive path planning [\[3\]](#ref3).
- **Open-Source Pipelines**: OpenDroneMap (ODM/WebODM) [\[4\]](#ref4) is a widely used open-source photogrammetry pipeline covering SfM, MVS, meshing, and orthophoto generation; it internally uses OpenSfM [\[5\]](#ref5) for the SfM stage. OpenSfM is a Python/C++ incremental SfM library with a modular design. Both assume fully passive, pre-planned image sets and have no adaptive replanning capability.

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

## Gaps This Work Addresses

- **Monocular-only active reconstruction indoors**: All confirmed active reconstruction systems with real-world validation use metric depth (RGB-D, LiDAR, or ToF) or stereo cameras, and assume depth input [\[15\]](#ref15), [\[17\]](#ref17), [\[21\]](#ref21), [\[34\]](#ref34). Kinetix's monocular-only constraint for indoor drones has no direct prior.
- **Object-centric drone scanning vs. environment mapping**: Environment exploration [\[15\]](#ref15), [\[31\]](#ref31) and object scanning with arms [\[32\]](#ref32) are distinct communities. PredRecon [\[33\]](#ref33) and SOAR [\[34\]](#ref34) target large outdoor structures. Drone-based object-level reconstruction indoors remains underexplored.
- **CPP vs. NBV efficiency gap**: CPP methods [\[8\]](#ref8), [\[7\]](#ref7) plan globally but cannot react to local reconstruction quality. NBV methods [\[17\]](#ref17), [\[21\]](#ref21) react adaptively but typically require depth sensors. Kinetix closes this gap with monocular-only adaptive NBV.
- **COLMAP + Gaussian Splatting pipeline compatibility**: Neural-uncertainty NBV approaches [\[21\]](#ref21), [\[22\]](#ref22) reconstruct inside NeRF pipelines. None maintain compatibility with the standard COLMAP → 3DGS workflow needed for archival photogrammetric use (museums, e-commerce, archaeology).
- **Quantified efficiency-quality trade-off**: Existing NBV and CPP papers report coverage percentage or rendering quality (PSNR/SSIM) but rarely tie results to photogrammetric completeness metrics (Chamfer distance, F-score). Kinetix provides this bridge.

Kinetix uniquely bridges adaptive NBV, monocular drone imaging, indoor object-centric constraints, and photogrammetric pipeline compatibility.

## References

<a id="ref1"></a>[1] J. L. Schönberger and J.-M. Frahm, "Structure-from-motion revisited," in *Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit. (CVPR)*, 2016, pp. 4104–4113. DOI: [10.1109/CVPR.2016.445](https://doi.org/10.1109/CVPR.2016.445)

<a id="ref2"></a>[2] C. Mostegel, M. Rumpler, F. Fraundorfer, and H. Bischof, "UAV-based autonomous image acquisition with multi-view stereo quality assurance by confidence prediction," in *Proc. IEEE Conf. Comput. Vis. Pattern Recognit. Workshops (CVPRW)*, 2016. arXiv: [1605.01923](https://arxiv.org/abs/1605.01923)

<a id="ref3"></a>[3] J. Pan, H. Wang, L. Yang, and F. Xu, "On-the-fly feedback SfM: Online explore-and-exploit UAV photogrammetry with incremental mesh quality-aware indicator and predictive path planning," *arXiv preprint*, 2024. arXiv: [2512.02375](https://arxiv.org/abs/2512.02375)

<a id="ref4"></a>[4] G. Vacca, "WEB Open Drone Map (WebODM): A software open source for photogrammetry process," in *FIG Working Week 2020: Smart Surveyors for Land and Water Management*, Amsterdam, 2020. [Online]. Available: [fig.net/.../TS01B_vacca_10728_abs.pdf](https://www.fig.net/resources/proceedings/fig_proceedings/fig2020/papers/ts01b/TS01B_vacca_10728_abs.pdf)

<a id="ref5"></a>[5] Mapillary, "OpenSfM: Open source structure-from-motion," GitHub, 2014. [Online]. Available: [github.com/mapillary/OpenSfM](https://github.com/mapillary/OpenSfM)

<a id="ref6"></a>[6] E. Galceran and M. Carreras, "A survey on coverage path planning for robotics," *Robot. Auton. Syst.*, vol. 61, no. 12, pp. 1258–1276, 2013. DOI: [10.1016/j.robot.2013.09.004](https://doi.org/10.1016/j.robot.2013.09.004)

<a id="ref7"></a>[7] R. Bähnemann, N. Lawrance, J. J. Chung, M. Pantic, R. Siegwart, and J. Nieto, "Revisiting boustrophedon coverage path planning as a generalized traveling salesman problem," in *Proc. Int. Symp. Field and Service Robotics (FSR)*, 2019. DOI: [10.1007/978-981-15-9460-1_20](https://doi.org/10.1007/978-981-15-9460-1_20) · arXiv: [1907.09224](https://arxiv.org/abs/1907.09224)

<a id="ref8"></a>[8] M. Torres, D. A. Pelta, J. L. Verdegay, and J. C. Torres, "Coverage path planning with unmanned aerial vehicles for 3D terrain reconstruction," *Expert Syst. Appl.*, vol. 55, pp. 441–451, 2016. DOI: [10.1016/j.eswa.2016.02.007](https://doi.org/10.1016/j.eswa.2016.02.007)

<a id="ref9"></a>[9] W. Jing, D. Deng, Y. Wu, and K. Shimada, "Multi-UAV coverage path planning for the inspection of large and complex structures," in *Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst. (IROS)*, 2020, pp. 1480–1486. DOI: [10.1109/IROS45743.2020.9341089](https://doi.org/10.1109/IROS45743.2020.9341089) · arXiv: [2007.13065](https://arxiv.org/abs/2007.13065)

<a id="ref10"></a>[10] C. Peng and V. Isler, "Adaptive view planning for aerial 3D reconstruction," in *Proc. IEEE Int. Conf. Robot. Autom. (ICRA)*, 2019, pp. 2981–2987. arXiv: [1805.00506](https://arxiv.org/abs/1805.00506)

<a id="ref11"></a>[11] F. Stache, J. Westheider, F. Magistri, M. Popović, and C. Stachniss, "Adaptive path planning for UAVs for multi-resolution semantic segmentation," *Robot. Auton. Syst.*, vol. 157, p. 104288, 2022. DOI: [10.1016/j.robot.2022.104288](https://doi.org/10.1016/j.robot.2022.104288) · arXiv: [2203.01642](https://arxiv.org/abs/2203.01642)

<a id="ref12"></a>[12] M. Maboudi, M. R. Homaei, S. Song, S. Malihi, M. Saadatseresht, and M. Gerke, "A review on viewpoints and path planning for UAV-based 3D reconstruction," *IEEE J. Sel. Topics Appl. Earth Observ. Remote Sens.*, vol. 16, pp. 5026–5048, 2023. DOI: [10.1109/JSTARS.2023.3276427](https://doi.org/10.1109/JSTARS.2023.3276427) · arXiv: [2205.03716](https://arxiv.org/abs/2205.03716)

<a id="ref13"></a>[13] C. Connolly, "The determination of next best views," in *Proc. IEEE Int. Conf. Robot. Autom. (ICRA)*, vol. 2, 1985, pp. 432–435. DOI: [10.1109/ROBOT.1985.1087372](https://doi.org/10.1109/ROBOT.1985.1087372)

<a id="ref14"></a>[14] W. R. Scott, G. Roth, and J.-F. Rivest, "View planning for automated three-dimensional object reconstruction and inspection," *ACM Comput. Surv.*, vol. 35, no. 1, pp. 64–96, 2003. DOI: [10.1145/641865.641868](https://doi.org/10.1145/641865.641868)

<a id="ref15"></a>[15] A. Bircher, M. Kamel, K. Alexis, H. Oleynikova, and R. Siegwart, "Receding horizon 'next-best-view' planner for 3D exploration," in *Proc. IEEE Int. Conf. Robot. Autom. (ICRA)*, 2016, pp. 1462–1468. DOI: [10.1109/ICRA.2016.7487281](https://doi.org/10.1109/ICRA.2016.7487281)

<a id="ref16"></a>[16] S. Isler, R. Sabzevari, J. Delmerico, and D. Scaramuzza, "An information gain formulation for active volumetric 3D reconstruction," in *Proc. IEEE Int. Conf. Robot. Autom. (ICRA)*, 2016. DOI: [10.1109/ICRA.2016.7487527](https://doi.org/10.1109/ICRA.2016.7487527)

<a id="ref17"></a>[17] J. A. Delmerico, S. Isler, R. Sabzevari, and D. Scaramuzza, "A comparison of volumetric information gain metrics for active 3D object reconstruction," *Auton. Robots*, vol. 42, no. 2, pp. 197–208, 2018. DOI: [10.1007/s10514-017-9634-0](https://doi.org/10.1007/s10514-017-9634-0)

<a id="ref18"></a>[18] J. I. Vasquez-Gomez, L. E. Sucar, and R. Murrieta-Cid, "View/state planning for three-dimensional object reconstruction under uncertainty," *Auton. Robots*, vol. 41, no. 1, pp. 89–109, 2017. DOI: [10.1007/s10514-015-9531-3](https://doi.org/10.1007/s10514-015-9531-3)

<a id="ref19"></a>[19] M. Mendoza, J. I. Vasquez-Gomez, H. Taud, L. E. Sucar, and C. Reta, "Supervised learning of the next-best-view for 3D object reconstruction," *Pattern Recognit. Lett.*, vol. 133, pp. 224–231, 2020. DOI: [10.1016/j.patrec.2020.02.024](https://doi.org/10.1016/j.patrec.2020.02.024) · arXiv: [1905.05833](https://arxiv.org/abs/1905.05833)

<a id="ref20"></a>[20] D. Peralta, J. Casimiro, A. M. Nilles, J. A. Aguilar, R. Atienza, and R. B. Cajipe, "Next-best view policy for 3D reconstruction," in *Proc. ECCV Workshop on Embodied Vision*, 2020. arXiv: [2008.12664](https://arxiv.org/abs/2008.12664)

<a id="ref21"></a>[21] Y. Ran, J. Zeng, S. He, J. Chen, L. Li, and Q. Ye, "NeurAR: Neural uncertainty for autonomous 3D reconstruction with implicit neural representations," *IEEE Robot. Autom. Lett.*, vol. 8, no. 2, pp. 1125–1132, 2023. DOI: [10.1109/LRA.2023.3235686](https://doi.org/10.1109/LRA.2023.3235686) · arXiv: [2207.10985](https://arxiv.org/abs/2207.10985)

<a id="ref22"></a>[22] L. Jin, X. Chen, J. Rückin, and M. Popović, "NeU-NBV: Next best view planning using uncertainty estimation in image-based neural rendering," in *Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst. (IROS)*, 2023, pp. 11305–11312. DOI: [10.1109/IROS55552.2023.10342226](https://doi.org/10.1109/IROS55552.2023.10342226) · arXiv: [2303.01284](https://arxiv.org/abs/2303.01284)

<a id="ref23"></a>[23] B. Yamauchi, "A frontier-based approach for autonomous exploration," in *Proc. IEEE Int. Symp. Comput. Intell. Robot. Autom. (CIRA)*, 1997, pp. 146–151. DOI: [10.1109/CIRA.1997.613851](https://doi.org/10.1109/CIRA.1997.613851)

<a id="ref24"></a>[24] B. Mildenhall, P. P. Srinivasan, M. Tancik, J. T. Barron, R. Ramamoorthi, and R. Ng, "NeRF: Representing scenes as neural radiance fields for view synthesis," in *Proc. Eur. Conf. Comput. Vis. (ECCV)*, 2020. DOI: [10.1007/978-3-030-58452-8_24](https://doi.org/10.1007/978-3-030-58452-8_24) · arXiv: [2003.08934](https://arxiv.org/abs/2003.08934)

<a id="ref25"></a>[25] T. Müller, A. Evans, C. Schied, and A. Keller, "Instant neural graphics primitives with a multiresolution hash encoding," *ACM Trans. Graph. (SIGGRAPH)*, vol. 41, no. 4, Art. 102, 2022. DOI: [10.1145/3528223.3530127](https://doi.org/10.1145/3528223.3530127)

<a id="ref26"></a>[26] B. Kerbl, G. Kopanas, T. Leimkühler, and G. Drettakis, "3D Gaussian splatting for real-time radiance field rendering," *ACM Trans. Graph. (SIGGRAPH)*, vol. 42, no. 4, Art. 139, 2023. DOI: [10.1145/3592433](https://doi.org/10.1145/3592433)

<a id="ref27"></a>[27] A. Yu, V. Ye, M. Tancik, and A. Kanazawa, "pixelNeRF: Neural radiance fields from one or few images," in *Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit. (CVPR)*, 2021, pp. 4578–4587. DOI: [10.1109/CVPR46437.2021.00455](https://doi.org/10.1109/CVPR46437.2021.00455)

<a id="ref28"></a>[28] G. Wang, Z. Chen, C. C. Loy, and Z. Liu, "SparseNeRF: Distilling depth ranking for few-shot novel view synthesis," in *Proc. IEEE/CVF Int. Conf. Comput. Vis. (ICCV)*, 2023, pp. 9065–9076. arXiv: [2303.16196](https://arxiv.org/abs/2303.16196)

<a id="ref29"></a>[29] R. Liu, R. Wu, B. Van Hoorick, P. Tokmakov, S. Zakharov, and C. Vondrick, "Zero-1-to-3: Zero-shot one image to 3D object," in *Proc. IEEE/CVF Int. Conf. Comput. Vis. (ICCV)*, 2023, pp. 9264–9275. DOI: [10.1109/ICCV51070.2023.00853](https://doi.org/10.1109/ICCV51070.2023.00853) · arXiv: [2303.11328](https://arxiv.org/abs/2303.11328)

<a id="ref30"></a>[30] L. Yang, B. Kang, Z. Huang, X. Xu, J. Feng, and H. Zhao, "Depth anything: Unleashing the power of large-scale unlabeled data," in *Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit. (CVPR)*, 2024, pp. 10371–10381. arXiv: [2401.10891](https://arxiv.org/abs/2401.10891)

<a id="ref31"></a>[31] L. Schmid, V. Reijgwart, L. Ott, J. Nieto, R. Siegwart, and C. Cadena, "A unified approach for autonomous volumetric exploration of large-scale environments under severe odometry drift," *IEEE Robot. Autom. Lett.*, vol. 6, no. 3, pp. 4504–4511, 2021. DOI: [10.1109/LRA.2021.3068954](https://doi.org/10.1109/LRA.2021.3068954) · arXiv: [2010.09859](https://arxiv.org/abs/2010.09859)

<a id="ref32"></a>[32] S. Kriegel, C. Rink, T. Bodenmüller, and M. Suppa, "Efficient next-best-scan planning for autonomous 3D surface reconstruction of unknown objects," *J. Real-Time Image Process.*, vol. 10, no. 4, pp. 611–631, 2015. DOI: [10.1007/s11554-013-0386-6](https://doi.org/10.1007/s11554-013-0386-6)

<a id="ref33"></a>[33] C. Feng, H. Li, F. Gao, B. Zhou, and S. Shen, "PredRecon: A prediction-boosted planning framework for fast and high-quality autonomous aerial reconstruction," in *Proc. IEEE Int. Conf. Robot. Autom. (ICRA)*, 2023. arXiv: [2302.04488](https://arxiv.org/abs/2302.04488)

<a id="ref34"></a>[34] X. Liu, Y. Wang, C. Feng, B. Zhou, and S. Shen, "SOAR: Simultaneous exploration and photographing with heterogeneous UAVs for fast autonomous reconstruction," in *Proc. IEEE Int. Conf. Robot. Autom. (ICRA)*, 2024. arXiv: [2409.02738](https://arxiv.org/abs/2409.02738)

---

*Citation note: In-text citations are clickable and link to the matching entry in the reference list; every reference carries a resolvable DOI or arXiv link for verification. All references were checked against IEEE Xplore, ACM DL, CVF Open Access, arXiv, Springer, and Semantic Scholar. Two author lists were corrected during verification: reference [22] (NeU-NBV — corrected to Jin, Chen, Rückin, Popović) and reference [31] (Schmid — corrected to Schmid, Reijgwart, Ott, Nieto, Siegwart, Cadena). Reference [5] (OpenSfM) has no formal peer-reviewed paper and is cited as a software repository per IEEE convention. Reference [34] (SOAR) author list should be re-confirmed against the final camera-ready version before submission.*
