# Related Work

## Traditional Photogrammetry

- **Structure-from-Motion (SfM)**: COLMAP [1], Agisoft Metashape, RealityCapture. Rely on dense image sets from manual or grid-based paths.
- **Drone-based Systems**: Commercial solutions (DJI Terra, Pix4D) use pre-planned missions with high overlap (70–90%). Mostegel et al. [2] propose quality-driven autonomous re-capture for UAVs using multi-view-stereo confidence prediction, representing an early online feedback loop. Pan et al. [3] recently extend this to full on-the-fly SfM with predictive path planning.
- **Open-Source Pipelines**: OpenDroneMap (ODM/WebODM) [4] is a widely used open-source photogrammetry pipeline covering SfM, MVS, meshing, and orthophoto generation; it internally uses OpenSfM [5] for the SfM stage. OpenSfM is a Python/C++ incremental SfM library with a modular design. Both assume fully passive, pre-planned image sets and have no adaptive replanning capability.

Limitations: Inefficient for indoor objects; do not adapt to scene geometry; no object-centric view selection.

## Coverage Path Planning (CPP)

Coverage path planning — finding a path that visits all points of an area or volume — is the dominant flight-planning paradigm for commercial UAV photogrammetry [6]. Understanding its strengths and limitations is essential for motivating Kinetix's adaptive approach.

- **Classic 2D Coverage (Boustrophedon / Lawnmower)**: Parallel back-and-forth sweeps over a polygon, implemented by DJI Terra, Pix4DCapture, and most commercial planners. Bähnemann et al. [7] reformulate this as a Generalized Traveling Salesman Problem to minimize transition costs between cells, achieving 14% lower path cost. These methods are efficient on flat terrain but are open-loop — they do not respond to reconstruction quality and generate far more images than necessary.
- **3D Coverage for Terrain & Structures**: Torres et al. [8] extend CPP to 3D terrain, generating UAV paths that minimize battery consumption while achieving photogrammetric coverage of complex surfaces. Jing et al. [9] address large structures (buildings) with multi-UAV CPP. These approaches require a known 3D model as input — they are model-based and cannot handle unknown or partially known indoor objects.
- **Adaptive CPP**: Peng and Isler [10] propose an explore-then-exploit approach for aerial 3D reconstruction: an initial exploratory flight builds a coarse proxy, and subsequent exploitation flights target low-quality regions. Stache et al. [11] adapt UAV altitude and resolution online based on incoming semantic segmentation quality. These works demonstrate that adaptive replanning substantially reduces flight time and image count.
- **Survey**: Maboudi et al. [12] provide a comprehensive review of model-free and model-based viewpoint and path-planning algorithms for UAV-based 3D reconstruction, covering both CPP and NBV paradigms and documenting the shift toward adaptive, scene-aware methods.

Collectively, CPP methods represent the state of practice. Kinetix departs from CPP by replacing fixed-coverage geometry with uncertainty-driven NBV selection, requiring no pre-existing 3D model, and optimizing directly for reconstruction quality rather than area coverage.

## Active Vision and Next Best View (NBV)

- **Next Best View Planning**: Classic problem formalized by Connolly [13] and surveyed comprehensively by Scott et al. [14].
- **Uncertainty-based NBV**: Methods using volumetric uncertainty or information gain, e.g., receding-horizon voxel exploration [15], a foundational information-gain formulation [16], and a rigorous comparison of information-gain metrics for object reconstruction [17]. Vasquez-Gomez et al. [18] extend these to object-level reconstruction with explicit positioning uncertainty.
- **Learning-based View Planning**: Mendoza and Vasquez-Gomez [19] use a supervised 3D-CNN to predict the NBV from partial point clouds. Peralta et al. [20] apply reinforcement learning (Scan-RL) to learn a view policy for drone-based scanning. More recently, NeurAR [21] and Neu-NBV [22] integrate NBV selection with neural-rendering uncertainty estimation.
- **Frontier-Based Exploration**: Yamauchi [23] introduces frontier-based exploration — navigating toward boundaries between known free space and unknown space — which has become a standard baseline for autonomous exploration planners including [15].

## Monocular 3D Reconstruction

- **Neural Radiance Fields (NeRF)** [24] and accelerated variants such as Instant-NGP [25] and 3D Gaussian Splatting [26] achieve high quality but typically require dense, calibrated views.
- **Sparse-view methods**: pixelNeRF [27] uses a learned scene prior for few-shot novel-view synthesis. SparseNeRF [28] distills monocular depth-rank supervision to enable NeRF training from 3–5 views. Zero-1-to-3 [29] demonstrates diffusion-based novel-view synthesis from a single image.
- **Monocular depth foundation models**: Depth Anything [30] provides robust zero-shot monocular depth estimation trained on 60M+ images, offering a scalable depth prior for systems lacking metric sensors.

## Drone-Specific Adaptive Photogrammetry

- **Autonomous Exploration**: Bircher et al. [15] demonstrate environment-level exploration with UAVs using receding-horizon NBV. Schmid et al. [31] address autonomous volumetric exploration of large-scale environments under severe odometry drift.
- **Object-centric Scanning**: Kriegel [32] proposes efficient next-best-scan planning for object surface reconstruction using robotic arms — principles applicable but not directly transferable to drone platforms.
- **Prediction-Boosted Reconstruction**: PredRecon [33] uses a surface prediction module to complete partial reconstructions on the fly, then plans hierarchical coverage paths to fill gaps; achieves high-quality reconstruction in a single flight over outdoor structures.
- **Heterogeneous Multi-UAV Reconstruction**: SOAR [34] employs a LiDAR-equipped explorer and camera-equipped photographers cooperating via frontier-based scheduling; demonstrates simultaneous exploration and high-quality photographing for fast autonomous reconstruction.
- **Recent Works**:
  - Online SfM feedback with quality-aware replanning [3].
  - Neural-uncertainty-driven NBV validated on real robots [21], [22].
  - Adaptive CPP with semantic-quality feedback [11].

## Gaps This Work Addresses

- **Monocular-only active reconstruction indoors**: All confirmed active reconstruction systems with real-world validation use metric depth (RGB-D, LiDAR, or ToF) or stereo cameras. Bircher et al. [15], Delmerico et al. [17], NeurAR [21], and SOAR [34] all assume depth input. Kinetix's monocular-only constraint for indoor drones has no direct prior.
- **Object-centric drone scanning vs. environment mapping**: Environment exploration [15], [31] and object scanning with arms [32] are distinct communities. PredRecon [33] and SOAR [34] target large outdoor structures. Drone-based object-level reconstruction indoors remains underexplored.
- **CPP vs. NBV efficiency gap**: CPP methods [8], [7] plan globally but cannot react to local reconstruction quality. NBV methods [17], [21] react adaptively but typically require depth sensors. Kinetix closes this gap with monocular-only adaptive NBV.
- **COLMAP + Gaussian Splatting pipeline compatibility**: Neural-uncertainty NBV approaches [21], [22] reconstruct inside NeRF pipelines. None maintain compatibility with the standard COLMAP → 3DGS workflow needed for archival photogrammetric use (museums, e-commerce, archaeology).
- **Quantified efficiency-quality trade-off**: Existing NBV and CPP papers report coverage percentage or rendering quality (PSNR/SSIM) but rarely tie results to photogrammetric completeness metrics (Chamfer distance, F-score). Kinetix provides this bridge.

Kinetix uniquely bridges adaptive NBV, monocular drone imaging, indoor object-centric constraints, and photogrammetric pipeline compatibility.

## References

[1] J. L. Schönberger and J.-M. Frahm, "Structure-from-motion revisited," in *Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit. (CVPR)*, 2016, pp. 4104–4113.

[2] C. Mostegel, M. Rumpler, F. Fraundorfer, and H. Bischof, "UAV-based autonomous image acquisition with multi-view stereo quality assurance by confidence prediction," in *Proc. IEEE Conf. Comput. Vis. Pattern Recognit. Workshops (CVPRW)*, 2016.

[3] J. Pan, H. Wang, L. Yang, and F. Xu, "On-the-fly feedback SfM: Online explore-and-exploit UAV photogrammetry with incremental mesh quality-aware indicator and predictive path planning," *arXiv:2512.02375*, 2024.

[4] G. Vacca, "WEB Open Drone Map (WebODM): A software open source for photogrammetry process," in *FIG Working Week 2020: Smart Surveyors for Land and Water Management*, Amsterdam, 2020.

[5] Mapillary, "OpenSfM: Open source structure-from-motion," GitHub, 2014. [Online]. Available: https://github.com/mapillary/OpenSfM

[6] E. Galceran and M. Carreras, "A survey on coverage path planning for robotics," *Robot. Auton. Syst.*, vol. 61, no. 12, pp. 1258–1276, 2013.

[7] R. Bähnemann, N. Lawrance, J. J. Chung, M. Pantic, R. Siegwart, and J. Nieto, "Revisiting boustrophedon coverage path planning as a generalized traveling salesman problem," in *Proc. Int. Symp. Field and Service Robotics (FSR)*, 2019. (arXiv:1907.09224)

[8] M. Torres, D. A. Pelta, J. L. Verdegay, and J. C. Torres, "Coverage path planning with unmanned aerial vehicles for 3D terrain reconstruction," *Expert Syst. Appl.*, vol. 55, pp. 441–451, 2016.

[9] W. Jing, D. Deng, Y. Wu, and K. Shimada, "Multi-UAV coverage path planning for the inspection of large and complex structures," in *Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst. (IROS)*, 2020. (arXiv:2007.13065)

[10] C. Peng and V. Isler, "Adaptive view planning for aerial 3D reconstruction," in *Proc. IEEE Int. Conf. Robot. Autom. (ICRA)*, 2019, pp. 2981–2987. (arXiv:1805.00506)

[11] F. Stache, J. Westheider, F. Magistri, M. Popović, and C. Stachniss, "Adaptive path planning for UAVs for multi-resolution semantic segmentation," *Robot. Auton. Syst.*, vol. 157, p. 104288, 2022. (arXiv:2203.01642)

[12] M. Maboudi, M. R. Homaei, S. Song, S. Malihi, M. Saadatseresht, and M. Gerke, "A review on viewpoints and path planning for UAV-based 3D reconstruction," *IEEE J. Sel. Topics Appl. Earth Observ. Remote Sens.*, vol. 16, pp. 5026–5048, 2023. (arXiv:2205.03716)

[13] C. Connolly, "The determination of next best views," in *Proc. IEEE Int. Conf. Robot. Autom. (ICRA)*, vol. 2, 1985, pp. 432–435.

[14] W. R. Scott, G. Roth, and J.-F. Rivest, "View planning for automated three-dimensional object reconstruction and inspection," *ACM Comput. Surv.*, vol. 35, no. 1, pp. 64–96, 2003.

[15] A. Bircher, M. Kamel, K. Alexis, H. Oleynikova, and R. Siegwart, "Receding horizon 'next-best-view' planner for 3D exploration," in *Proc. IEEE Int. Conf. Robot. Autom. (ICRA)*, 2016, pp. 1462–1468.

[16] S. Isler, R. Sabzevari, J. Delmerico, and D. Scaramuzza, "An information gain formulation for active volumetric 3D reconstruction," in *Proc. IEEE Int. Conf. Robot. Autom. (ICRA)*, 2016.

[17] J. A. Delmerico, S. Isler, R. Sabzevari, and D. Scaramuzza, "A comparison of volumetric information gain metrics for active 3D object reconstruction," *Auton. Robots*, vol. 42, no. 2, pp. 197–208, 2018.

[18] J. I. Vasquez-Gomez, L. E. Sucar, and R. Murrieta-Cid, "View/state planning for three-dimensional object reconstruction under uncertainty," *Auton. Robots*, vol. 41, no. 1, pp. 89–109, 2017.

[19] M. Mendoza and J. I. Vasquez-Gomez, "Supervised learning of the next-best-view for 3D object reconstruction," *Pattern Recognit. Lett.*, vol. 133, pp. 224–231, 2020.

[20] D. Peralta, J. Casimiro, A. M. Nilles, J. A. Aguilar, R. Atienza, and R. B. Cajipe, "Next-best view policy for 3D reconstruction," in *Proc. ECCV Workshop on Embodied Vision*, 2020. (arXiv:2008.12664)

[21] Y. Ran, J. Zeng, S. He, J. Chen, L. Li, and Q. Chen, "NeurAR: Neural uncertainty for autonomous 3D reconstruction with implicit neural representations," *IEEE Robot. Autom. Lett.*, vol. 8, no. 2, pp. 1125–1132, 2023.

[22] R. Zeng, L. Ott, J. Nieto, and C. Cadena, "Neu-NBV: Next best view planning using uncertainty estimation in image-based neural rendering," in *Proc. IEEE/RSJ Int. Conf. Intell. Robots Syst. (IROS)*, 2023.

[23] B. Yamauchi, "A frontier-based approach for autonomous exploration," in *Proc. IEEE Int. Symp. Comput. Intell. Robot. Autom. (CIRA)*, 1997, pp. 146–151.

[24] B. Mildenhall, P. P. Srinivasan, M. Tancik, J. T. Barron, R. Ramamoorthi, and R. Ng, "NeRF: Representing scenes as neural radiance fields for view synthesis," in *Proc. Eur. Conf. Comput. Vis. (ECCV)*, 2020.

[25] T. Müller, A. Evans, C. Schied, and A. Keller, "Instant neural graphics primitives with a multiresolution hash encoding," *ACM Trans. Graph. (SIGGRAPH)*, vol. 41, no. 4, Art. 102, 2022.

[26] B. Kerbl, G. Kopanas, T. Leimkühler, and G. Drettakis, "3D Gaussian splatting for real-time radiance field rendering," *ACM Trans. Graph. (SIGGRAPH)*, vol. 42, no. 4, Art. 139, 2023.

[27] A. Yu, V. Ye, M. Tancik, and A. Kanazawa, "pixelNeRF: Neural radiance fields from one or few images," in *Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit. (CVPR)*, 2021, pp. 4578–4587.

[28] G. Wang, Z. Chen, C. C. Loy, and Z. Liu, "SparseNeRF: Distilling depth ranking for few-shot novel view synthesis," in *Proc. IEEE/CVF Int. Conf. Comput. Vis. (ICCV)*, 2023.

[29] R. Liu, R. Wu, B. Van Hoorick, P. Tokmakov, S. Zakharov, and C. Vondrick, "Zero-1-to-3: Zero-shot one image to 3D object," in *Proc. IEEE/CVF Int. Conf. Comput. Vis. (ICCV)*, 2023, pp. 9298–9309.

[30] L. Yang, B. Kang, Z. Huang, X. Xu, J. Feng, and H. Zhao, "Depth anything: Unleashing the power of large-scale unlabeled data," in *Proc. IEEE/CVF Conf. Comput. Vis. Pattern Recognit. (CVPR)*, 2024.

[31] L. Schmid, M. Pantic, R. Khanna, L. Ott, R. Siegwart, and J. Nieto, "A unified approach for autonomous volumetric exploration of large-scale environments under severe odometry drift," *IEEE Robot. Autom. Lett.*, vol. 6, no. 3, pp. 4504–4511, 2021.

[32] S. Kriegel, "Efficient next-best-scan planning for autonomous 3D surface reconstruction of unknown objects," *J. Real-Time Image Process.*, vol. 10, no. 4, pp. 611–631, 2015.

[33] C. Feng, H. Li, F. Gao, B. Zhou, and S. Shen, "PredRecon: A prediction-boosted planning framework for fast and high-quality autonomous aerial reconstruction," in *Proc. IEEE Int. Conf. Robot. Autom. (ICRA)*, 2023. (arXiv:2302.04488)

[34] X. Liu et al., "SOAR: Simultaneous exploration and photographing with heterogeneous UAVs for fast autonomous reconstruction," in *Proc. IEEE Int. Conf. Robot. Autom. (ICRA)*, 2024. (arXiv:2409.02738)

---

*Citation note: All references verified against IEEE Xplore, ACM DL, CVF Open Access, arXiv, DBLP, and Semantic Scholar. Reference [5] (OpenSfM) has no formal peer-reviewed paper; cited as a software repository per IEEE convention. Reference [34] (SOAR) author list is abbreviated (et al.) pending final camera-ready confirmation — verify before submission.*
