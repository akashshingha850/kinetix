# Related Work

## Traditional Photogrammetry

- **Structure-from-Motion (SfM)**: COLMAP [Schönberger & Frahm, 2016], Agisoft Metashape, RealityCapture. Rely on dense image sets from manual or grid-based paths.
- **Drone-based Systems**: Commercial solutions (DJI Terra, Pix4D) use pre-planned missions with high overlap (70–90%). Mostegel et al. [2016] propose quality-driven autonomous re-capture for UAVs using multi-view-stereo confidence prediction, representing an early online feedback loop. Pan et al. [2024] recently extend this idea to full on-the-fly SfM with predictive path planning.

Limitations: Inefficient for indoor objects; do not adapt to scene geometry; no object-centric view selection.

## Active Vision and Next Best View (NBV)

- **Next Best View Planning**: Classic problem formalized by Connolly [1985] and surveyed comprehensively by Scott et al. [2003].
- **Uncertainty-based NBV**: Methods using volumetric uncertainty or information gain, e.g., receding-horizon voxel exploration [Bircher et al., 2016] and a rigorous comparison of information-gain metrics [Delmerico et al., 2018]. Vasquez-Gomez et al. [2017] extend these to object-level reconstruction with explicit positioning uncertainty.
- **Learning-based View Planning**: Mendoza & Vasquez-Gomez [2020] use a supervised 3D-CNN to predict the NBV from partial point clouds. Peralta et al. [2020] apply reinforcement learning (Scan-RL) to learn a view policy for drone-based scanning. More recently, Ran et al. [2023] (NeurAR) and Zeng et al. [2023] (Neu-NBV) integrate NBV selection with neural-rendering uncertainty estimation.

## Monocular 3D Reconstruction

- **Neural Radiance Fields (NeRF)** [Mildenhall et al., 2020] and accelerated variants such as Instant-NGP [Müller et al., 2022] and 3D Gaussian Splatting [Kerbl et al., 2023] achieve high quality but typically require dense, calibrated views.
- **Sparse-view methods**: pixelNeRF [Yu et al., 2021] uses a learned scene prior for few-shot novel-view synthesis. SparseNeRF [Wang et al., 2023] distills monocular depth-rank supervision to enable NeRF training from 3–5 views. Liu et al. [2023] (Zero-1-to-3) demonstrate diffusion-based novel-view synthesis from a single image.
- **Monocular depth foundation models**: Depth Anything [Yang et al., 2024] provides robust zero-shot monocular depth estimation trained on 60M+ images, offering a scalable depth prior for systems lacking metric sensors.

## Drone-Specific Adaptive Photogrammetry

- **Autonomous Exploration**: Bircher et al. [2016] demonstrate environment-level exploration with UAVs using receding-horizon NBV. Schmid et al. [2021] address autonomous volumetric exploration of large-scale environments under severe odometry drift.
- **Object-centric Scanning**: Kriegel [2015] proposes efficient next-best-scan planning for object surface reconstruction using robotic arms — principles applicable but not directly transferable to drone platforms.
- **Recent Works**:
  - Online SfM feedback with quality-aware replanning [Pan et al., 2024].
  - Neural-uncertainty-driven NBV validated on real robots [Ran et al., 2023; Zeng et al., 2023].

## Gaps This Work Addresses

- Most NBV methods assume metric depth (RGB-D or LiDAR); none target **monocular-only** active reconstruction on indoor drones.
- Object-level scanning is studied primarily with robotic arms [Kriegel, 2015] or depth sensors [Delmerico et al., 2018; Vasquez-Gomez et al., 2017]; drone-based object-centric scanning in indoor constrained spaces remains underexplored.
- Neural-uncertainty NBV approaches [NeurAR, Neu-NBV] operate within NeRF output pipelines; none maintain **COLMAP + Gaussian Splatting compatibility** required for archival photogrammetric workflows.
- Existing efficiency claims rarely report standard photogrammetric metrics (Chamfer distance, F-score); Kinetix provides this bridge.

Kinetix uniquely bridges active vision, monocular drone imaging, indoor constraints, and photogrammetric pipeline compatibility.

## References

- Bircher, A., Kamel, M., Alexis, K., Oleynikova, H., & Siegwart, R. (2016). Receding horizon "next-best-view" planner for 3D exploration. *IEEE International Conference on Robotics and Automation (ICRA)*, 1462–1468.
- Connolly, C. (1985). The determination of next best views. *Proceedings of the IEEE International Conference on Robotics and Automation (ICRA)*, 2, 432–435.
- Delmerico, J. A., Isler, S., Sabzevari, R., & Scaramuzza, D. (2018). A comparison of volumetric information gain metrics for active 3D object reconstruction. *Autonomous Robots*, 42(2), 197–208.
- Kerbl, B., Kopanas, G., Leimkühler, T., & Drettakis, G. (2023). 3D Gaussian splatting for real-time radiance field rendering. *ACM Transactions on Graphics (SIGGRAPH)*, 42(4), Article 139.
- Kriegel, S. (2015). Efficient next-best-scan planning for autonomous 3D surface reconstruction of unknown objects. *Journal of Real-Time Image Processing*, 10(4), 611–631.
- Liu, R., Wu, R., Van Hoorick, B., Tokmakov, P., Zakharov, S., & Vondrick, C. (2023). Zero-1-to-3: Zero-shot one image to 3D object. *Proceedings of ICCV 2023*, 9298–9309.
- Mendoza, M., & Vasquez-Gomez, J. I. (2020). Supervised learning of the next-best-view for 3D object reconstruction. *Pattern Recognition Letters*, 133, 224–231.
- Mildenhall, B., Srinivasan, P. P., Tancik, M., Barron, J. T., Ramamoorthi, R., & Ng, R. (2020). NeRF: Representing scenes as neural radiance fields for view synthesis. *European Conference on Computer Vision (ECCV)*.
- Mostegel, C., Rumpler, M., Fraundorfer, F., & Bischof, H. (2016). UAV-based autonomous image acquisition with multi-view stereo quality assurance by confidence prediction. *IEEE CVPR Workshops*.
- Müller, T., Evans, A., Schied, C., & Keller, A. (2022). Instant neural graphics primitives with a multiresolution hash encoding. *ACM Transactions on Graphics (SIGGRAPH)*, 41(4), Article 102.
- Pan, J., Wang, H., Yang, L., & Xu, F. (2024). On-the-fly feedback SfM: Online explore-and-exploit UAV photogrammetry with incremental mesh quality-aware indicator and predictive path planning. *arXiv:2512.02375*.
- Peralta, D., Casimiro, J., Nilles, A. M., Aguilar, J. A., Atienza, R., & Cajipe, R. B. (2020). Next-best view policy for 3D reconstruction. *ECCV Workshop on Embodied Vision*. (arXiv:2008.12664)
- Ran, Y., Zeng, J., He, S., Chen, J., Li, L., & Chen, Q. (2023). NeurAR: Neural uncertainty for autonomous 3D reconstruction with implicit neural representations. *IEEE Robotics and Automation Letters*, 8(2), 1125–1132.
- Schmid, L., Pantic, M., Khanna, R., Ott, L., Siegwart, R., & Nieto, J. (2021). A unified approach for autonomous volumetric exploration of large-scale environments under severe odometry drift. *IEEE Robotics and Automation Letters*, 6(3), 4504–4511.
- Schönberger, J. L., & Frahm, J.-M. (2016). Structure-from-motion revisited. *Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition (CVPR)*, 4104–4113.
- Scott, W. R., Roth, G., & Rivest, J.-F. (2003). View planning for automated three-dimensional object reconstruction and inspection. *ACM Computing Surveys*, 35(1), 64–96.
- Vasquez-Gomez, J. I., Sucar, L. E., & Murrieta-Cid, R. (2017). View/state planning for three-dimensional object reconstruction under uncertainty. *Autonomous Robots*, 41(1), 89–109.
- Wang, G., Chen, Z., Loy, C. C., & Liu, Z. (2023). SparseNeRF: Distilling depth ranking for few-shot novel view synthesis. *Proceedings of ICCV 2023*.
- Yang, L., Kang, B., Huang, Z., Xu, X., Feng, J., & Zhao, H. (2024). Depth anything: Unleashing the power of large-scale unlabeled data. *Proceedings of CVPR 2024*.
- Yu, A., Ye, V., Tancik, M., & Kanazawa, A. (2021). pixelNeRF: Neural radiance fields from one or few images. *Proceedings of CVPR 2021*, 4578–4587.
- Zeng, R., Ott, L., Nieto, J., & Cadena, C. (2023). Neu-NBV: Next best view planning using uncertainty estimation in image-based neural rendering. *IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*.

---

