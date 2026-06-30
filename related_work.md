# Related Work

## Traditional Photogrammetry
- **Structure-from-Motion (SfM)**: COLMAP, Agisoft Metashape, RealityCapture. Rely on dense image sets from manual or grid-based paths [Schönberger et al., 2016].
- **Drone-based Systems**: Commercial solutions (DJI Terra, Pix4D) use pre-planned missions with high overlap (70-90%).

Limitations: Inefficient for indoor objects; do not adapt to scene geometry.

## Active Vision and Next Best View (NBV)
- **Next Best View Planning**: Classic problem in robotics and computer vision [Connolly, 1985; Scott et al., 2003].
- **Uncertainty-based NBV**: Methods that use volumetric uncertainty or information gain (e.g., voxel entropy) [Bircher et al., 2016; Vasquez-Gomez et al., 2017].
- **Deep Learning for View Planning**: Reinforcement learning (RL) and graph neural networks for predicting optimal views [Mendoza et al., 2021; Ran et al., 2022].

## Monocular 3D Reconstruction
- **Neural Radiance Fields (NeRF)** and derivatives (Instant-NGP, 3D Gaussian Splatting): High quality but often require dense views [Mildenhall et al., 2020; Kerbl et al., 2023].
- **Recent sparse-view NeRFs**: Efforts to reduce input images using priors (e.g., Zero-1-to-3, SparseNeRF) [Wang et al., 2023; Xu et al., 2024].

## Drone-Specific Adaptive Photogrammetry
- **Autonomous Exploration**: Systems like those in [Schmid et al., 2020] for cave mapping.
- **Object-centric Scanning**: Works on robotic arms for objects (e.g., [Kriegel et al., 2015]), less common on drones.
- **Recent Works**: 
  - Adaptive path planning for drones using semantic segmentation and coverage metrics.
  - Real-time SfM feedback loops [Mostegel et al., 2016].

## Gaps This Work Addresses
- Most NBV methods are simulation-heavy or use RGB-D sensors.
- Few works focus on **monocular drone** in **indoor constrained spaces** for **object-level** reconstruction.
- Limited emphasis on minimizing image count while maintaining photogrammetry compatibility (COLMAP + Gaussian Splatting pipeline).

Kinetix aims to bridge active vision with practical monocular drone photogrammetry.