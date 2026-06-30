# Kinetix Work Plan

## Phase 1: Foundation (Weeks 1-4)
- Literature review (expand Related Work)
- Set up simulation environment (Gazebo + AirSim or Isaac Sim)
- Implement baseline traditional grid/circular path photogrammetry
- Choose evaluation metrics: reconstruction completeness, accuracy (Chamfer distance), image count, flight time

## Phase 2: Core Algorithm Development (Weeks 5-12)
- Monocular depth/uncertainty estimation (Depth Anything, DINO features, or custom model)
- Next Best View planner:
  - Candidate pose sampling
  - Information gain estimation
  - Optimization with constraints (drone dynamics, collision)
- Integrate with drone simulator
- Initial real-world tests with small drone (e.g., DJI Mini or custom)

## Phase 3: Optimization & Efficiency (Weeks 13-18)
- Refine planner with learning-based components (optional RL or imitation learning)
- Multi-objective optimization (quality vs. image count)
- Lighting and texture adaptation
- Pipeline: Capture → Real-time SfM → 3D Gaussian Splatting / Mesh reconstruction

## Phase 4: Evaluation & Experiments (Weeks 19-24)
- Benchmark against baselines on standard datasets + custom indoor objects
- Ablation studies on key components
- Real-world validation in different indoor scenarios
- Quantitative + qualitative results

## Phase 5: Paper Writing & Publication (Weeks 25-28)
- Target venues: CVPR, ICCV, IROS, ICRA, 3DV, or RA-L
- Prepare supplementary material (videos, code)
- Open-source dataset of indoor object captures if possible

## Resources Needed
- Drone hardware (monocular camera preferred)
- Computing: GPU workstation for reconstruction
- Software: COLMAP, PyTorch, ROS2

## Risks & Mitigations
- Indoor flight regulations → Use simulation heavily first
- Real-time performance → Start with offline planning, then move to online
- Reconstruction quality drop → Hybrid approach (initial dense + adaptive refinement)

**Timeline**: ~7 months to submission-ready paper.