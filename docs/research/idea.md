# Kinetix: Adaptive Dynamic Photogrammetry for Indoor Objects with Monocular Drone Imaging

## Project Overview
Kinetix is a novel system that enables efficient, high-quality 3D reconstruction of indoor objects using a monocular drone. Unlike traditional photogrammetry pipelines that rely on dense, pre-defined waypoint grids or circular orbits, Kinetix uses **dynamic, adaptive camera pose planning** to intelligently select the next best views in real-time (or near real-time), significantly reducing the number of required images while improving reconstruction quality.

## Core Problem
- Traditional drone photogrammetry for indoor items requires hundreds of overlapping images from fixed paths.
- Indoor environments are constrained (space, lighting, obstacles).
- Many images lead to longer flight times, higher energy consumption, and processing overhead.
- Generic paths often miss optimal angles for complex geometry.

## Proposed Solution
Kinetix introduces an **active vision** framework where the drone:
- Starts with a minimal set of seed images.
- Uses real-time analysis (depth estimation, uncertainty modeling, feature tracking) to predict the **Next Best View (NBV)**.
- Dynamically adjusts flight path, camera orientation, and capture parameters.
- Optimizes for coverage, completeness, and reconstruction confidence with minimal images.

## Key Innovations
Revised 2026-10-08 after the novelty search ([reports/Kinetix novelty search.md](reports/Kinetix%20novelty%20search.md)).
"Monocular active reconstruction", "foundation depth in NBV" and "50–80 % fewer images" are already
claimed in the literature. Kinetix's claims are therefore the following (paper wording in
[../publication-plan.md](../publication-plan.md)):
- **C1, SfM-anchored monocular NBV**: a relative-depth foundation model (Depth Anything V2) is scale-anchored
  online to the growing SfM reconstruction and fused into a TSDF frontier score. This is combined with an
  SfM-uncertainty score. Planning needs no depth sensor and no metric-depth model.
- **C2, what is depth for?**: the first controlled mono-vs-stereo (ZED Mini, 63 mm) study inside one
  NBV planner, separating depth for *planning* from depth as *input to reconstruction* (2×2 design).
- **C3, N@q**: an images-to-quality metric, defined as the number of images needed to reach q × the F-score of a dense
  capture of the same object.
- **C4, tiered realism protocol**: photoreal view-pool replay → continuous render → PX4 SITL → real
  flight, measuring how method rankings and savings shift with realism.
- **C5, quantitative real flights**: an indoor PX4/Jetson drone, scanned ground truth, unmodified COLMAP → MVS/3DGS
  output, released pools, twins, code and baselines.
- Indoor constraints (collision, standoff, fixed camera mount) are part of the method, not a contribution.

## Goals
- Achieve state-of-the-art reconstruction quality with significantly fewer images. The saving is a
  *reported result*, measured against the strongest baseline and set against prior reports of ~53–72 %, not a headline claim.
- Enable faster capture sessions suitable for real-world indoor applications (museums, warehouses, archaeology, e-commerce product modeling, etc.).
- Create a publishable research contribution in computer vision/robotics.

## Potential Impact
Kinetix could make high-quality photogrammetry accessible with consumer/prosumer drones, reducing time and computational costs dramatically.

**Project Name**: Kinetix  
**Tagline**: *Fewer Flights. Smarter Views. Better Models.*