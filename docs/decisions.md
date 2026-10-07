# Decisions (ADRs)

Short records. A decision changes only with a new entry that supersedes the old one.

## ADR-K01 — Reuse bisg_isaac as the platform, through its interface contract only (2026-10-08, accepted)

**Context.** The work plan needs Isaac Sim + Pegasus + PX4 SITL + MAVROS. That stack already exists
and is verified in `~/bisg_isaac` (Isaac 6.0, PX4 v1.17, MAVROS, ROS 2 Jazzy, same drone as the hardware).
**Decision.** Kinetix does not copy or fork any of it. T2/T3 use only the topics in
`bisg_isaac/docs/interface-contract.md`. Kinetix-specific sim code (`sim/render_server.py`, scenes)
lives here and runs in the `bisg/sim` image. Changes needed in bisg (scene mount, Q3) are made in bisg,
under its rules.
**Consequences.** No sim maintenance here. The same code flies sim and hardware. Kinetix depends on
bisg image tags (recorded in `run.yaml`). Until bisg has `vehicle/cmd`, `MavrosRig` streams setpoints itself.

## ADR-K02 — Discrete view pools (T0) as the primary research loop (2026-10-08, accepted)

**Context.** Flying every experiment in Isaac runs at 0.3–0.5× real time. A benchmark of hundreds of runs is not feasible that way.
**Decision.** Pre-render dense view pools with Isaac (T1 batch). Most development and benchmarking
selects from pools (T0). Continuous-pose (T1) and flown (T2) runs are a transfer subset.
**Consequences.** Deterministic, CI-able, minutes per matrix. The view space is discretised, so pool
density is a parameter and must be reported. Real dense captures become real pools with the same code.

## ADR-K03 — Monocular rule (2026-10-08, accepted)

**Decision.** Planning and reconstruction use only the left ZED image + `camera_info` + the vehicle
pose. Right image, ZED depth, point cloud, disparity, mapping and ZED odometry topics are forbidden
in `MavrosRig`, and a unit test checks this. The vehicle pose (EKF2, fed by ZED VIO on hardware)
is allowed as navigation and as a metric prior. Its use for reconstruction is an ablation axis
(`pose_source`) and is reported (Q5).

## ADR-K04 — pycolmap as the only SfM backend; COLMAP-format everywhere (2026-10-08, accepted)

**Decision.** Online and offline SfM use pycolmap (4.x: incremental and global mapping, triangulation, BA,
Sim3 alignment from Python). Every run stores COLMAP binary models, so any COLMAP-compatible tool
(MVS, 3DGS, nerfstudio) consumes the output. Dense MVS uses the CUDA `colmap/colmap` container
because the pip wheel is CPU-only.
**Consequences.** One dependency for geometry, and compatibility with the archival photogrammetry
workflow (a stated gap in related work). Online speed is limited by re-running mapping (v1) until v2 (K5).

## ADR-K05 — Python package with a uv venv; Docker only where ROS or Isaac is needed (2026-10-08, accepted)

**Decision.** `uv` venv on the host (Python 3.12) for core, T0, offline and bench. One `kinetix` image
(`ros:jazzy-ros-base` + the same lockfile, `--system-site-packages` for `rclpy`) for T1/T2/T3.
The host never gets a ROS install (matches bisg).
**Consequences.** Fast iteration without containers for most work. The same `uv.lock` in both places
keeps versions identical.

## ADR-K06 — Stop-and-shoot capture in v1 (2026-10-08, accepted)

**Decision.** The rig settles before every capture (no motion blur, exact poses, simple timing).
Capture on the move is a K5 optimisation with its own benchmark row (flight-time metric).
