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

## ADR-K03 — Monocular rule (2026-10-08, **superseded by ADR-K07**)

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

## ADR-K07 — Sensing mode is an experiment axis: monocular method, stereo comparison (2026-10-08, accepted)

**Context.** The drone carries a ZED Mini (stereo). A monocular-only design leaves "why not use the
stereo?" unanswered, and ActMVS (2026) narrows the monocular novelty claim. Study:
[research/mono-vs-stereo.md](research/mono-vs-stereo.md).
**Decision.** Two independent config axes, `sensing.plan_depth` (none | mono | stereo | gt) and
`sensing.recon_input` (left | stereo_rig | stereo_rig+depth), with named presets. `mono` stays the
Kinetix method. Stereo presets are comparisons that run through the same loop, offline pipeline and
metrics. The rig's topic whitelist is derived from the mode. ZED point cloud, mapping and odometry
are never Kinetix inputs. Pools always store left + right + GT depth.
**Consequences.** Pools are about 2× larger. One more dependency at K3 (FoundationStereo; SGBM fallback).
A depth study is needed to justify learned stereo as a stand-in for the ZED SDK on T0. The paper's claim
becomes a measured mono-vs-stereo trade-off instead of an assumption.

## ADR-K08 — Position Kinetix as a measurement paper; strengthen baselines and protocol compatibility (2026-10-08, accepted)

**Context.** The novelty search ([research/reports/Kinetix novelty search.md](research/reports/Kinetix%20novelty%20search.md))
found the image-saving headline, "first monocular active reconstruction" and "foundation depth in NBV" already
claimed. A combination and three experimental contributions remain open.
**Decision.** The paper's claims are C1–C5 ([publication-plan.md](publication-plan.md)). The image saving is a
reported result against the strongest baselines. The baseline set grows to fixed patterns + `fvs` + `volumetric_ig`
+ `fisherrf` + `ma_scvp`. Pools add an ObjView-Bench-compatible `tammes:128` layout with reachability masks. Metrics add
N@q at q ∈ {0.9, 0.95} with reference-density sensitivity, coverage AUC, K = 5/30 budgets and Kendall's τ across tiers.
C1 must win a decisive anchoring ablation (go/no-go at G2).
**Consequences.** More baseline integration work in K2–K3 (two external code bases). The object set must include
community-pool objects and is constrained by licences. Real flights must be quantitative.

## ADR-K09 — ROS 2 pipeline: ROS at the edges, one goto action + one snapshot service (2026-10-10, proposed)

**Context.** T1/T2/T3 need ROS. The current design uses two rig adapters (`IsaacRenderRig` and `MavrosRig`) that speak different
protocols. The design has a T1 request/response emulated over topics, and `MavrosRig` subscribes to 30 Hz HD1080 image streams (≈ 250 MB/s each on
CycloneDDS loopback) to keep one frame per view.
**Decision.** [ros-pipeline.md](ros-pipeline.md). Planner/recon stay pure Python in one `session` node. The rig side is two
generic nodes, `goto` (action `kinetix_msgs/Goto`, the only setpoint producer) and `snapshot` (service `kinetix_msgs/Snapshot`,
lazy sensor subscription, stream whitelist from the sensing mode). The T1 render server serves the same two interfaces, so one
client, `RosRig`, covers T1–T3. Kinetix itself publishes only `kinetix/status`, `kinetix/viz` and `kinetix/recon/points`, all transient-local and the last two only while subscribed.
**Consequences.** Steady-state traffic ≈ 50 KB/s plus one lossless frame set per view. A `kinetix_msgs` package is built in the kinetix image.
`goto` / `snapshot` carry no Kinetix types, so bisg can adopt them (`goto` = the contract's `offboard_controller`).
interfaces.md §4/§5 change on acceptance. The latency and QoS assumptions are measured at K4 (ros-pipeline.md §8).

## ADR-K10 — Pools carry real ZED SDK depth, rendered through the twin (2026-10-10, proposed)

**Context.** ADR-K07 planned FoundationStereo (SGBM fallback) as the stereo depth of T0/T1 pools because the ZED SDK cannot read
stored images, which left Q6 (is learned stereo a fair stand-in for the SDK?) open. bisg's pool-depth probe (2026-10-10,
bisg `docs/zed-sdk-sim.md`) streamed teleported views of the ZED_M twin into the real SDK. Depth settled ≤ 0.54 s after a teleport, with median error
0.35 % (0.3–2 m) / 0.65 % (2–5 m) and revisits within 0.5–0.8 %, at ~0.7 s per view.
**Decision.** `render_server.py --zed-depth` stores the SDK's `depth_registered` + `confidence_map` per view (`zed/`, `zed_conf/`), taken
≥ 0.6 s after the teleport. `stereo_zed` reads them on T0/T1, so T0 and T2 share one depth source. FoundationStereo/SGBM stay as the fallback
(pools without the twin, real dense captures without an SVO) and as an ablation row.
**Consequences.** One pool = one sim session with one wrapper (bisg B18). Pool rendering takes ~0.7 s per view (≈ 7 min / 600 views). Pools grow
by two float32 maps per view. Sim stereo has no sensor noise, so sim SDK depth is an upper bound; the real half of the depth study uses
SVO recordings (`./bisg zed record`). Q6 is closed if this is accepted.

## ADR-K11 — Room scenes: the whole room is the reconstruction target (2026-10-17, accepted)

**Context.** The user's target changed from one object to **photogrammetry of an entire room** (bisg's default world is now the
Isaac Simple Warehouse, 19.3 × 29.8 × 9 m inside). The scene format (one `roi` box, `standoff_m` to its surface), the ring-stack
sampler and the keep-out shell are object-centred (outside-in). A room is inside-out: the cameras are inside the surface they
reconstruct, and obstacles (racks, carts) both block flight and are part of the surface.
**Decision.** A scene declares `kind: object` (default, unchanged) or `kind: room`. In a room scene `roi` is the room's
**inner** box: its walls, floor and ceiling (inward normals) plus the outer faces of the `no_fly` boxes are the surface proxy, and
the `no_fly` boxes occlude it (`recon/coverage.py`, same triangulation rule as objects: two rays ≥ `tri_min_deg` apart).
Candidates are a 4-DoF grid in the `workspace` (`planner/sampler.py` `GridSampler`). A candidate is only worth flying to if
at least `min_overlap` of what it sees is already seen, so every new image can register in SfM (connectivity, which an
object orbit gets for free). Room-specific defaults live in `config/room.yaml`, layered automatically after `default.yaml` for room
scenes. The online model is still the GT-free coverage proxy; offline, `kinetix sfm` reconstructs the captured images with pycolmap
(registration, ATE, completeness against the proxy surface). The MAVROS rig records each flight to an MCAP rosbag in the run dir.
**Evidence (2026-10-17, Warehouse, T2).** Flight 1 (47 views planned with a 35° match limit and a 30 % overlap fraction, 351 m,
21 min, landed): exhaustive SIFT matching "verified" 817 of 1128 pairs, most of them false matches between identical 6 m bays
(697 fundamental-matrix fits on a calibrated camera); the one model put 28/48 cameras within 6.5 cm of GT and the other 20 6-28 m
off. Checking every verified pair against the rig's relative pose (rotation ≤ 5°, baseline direction ≤ 30°) removes them: the model
is then correct (24/24 cameras, ATE 6.9 cm vs GT) but only half the images connect. Measured over all pairs, P(match) is 0.36-0.48
below 30° between the views, 0.15 at 30-45°, 0.04 at 60-90°, 0 beyond. So the room defaults are `match_max_deg` 20 and an absolute
shared-surface gate (`min_shared` 120 points) instead of a fraction, and offline SfM pairs images by co-visibility + the rig check.
**Consequences.** The proxy is a box room: interior structure that is not a `no_fly` box (roof rafters, wall pillars) neither occludes
nor counts. Repetitive interiors make pure SfM unreliable without the rig poses: the rig check uses EKF2 relative poses (Q5: the
drone pose as a prior), so `pose_source: sfm` alone is not enough here. The paper claims C1–C5 (publication-plan.md) are written for objects; whether the room becomes the paper's target or
a second scene class is open and does not change them yet. The K3 scorers (`sfm_uncertainty`, `depth_frontier`, `combo`) will need
the same inside-out visibility, which `recon/coverage.py` now provides.

## ADR-K12 — Rooms: plan the tour before take-off, fly it continuously, triangulate at the rig poses (2026-10-18, accepted)

**Context.** Stop-and-shoot NBV in the Warehouse (ADR-K11) took 12-17 min of drone time (26-28 min wall at the sim's RTF 0.47-0.6):
62 % of it flying 6.6 m zig-zag hops at 0.8 m/s (the greedy next view ignores the route), 38 % stopping (accelerate, turn, settle,
grab: 5.2 s per view). Planning was 2 % (0.6 s per step). The long path also made the VIO drift (random walk, 0.3-2.4 m per
350-416 m) and exposed bisg's R9 wrapper crash (one flight lost after 24 min). The target is a 5-10 min flight. Images are cheap
to process, flight time is not. SOTA fast aerial reconstruction plans a viewpoint set, orders it as a TSP and flies a continuous
trajectory (PredRecon, Plan3D, FC-Planner); for repetitive interiors, SfM fails on visual aliasing (Doppelgangers++) and drifts on
low-parallax video, and drone photogrammetry fuses the platform poses (GNSS/RTK priors).
**Decision.** Method `tour` (`planner/tour.py`): greedy set cover of the room proxy to `planner.tour.coverage_target` (0.8)
without travel cost, an open TSP (nearest neighbour + 2-opt, metres + 1 m per radian of turn), a dense path, and `MavrosRig.fly_path`:
a time-parameterised carrot at `cruise_mps` 1.0 with turns paced at `yaw_rate_deg_s`, keyframes every 0.4 m / 6° from the continuous
left stream, no stopping. Offline (`kinetix sfm`): `video` pairing (10 sequential neighbours + 8 most co-visible frames, rig-checked)
and `pose_source: prior` (triangulate at the EKF2 poses, bundle adjustment with fixed intrinsics, re-triangulate), the default for
tours. Stop-and-shoot (ADR-K06) stays for objects and for the NBV research loop.
**Evidence (2026-10-18, Warehouse, T2).** Dry: 34 viewpoints, 112 m, 2.2 min est., keyframes give 0.91 proxy coverage (NBV: 71
images, 416 m). Flown: 32 viewpoints, 108 m, **2.0 min of drone time** (5 min wall from take-off to landing), 403 keyframes, EKF2
vs GT 0.19 m RMS (max 0.36), landed. SfM: incremental + priors registered 98 % but put the first half of the tour one bay (6 m) off;
known-pose triangulation + BA: **403/403 cameras, ATE 8.2 cm vs GT (EKF2 itself 18.9 cm), 65 k points, 0.65 px, proxy completeness
0.55 (sparse), 55 s of processing**.
**Consequences.** The method needs a room map before flight (the scene file: room box + obstacles), so it is coverage planning, not
exploration; an unknown room needs a first exploration pass. Its quality depends on the rig poses being right to a few decimetres
over the flight, which a 2-min tour gives in the sim (the real ZED adds its 400 Hz IMU). The online coverage model is a proxy; the
true completeness needs dense reconstruction (K2).
