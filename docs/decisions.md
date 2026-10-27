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


## ADR-K13 — Unknown rooms: explore with the ZED depth at the EKF2 pose, fit the room, then tour (2026-10-24, accepted; dry rig only)

**Context.** `tour` (ADR-K12) reads the room box and obstacles from the scene file: coverage planning of a known room. A new room
needs a person to measure and type it in. The drone carries a stereo camera whose depth is already an experiment axis (ADR-K07:
`plan_depth: stereo`); the flight controller's EKF2 already fuses the ZED odometry with its own IMU, barometer, rangefinder and
optical flow, and is the pose Kinetix flies and reconstructs on. The ZED's own spatial map lives in the ZED's tracking frame, which
EKF2 corrects, so it is not the map to plan with (ADR-K07 keeps ZED mapping and odometry out of the method).
**Decision.** `kinetix run --scene … --method tour --explore` (config `explore.*`):
1. **Live map** (`recon/depthmap.py`): a voxel log-odds grid (0.2 m) fed with the ZED `depth_registered` image at the **rig pose**
   (EKF2 ∘ camera mount). Returns inside 8 m mark a voxel occupied and carve the ray free; a return farther out carves free space up to
   8 m and marks nothing. Map and flight path share one frame by construction. This is the fast, coarse tier.
2. **Room fit** (`recon/roomfit.py`): walls, floor and ceiling are the outer voxel slices (plane position = centroid of the hits in
   the outer slices, unbiased by depth noise), obstacles are 26-connected components inside the room (boxes; a floor-standing one is
   extended to the floor), workspace and altitude band are the room shrunk by the flight margins. Output is an ordinary `Scene`.
3. **Exploration** (`planner/explore.py`): a yaw sweep, then frontier views: score reachable viewpoints by the frontier voxels in
   their cone, re-score the best few with a line-of-sight test, fly there. The drone parks only where it has seen the surroundings
   empty and flies only through known-free voxels, clear of occupied space by at least `planner.clearance_m` (the tour starts where
   exploration ends and boxes obstacles by that). Stop: every face seen to 60 %, or the fit unchanged for 4 steps with every face
   seen to 25 %, or the step limit. Frontier voxels farther than 6 m stay frontier until a closer view resolves them.
4. The scene file's room and obstacles are never read in this mode; only its take-off point, altitude band and `world_to_map`. The
   file, when present, only scores the fit (`explore/report.json: vs_truth`). The accurate tier is unchanged: `kinetix sfm` with the
   EKF2 poses as priors and bundle adjustment.
**Evidence (dry rig, synthetic depth with 1 % noise and 20 m range).** Room 12×9×4 m with two crates: closed after 11 views (17 m of
hops), walls within 0.07 m, both crates found. Warehouse (19.3×29.8×9 m): stable after 27 views (124 m of hops), walls within 0.04 m,
no false obstacles; the tour on the explored room has 24 viewpoints, 96.7 m, est. 111 s, against 24, 101 m, 116 s on the scene file.
Live data path checked read-only on the sim: one ZED depth frame (1280×720 32FC1, 0.24-15 m) at the EKF2 pose maps the floor ahead.
**First live flight (2026-10-24, sim, `explore: true` from the GCS) FAILED, and the dry rig had hidden why.** 38 views took 11 min
wall (16 s per stop-and-shoot view at RTF 0.57). The map was built on EKF2, which drifted from ground truth gradually (path length
ratio 0.99, so the scale is right): mean error 0.03 m in minute 1, 0.2 m by minute 5, 0.7 m in minute 6-7, 1.4 m by minute 10-12
(no step above 0.01 m between samples: EKF2 smooths the ZED jumps, so the gate cannot be judged from this output). Late views
re-saw the walls 1.1-1.5 m off, so the fit grew walls out to 21.6×31.6×9.1 m (true 19.3×29.8×9.0, max error 1.64 m) and 30 false
obstacles (thin sheets beside the walls), the faces counted against that oversized box never reached 25 % (2-3 % all flight) and the
stop rule never fired. The tour then had no feasible viewpoint on that scene and crashed in `select_viewpoints` (the `finally` landed
the drone). Real ZED depth is also valid almost everywhere (14400/14400 rays), unlike the dry rig's.
Changes: exploration tolerances loosened (`explore.rig`: 0.35 m / 10° / 0.3 m/s, settle 0.2 s) and 20 views at most, so a run is a few
minutes, not eleven; thin wall-parallel sheets near a wall are dropped as drift duplicates (`wall_artifact_m`); unknown space past a
range-limited ray is flagged probed, not frontier; the tour stops with `no_feasible_view` instead of crashing; the dry rig has a
drift model (`dry_drift_m_per_sqrt_m`, 0.1 ≈ the sim) and 15 m depth. **Open:** one still-grounded frame suggests the sim's ZED depth
reads ~10 % short of the room box (ratio 0.86-0.92 over 13 columns at 8-15 m; unexplained, may be the box, may be the sim's stereo
calibration): check with a depth-box test before trusting wall positions from far returns.
**Consequences.** Exploration adds flight time before the tour, so a flown run is about three times a tour alone, and it is only as
good as EKF2 stays accurate over those minutes: short exploration is a requirement, not a tuning choice. Depth error grows with range
(stereo: ∝ z²), hence the 8 m mapping range and the closer re-look. Stereo depth fails on blank walls and glass; a missing return is
never treated as free space. A wrong wall comes from a wrong EKF2 pose, which is why the map is built on the same estimate the
controller flies, so its drift is the limit. bisg_isaac's ZED odometry jump gate was meant to remove the event part of it; it does not
work in the sim yet (bisg_isaac docs/vio.md), so there is no fix for the pose yet and the guards above are what keeps a bad exploration from
becoming a crash.
**Second live flight (2026-10-24, after the changes above): exploration faster, still unusable, and the run crashed the drone.** 28
views in 300 s wall (about 170 s sim; 28 views in 672 s before), but EKF2 drifted faster this time (0.06 m at 1 min, 0.55 m at 3-4 min,
1.0 m at 4-5 min, 1.2-1.4 m at 6-7 min; the two runs differ a lot, the sim's ZED has random tracking events), the depth registration
estimated 0.05 m of drift against about 1 m (it only moves along axes whose score has curvature, and the hall gives it little to hold
on to), the fit had walls 0.4-1.3 m off, 15 false obstacles and 2-14 % of the walls seen. The tour planned 120 viewpoints on that scene
and flew it; at minute 14 EKF2 diverged (up to 45 m error), the drone fell, and the rig streamed setpoints to it until the path
timeout, 33 minutes. Existing known limit (todo): bisg's sim VIO drifts over long flights, so flights past 2-3 minutes are not reliable
there; an exploration plus a tour is 8+ minutes. **Guards added, whatever the cause:** the rig aborts when the drone stays more than
3×`max_lead_m` behind the setpoint for `stall_s` or when EKF2's speed is not physical (`max_speed_mps`); exploration stops at
`explore.max_time_s` (240 sim seconds); the tour is not flown unless every wall and the floor were seen to `explore.tour_min_wall`
(the run lands, `explore_incomplete`); a tour with no feasible viewpoint lands instead of raising. Depth registration is kept (dry
model: residual drift 0.99 → 0.59 m, false obstacles 2 → 0 at 0.05 m/√m; never worse in six trials) but it is a partial correction, not
a replacement for short flights or for a better pose.
**NBV benchmark (2026-10-26, dry rig, no drift: `tests/nbv_bench.py`).** Four box rooms the planner never sees (12×9 m with 2 crates,
a low 10×8×3.5 m room with 6 crates, 14×10 m split by a floor-to-ceiling partition, the Warehouse), two take-off points each. Every
candidate the NBV scored is also flown in simulation (depth cast against the true room) and compared with the prediction; the whole
exploration is run with the NBV, the cone score without line of sight, the nearest view, a random view and an oracle that picks by the
true outcome. **Found and fixed:** (1) `los_clear` rebuilt the occupied mask of the whole grid at every ray sample: 85 % of planning time,
671 → 89 ms per view in the Warehouse, same picks; (2) candidates on a fixed 1 m lattice (`safe[::5]`) missed thin free space: the low
cluttered room got 1 candidate and exploration ended after the take-off sweep; now one random safe voxel per grid cell; (3) only
straight segments from the drone were reachable, so in clutter nothing was; now `Reach` (a wavefront through the corridor) gives the
flight distance and, for a view that is not in line, the first straight leg there (`reach_cell: 1`; 2 finds no leg in clutter); every
leg stays in the altitude band (a first version flew legs to 6.2 m against a 4.9 m ceiling); (4) a partition covering 80 % of the
face the fit put on it made the room 'closed' 7 m short: free space seen 2-4 voxels past a face is an opening, and neither `closed` nor
`stable` fires with one (`fit.max_opening_voxels`); (5) the line-of-sight re-score now goes past the first 8 views when they all
look through a wall (`los_max`); (6) unknown space past the 8 m range was never frontier, so nothing led to a wall farther than that:
the range frontier counts too (`range_frontier_weight: 1.0`; Warehouse: worst wall seen 9 → 19 % and 8 → 12 %, 147 → 111 m flown).
**Result.** Prediction: in the three small rooms the NBV ranks candidates as their true outcome does (Spearman 0.86-0.98 on resolved
frontier) and picks within 0-16 % of the best (mean regret); in the Warehouse the ranking is weaker (0.43-0.47), but the regret is still
5-7 % because many views are about as good. End to end it matches the oracle within a few views (the partition room: 19 / 15 views
for the NBV against 15 / 16 for the oracle), the cone-only score needs 28 views there (line of sight matters), and the nearest and random
views fit the partition as the wall or end at the step limit. Walls within 0.09 m in every small room, 5/6 crates in the cluttered one (the 0.6 m-high one is under the fit's
floor margin, by design). Planning 25-65 ms per view in the small rooms, 160-210 ms in the Warehouse (desktop CPU; the oracle takes
1.4-3 s). The parameter sweep found no setting better everywhere: the defaults stay. **Limit:** no strategy, oracle included, closes the
Warehouse in 20 views (best 22 % of the worst wall): with a level camera, an 8 m range and a 9 m ceiling the views needed exceed the
budget, so a large room passes the tour gate (10 %) but is not 'closed'.
**Pose fix on the bisg side (2026-10-26, bisg_isaac docs/vio-drift.md).** The drift behind the two failed flights is the sim ZED losing
its heading in in-place turns (2-40 deg a slip) and jumping its position with it; EKF2 followed it. bisg now holds the heading with a
Manhattan compass in the zed-bridge (the room's walls in the ZED depth) and fuses the ZED velocity + yaw only (sim default). Same
Kinetix code, `--explore` in the Warehouse, two flights each: EKF2 max 0.62-0.98 m (mean 0.23-0.47) against 1.50-2.73 m (mean
0.64-0.76) before, worst fitted wall 0.66-1.04 m against 2.41-4.11 m. Still no tour: every run stopped at the 20-view budget with a
face seen < 10 % (the large-room limit above), and the walls stay ~1 m off with EKF2 at 0.2-0.5 m, which points at the sim ZED's
depth at range (reads ~10 % short at 8-15 m, open above) rather than the pose.
