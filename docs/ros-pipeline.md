# ROS 2 pipeline and topic budget

Status: **proposed** (ADR-K09, [decisions.md](decisions.md)). Once it is accepted, the tables in §3 and §4 replace
[interfaces.md](interfaces.md) §4 and §5, and this file keeps only the rationale and the budget.
Verification: design only (static reading of the bisg contract and `docker/zed/zed.yaml`). Nothing here has run on a live graph. §8 lists what to measure.

## 1. Principles

1. **ROS at the edges.** `planner/`, `recon/` and `core/` stay pure Python (the dependency rule in
   architecture.md §4 does not change). ROS lives in thin nodes and in one rig client, `rig/ros.py`.
   The scorers query `ReconState` as numpy in-process. Putting that query on the wire would cost a copy per candidate and add nothing.
2. **One pattern per interaction** (ROS 2 rule: a stream is a topic, a quick request is a service, and a long cancelable task is an action).
   "Fly there" takes seconds and must be cancelable, so it is an **action**. "Give me the next frame after t" is a **service**.
   Topics are only for streams and for observability.
3. **Pixels cross a process boundary once per capture**, never as a 30 Hz stream (§5). Zero-copy is an
   upgrade path for the `snapshot` node only, behind the same interface (§5a).
4. **One rig interface for T1, T2 and T3.** The Isaac render server and the MAVROS/ZED nodes expose the same
   `goto` action and `snapshot` service. `IsaacRenderRig` and `MavrosRig` merge into one client, `RosRig`.
5. **Generic, adoptable pieces.** `goto` and `snapshot` use no Kinetix types (they carry no pool ids, scores or run dirs).
   `goto` is the setpoint producer bisg's contract already names (`bisg_vehicle/offboard_controller`), so it can move
   into bisg unchanged. Any bisg mission can use `snapshot` to take a "settled photo".
6. **Two custom interfaces only** (`Goto.action`, `Snapshot.srv`). Everything else uses standard types.

## 2. Node graph (one drone, namespace `/drone_<n>`)

```
 kinetix image                                         bisg (unchanged)
┌───────────────────────────────┐
│ session  (planner+recon+log)  │
│   RosRig client               │
│     │ action  kinetix/goto ───┼──► goto ───────── pub mavros/setpoint_position/local (20 Hz) ─► MAVROS ─► PX4
│     │ srv  kinetix/snapshot ──┼──► snapshot        sub mavros/state, mavros/local_position/odom
│     ▼                         │      │             sub tf_static (T_base_cam)
│   pub kinetix/status          │      └─ lazy sub  zed/zed_node/left/color/rect/{image,camera_info}
│   pub kinetix/viz      (debug)│         (+ right/…, depth/depth_registered by sensing mode)
│   pub kinetix/recon/points    │         sub mavros/local_position/odom (pose at image stamp)
└───────────────────────────────┘         sub state/pose (sim, gt only)

 T1: render_server (bisg/sim image) serves the same kinetix/goto + kinetix/snapshot. It has no MAVROS and no ZED.
 T0: PoolRig, no ROS at all.
```

`goto` and `snapshot` are two small processes, or two components in one container. They stay separate so that
K5 capture-on-the-move can call `snapshot` without `goto`, and bisg can take `goto` without `snapshot`.

## 3. Interfaces Kinetix provides

| Name (under `/drone_<n>/`) | Kind / type | Server → client | Rate | QoS |
|---|---|---|---|---|
| `kinetix/goto` | action `kinetix_msgs/Goto` | goto (T2/T3) or render_server (T1) → session | 1 goal per view | action defaults (reliable) |
| `kinetix/snapshot` | service `kinetix_msgs/Snapshot` | snapshot (T2/T3) or render_server (T1) → session | 1 per view | service defaults (reliable) |
| `kinetix/status` | `diagnostic_msgs/DiagnosticStatus` | session | on change, ≤ 1 Hz | reliable, transient-local, depth 1 |
| `kinetix/viz` | `visualization_msgs/MarkerArray` | session | per step, **only if subscribed** | reliable, transient-local, depth 1 |
| `kinetix/recon/points` | `sensor_msgs/PointCloud2` (sparse SfM, `map`) | session | per step, **only if subscribed** | reliable, transient-local, depth 1 |

- `status`: `level` OK/WARN/ERROR, `message` = phase (`seed | plan | goto | snap | recon | done | aborted`),
  `values` = `frame_idx`, `budget`, `registered`, `points`, `last_error`. A late `ros2 topic echo` still sees the latest state.
- `viz`: candidate frusta coloured by gain, the chosen view, the captured views. The run directory stays the record.
  The viz topics only show progress and are not logged.

### Definitions (`kinetix_msgs`)

```
# action/Goto.action  — put frame `frame` at `target`, hold, report when settled
geometry_msgs/PoseStamped target   # header.frame_id = map (ENU)
string frame                       # frame placed at target: "base_link" or a camera optical frame (converted via tf_static)
float32 tol_pos_m                  # 0 = server default (0.10)
float32 tol_yaw_rad                # 0 = server default (3°)
float32 settle_s                   # 0 = server default (1.0)
float32 timeout_s                  # 0 = server default (60, sim time in sim)
---
uint8 REACHED=0
uint8 UNREACHABLE=1                # rejected: outside geofence, not armed / not OFFBOARD
uint8 TIMEOUT=2
uint8 ABORTED=3                    # cancel, mode change by operator / RC
uint8 code
geometry_msgs/PoseStamped reached  # `frame` pose when settled
float32 travel_m
---
float32 dist_m
float32 yaw_err_rad
float32 speed_mps
```

```
# srv/Snapshot.srv  — the first synchronised frame stamped at/after not_before
builtin_interfaces/Time not_before
float32 timeout_s
---
bool success
string error                       # "" | no_image | timeout | no_pose
sensor_msgs/Image left             # rgb8 (alpha dropped), rectified
sensor_msgs/CameraInfo left_info
sensor_msgs/Image right            # empty unless the node's `streams` has "right"
sensor_msgs/CameraInfo right_info
sensor_msgs/Image depth            # 32FC1 m; empty unless `streams` has "depth"
geometry_msgs/PoseStamped pose     # T_map_cam of left_info.header.frame_id at left.header.stamp
geometry_msgs/PoseStamped gt_pose  # sim only (`use_gt`); header.frame_id "" otherwise
```

`RosRig.capture_at(target)` = `goto(target, frame=<camera optical>)` → on REACHED `snapshot(not_before=now)` → `Frame`.
The session writes the images into the run directory. `RigError` kinds map 1:1 from `Goto.code` and `Snapshot.error`.

## 4. bisg topics consumed (strict subset of `bisg_isaac/docs/interface-contract.md`)

| Topic | Node | Subscription | Mode whitelist |
|---|---|---|---|
| `mavros/state` | goto | persistent | every mode |
| `mavros/local_position/odom` | goto, snapshot | persistent | every mode |
| `tf_static` | goto, snapshot | persistent (latched) | every mode |
| `zed/zed_node/left/color/rect/image` + `camera_info` | snapshot | **lazy**: only between request and reply | every mode |
| `zed/zed_node/right/color/rect/image` + `camera_info` | snapshot | lazy | `recon_input: stereo_rig*` |
| `zed/zed_node/depth/depth_registered` + `confidence/confidence_map` | snapshot | lazy | `plan_depth: stereo` |
| `state/pose` (Pegasus GT) | snapshot | persistent, sim only | `use_gt: true` |
| → `mavros/setpoint_position/local` (pub) | goto | 20 Hz while it holds the OFFBOARD role | every mode |

- **`local_position/odom` replaces `local_position/pose`.** One topic carries pose and twist, so the settle test
  (speed < 0.05 m/s) needs no second topic and no finite differences. To check on a live graph: the header/child frame ids MAVROS publishes.
- The **whitelist is the snapshot node's `streams` parameter** (`[left]`, `[left,right]`, `[left,right,depth]`), and the
  session derives it from the sensing mode. The whitelist test asserts the node's subscriptions for every preset.
  Never subscribed: `point_cloud/*`, `disparity/*`, `mapping/*`, `zed_node/odom`, `imu/*`.
- Stereo pairing: `message_filters` exact-time sync. The wrapper stamps left, right and depth from one grab.
- Pose at the image stamp: interpolate in a 2 s odom buffer, then compose with `T_base_cam` from `tf_static`.
  We do not use a `map → base_link` TF lookup because bisg's TF chain has the frame-prefix gap (bisg B16) and MAVROS TF output is a config choice.

## 5. Bandwidth budget (why lazy snapshots)

`docker/zed/zed.yaml` grabs **HD1080 @ 30 Hz** and the wrapper publishes `bgra8`. bisg runs CycloneDDS (no
shared memory transport, `MaxMessageSize 65500B`), so a large image between containers is fragmented over UDP loopback.

| Stream | Bytes / msg | Persistent subscription | Proposed |
|---|---|---|---|
| left `bgra8` 1920×1080 | 8.29 MB | 249 MB/s | 1 msg per view, sent as `rgb8` 6.22 MB |
| right `bgra8` | 8.29 MB | 249 MB/s | stereo modes only, 1 per view |
| depth `32FC1` | 8.29 MB | 249 MB/s | `plan_depth: stereo` only, 1 per view |
| `local_position/odom` | ≈ 0.7 KB | 21 KB/s (×2 nodes) | same |
| `mavros/state` | < 0.1 KB | ~1 KB/s | same |
| setpoint out | ≈ 0.1 KB | 2 KB/s | same |

Steady state: **≈ 50 KB/s** instead of 250–750 MB/s. A capture costs one 6.2 MB reply in `mono` and ≈ 20.7 MB in `stereo_full`.
At stop-and-shoot pace (one view per ~5–20 s), that averages at most ≈ 4 MB/s, even over Wi-Fi to a ground station.
Images stay **lossless** (`sensor_msgs/Image`, never JPEG `CompressedImage`) because SfM accuracy is the measured quantity.
PNG transport is the only allowed compression, and only if Wi-Fi proves too slow.

Lazy subscription costs reader discovery plus one frame period per snapshot (an estimate, not a measurement). The settle
time is 1 s, so this should not lengthen a capture. If it does (§8), subscribe while `goto` reports `dist_m < 2·tol_pos_m`.

## 5a. Zero-copy: when, and which mechanism

Lazy snapshots already remove the 30 Hz stream. What is left is **one 6–21 MB copy per view**, a few tens of ms (estimate)
against a 1 s settle and seconds of SfM per step. Zero-copy is not needed for stop-and-shoot (v1). It becomes worth doing for
K5 capture-on-the-move (a continuous 15–30 Hz stream into the selector) or if §8 item 2 measures a slow snapshot.

| Mechanism | Works here? | Why |
|---|---|---|
| **rclcpp intra-process, in the ZED container** | **yes, the chosen upgrade path** | The wrapper is already a component in `/drone_<n>/zed/zed_container`, and its stock launch sets `enable_ipc:=true` by default (bisg does not override it, to be confirmed at runtime). In IPC mode it publishes images through an `sl::Mat` ↔ `Image` TypeAdapter, so a C++ component in the same container receives frames without serialisation or a copy. The `snapshot` node becomes such a component and only the selected frame leaves the process. |
| rclpy (any Python node) | no | rclpy has no intra-process path and no loaned messages. Every Python subscriber pays a full deserialise. |
| CycloneDDS + iceoryx / PSMX loans | no | Loans need fixed-size types, and `sensor_msgs/Image` is unbounded. It also needs a RouDi daemon and a bisg RMW config change. |
| Fast DDS data-sharing / SHM transport | partly | Zero-copy data-sharing needs bounded types, so it does not apply to `Image`. Plain SHM transport (one copy, no UDP fragmentation) would work, but bisg chose Cyclone (`ZED_RMW`), so this is bisg's call, not Kinetix's. |
| NITROS (Isaac ROS, GPU) | not now | The wrapper can publish NITROS images (it disables IPC when it does). That is useful only if a GPU consumer runs in the same graph on the Jetson. Kinetix's depth and SfM run in Python/PyTorch, outside it. |
| `rmw_zenoh` SHM | not now | Experimental in Jazzy, and it would change bisg's RMW. |

Upgrade path, with no interface change: reimplement `snapshot` as a C++ component (`kinetix_snapshot::Snapshot`) and load it into
the ZED container next to the wrapper. `Snapshot.srv` stays the same, so `session` and `RosRig` do not notice. In the same move the component
could write the lossless PNG straight into the run directory and reply with its path when session and camera share a disk
(sim, or session on the Jetson). Then not even the selected frame crosses DDS. The C++ component is generic, so it belongs in
bisg (its container, its launch), added under bisg's rules. This is the only place Kinetix would need C++, and only if measured to matter.

## 6. QoS

| Endpoint | QoS | Why |
|---|---|---|
| ZED image / camera_info sub | `SensorDataQoS` (best effort, keep last 1) | compatible with a reliable or best-effort publisher, and a stale frame is never wanted. A dropped fragment costs only one frame period, because the node waits for the next stamp |
| `mavros/*` subs and setpoint pub | MAVROS' own profile (bisg contract §QoS: match the publisher) | read with `ros2 topic info -v` at bringup and do not guess |
| `state/pose`, `tf_static` | publisher's profile; `tf_static` transient-local | standard |
| `kinetix/status`, `viz`, `recon/points` | reliable, transient-local, depth 1 | late joiners see the latest state |
| action / service | rclpy defaults | reliable request/response |

The kinetix container uses bisg's RMW and DDS config: `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, the same
`CYCLONEDDS_URI`, `ROS_DOMAIN_ID` and `network_mode: host`, `ipc: host`. If its middleware differs, discovery or large messages fail without an error.

## 7. Executors and callback groups (rclpy)

- **goto**: no blocking. A 20 Hz timer runs the state machine (`idle-hold → tracking → settling → reached`). The action
  `execute_callback` waits on an `Event` that the timer sets. The action and the timer/subscriptions are in separate callback groups, run by a
  `MultiThreadedExecutor` with ≥ 2 threads. When idle it streams a hold setpoint at the current pose, which OFFBOARD requires before the mode switch.
- **snapshot**: the service callback runs in its own `MutuallyExclusiveCallbackGroup`, and the image/odom subscriptions in another.
  It waits on a `threading.Event` with `timeout_s`. `MultiThreadedExecutor` with ≥ 2 threads.
- **session**: the executor spins in a background thread. The session loop (SfM takes seconds) runs on the main thread and
  calls `send_goal_async` / `call_async` and waits on the futures with a timeout. Do not block in a ROS callback.
- **Safety** (T2/T3): `goto` never arms. In sim the session arms and sets OFFBOARD. On hardware the operator does it from RC/QGC.
  An operator mode change aborts the active goal (`ABORTED`). The independent stop path is RC plus PX4's offboard-loss failsafe
  (`COM_OF_LOSS_T`), not this code.

## 8. Changes against interfaces.md, and open measurements

Changes once accepted:
- §4: `local_position/pose` → `local_position/odom`. The session no longer subscribes to sensors directly: `snapshot` does, lazily.
  Setpoint streaming moves from `MavrosRig` into the `goto` node.
- §5: the T1 render topics (`goal` / `image` / `camera_info` / `depth`, request id in the stamp) become the same
  `kinetix/goto` + `kinetix/snapshot` (teleport, render `settle_frames`, reply).
- architecture.md §4: `rig/isaac_render.py` + `rig/mavros.py` → `rig/ros.py` (`RosRig`) + `ros/` nodes `goto.py`,
  `snapshot.py` + the `kinetix_msgs` package. The dependency rule is unchanged: only `rig/ros.py` and `ros/` import `rclpy`.

To measure at K4 (L2/L3 on the sim graph; none of this is verified yet):
1. `ros2 topic info -v` on the ZED image topics and `mavros/local_position/odom`: the offered QoS and the frame ids.
2. Lazy-snapshot latency (request → reply) over 50 captures with CycloneDDS. If p95 > 0.3 s, use the pre-arm variant (§5).
3. Loss rate of a 6 MB `Snapshot` reply and of best-effort 8 MB frames over loopback with `rmem_max` = 16 MB (bisg `docs/setup.md`).
4. The whitelist test: for each sensing preset, the subscriptions the snapshot node actually creates.
5. On hardware: snapshot reply time over the planned Wi-Fi link, if the session runs on a ground station rather than on the Jetson.

## 9. Why ROS 2, and the alternatives considered

ROS 2 is used **only where the platform already speaks it**: T1, T2 and T3. T0, where most research and benchmark runs happen, has no ROS.

| Alternative | Verdict | Reason |
|---|---|---|
| ROS 2 at the edges (this design) | **chosen** | bisg's contract, MAVROS (bisg ADR-001) and the ZED wrapper are ROS 2 on both sim and hardware. Speaking it directly means no bridge, and sim and drone behave identically. |
| MAVSDK / pymavlink straight to PX4 | no | MAVROS already owns the FCU link. A second MAVLink client needs another port, duplicates the bridge bisg already studied and rejected as default, and splits the sim-vs-hardware parity. |
| `pyzed` straight to the camera | no | Only one process can open the ZED, and the wrapper holds it (in sim the SDK connects once per run, bisg B18). |
| Dora-rs, ZeroMQ, gRPC, LCM (own dataflow) | no | Fast, with zero-copy Arrow in Dora, but every input (MAVROS, ZED) still arrives as ROS 2, so it adds a bridge per topic. The data rate is one frame set per view, so their speed buys nothing. |
| Non-ROS RPC for T1 only (render server) | possible, not chosen | T1 needs no ROS. Serving the same `goto`/`snapshot` as T2 makes one client cover T1–T3, at the cost of one render-sized message per request. |
| `rmw_zenoh` / `zenoh-bridge-ros2dds` | keep for T3 | If the session runs on a ground station, DDS discovery over Wi-Fi is the classic failure. Bridging only the `kinetix/*` interfaces with Zenoh is the fallback, and it does not touch bisg's RMW. Decide after §8 item 5. |

The real throughput limits are elsewhere: SfM re-runs each step (`recon/state.py` v2, K5), the sim at 0.3–0.5× real time (bisg M8)
and flight plus settle time per view. Transport choice should not move ahead of those.

## 10. ZED SDK modules worth using (beyond images, depth and odometry)

Checked against the wrapper bisg runs (`docker/zed/zed.yaml`, wrapper services under `zed/zed_node/`). The rule stays
the same: ZED point cloud, mapping and odometry are never **method inputs** (ADR-K07). The items below are tooling for experiments and setup,
or they feed a component that already exists.

| Module / service | Use in Kinetix | Fit |
|---|---|---|
| **SVO recording** (`start_svo_rec` / `stop_svo_rec`, `set_svo_frame`) | Record every T2/T3 session (and K6 dense captures) as SVO. Replaying it through the full SDK gives depth in any mode (NEURAL vs NEURAL_PLUS), tracking and frame-exact re-extraction offline. The real-data half of the depth study and real pools then come from one recording, and failed runs can be re-analysed without re-flying. **Available:** `./bisg zed record start [NAME] [lossless|h264|h265]` / `record stop` (→ `$ARCHIVE_DIR/svo/`); verified in the sim 2026-10-10, where the `.svo2` replays offline through pyzed with depth recomputed. | **adopt** (K4 for T2, K6 for T3) |
| **Depth confidence** (`depth.publish_depth_confidence`) | `DepthSource.depth()` must return a 0..1 confidence. `stereo_zed` reads `confidence/confidence_map` instead of inventing one: confidence = 1 − c/100 (0–100, **higher = less confident**; depth above `depth_confidence` = 95 is already NaN; checked live 2026-10-10). Lazy-subscribed by `snapshot` only in `plan_depth: stereo`. **Available:** advertised by default in bisg's zed.yaml, with a contract row. | **adopt** (K4) |
| **Area memory** (`area_memory`, `save_area_memory`, `area_file_path`) | Relocalise each real flight in the same `.area` map of the lab. Every T3 run of an object then shares one frame, so the ROI box and the no-fly boxes are defined once and runs are directly comparable. This is a bisg navigation setting (zed.yaml) used on the hardware day. | **adopt** for T3 (K6) |
| **Floor alignment** (`positional_tracking.floor_alignment`) | The world z origin is set on the floor, so `scene.yaml` altitude limits mean "above the floor" on hardware as in sim. | try at K6 |
| **Custom object detection** (`custom_onnx_file`, YOLO ONNX; `enable_obj_det`) | Bootstrap the ROI box of a real object (3D box with tracking) before the seed views, instead of measuring it by hand. It is setup only: it runs once, the result is written into `scene.yaml` and reported, and it is never called during planning. | optional (K6) |
| Plane detection (`plane` at a clicked point) | The table-top plane under the object gives the ROI's bottom face. Needs an operator click, so it is manual setup only. | optional |
| Spatial mapping (`enable_mapping`) | Already the `zed_mapping` baseline (a separate script). Not an input. | baseline only |
| Streaming (`enable_streaming`, H.264/H.265) | Would run the SDK and Kinetix on the workstation during T3. Rejected for capture: the codec is lossy and SfM accuracy is the measured quantity. Use SVO (lossless option) instead. | no |
| Body tracking / people detection | Pause the session if a person enters the workspace (indoor flights). This is a safety convenience, not part of the method. | later (K6, if wanted) |
| Region of interest (`set_roi`) | Masks props and legs. With the forward mount at x = 0.18 m they are out of view. | not needed |

**SDK depth on pools (Q6), tested 2026-10-10 → ADR-K10.** bisg's probe (`sim/tools/zed_pool_probe.py`, results in bisg
`docs/zed-sdk-sim.md`) teleported a kinematic ZED_M twin through 28 warehouse views while streaming it to the real wrapper. The SDK accepts
every jump. After each teleport it delivers 2 stale frames (~0.1 s), then settles in 0.22 s median (0.54 s worst). Median relative error is 0.35 % at 0.3–2 m,
0.65 % at 2–5 m, 1.8 % at 5–10 m and 3.9 % at 10–15 m; > 99.7 % valid pixels below 10 m; revisits agree within 0.5–0.8 %. At 0.7 s per view,
a 600-view pool takes ~7 min. So T1 batch pools carry real SDK depth + confidence (`--zed-depth`), and FoundationStereo becomes the fallback and an
ablation. Caveats: sim stereo is clean (no noise, blur or calibration error), so this is an upper bound on real ZED accuracy. The real half of the
depth study still comes from SVO recordings.
