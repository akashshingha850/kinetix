# Interfaces

The contracts code is written against. Change this file first, then the code, then the tests that
pin it ([testing.md](testing.md)). Architecture context: [architecture.md](architecture.md).

## 1. Conventions (one place, used everywhere)

- Units: SI (m, s, rad internally; degrees only in config/YAML).
- World frame: **`map`**, ENU (x east, y north, z up), the same `map` as the bisg contract (EKF2 origin).
  Scene files, pools, planner, run logs and evaluation all use it.
- Camera frame: **OpenCV optical** (x right, y down, z forward).
- A pose is `T_map_cam`: camera → map. Stored as `{"p": [x, y, z], "q": [qx, qy, qz, qw]}`.
- COLMAP stores world → camera (`cam_from_world`). The conversion happens **only** in
  `recon/` and `offline/sfm.py` via `core.geometry`, nowhere else.
- Drone vs camera: the rig converts a camera target into a body setpoint using the mount transform
  `T_base_cam` (read from TF on T2/T3; from config on T0/T1). The planner only ever reasons about
  camera poses.
- Time: float seconds. In sim this is sim time (`use_sim_time`).

## 2. Core types (`core/types.py`)

```python
@dataclass(frozen=True)
class Pose:            # T_map_cam
    p: np.ndarray      # (3,)
    q: np.ndarray      # (4,) xyzw, unit

@dataclass(frozen=True)
class Intrinsics:
    width: int; height: int
    fx: float; fy: float; cx: float; cy: float
    dist: tuple[float, ...] = ()   # empty = rectified (the ZED rect image is)

@dataclass(frozen=True)
class Frame:
    idx: int                       # capture order, 0-based
    t: float
    image: Path                    # written by the rig into the run dir
    K: Intrinsics
    target: Pose                   # what the planner asked for
    pose: Pose                     # what the rig reports (pool pose / EKF2 ∘ mount)
    gt_pose: Pose | None           # sim only; never read by planner/recon (except pose_source=gt)
    travel_m: float                # path length flown since the previous frame
    pool_id: str | None = None     # T0: id of the pool view
    right_image: Path | None = None  # only when the sensing mode needs it (stereo_rig*, stereo depth on T0/T1)
    depth: Path | None = None        # float32 m, left optical frame; set by the DepthSource, not the rig
    depth_source: str | None = None  # mono | stereo_learned | stereo_zed | gt

@dataclass(frozen=True)
class Candidate:
    pose: Pose
    pool_id: str | None = None

@dataclass(frozen=True)
class Box:  center: np.ndarray; size: np.ndarray; yaw: float = 0.0   # in map

@dataclass(frozen=True)
class Scene:
    name: str
    roi: Box; workspace: Box; no_fly: tuple[Box, ...]
    standoff: tuple[float, float]; altitude: tuple[float, float]
    gt_mesh: Path | None           # None on real scenes without a reference
```

## 3. Component protocols

Selected by name from config through a registry (`config.py`: `@register("scorer", "combo")`).

```python
class Rig(Protocol):
    caps: RigCaps                  # continuous: bool, has_gt: bool, pool: list[Candidate] | None,
                                   # mount_pitch_rad: float, speed_mps: float,
                                   # stereo: bool, baseline_m: float | None, sensor_depth: bool (ZED SDK topic)
    def start(self) -> None: ...
    def pose(self) -> Pose: ...
    def capture_at(self, target: Pose) -> Frame: ...      # move, settle, grab; raises RigError
    def finish(self) -> None: ...

class Sampler(Protocol):
    def sample(self, scene: Scene, recon: "ReconState", caps: RigCaps) -> list[Candidate]: ...

class Constraint(Protocol):
    def feasible(self, cands: list[Candidate], current: Pose, scene: Scene) -> np.ndarray: ...  # bool mask

class Scorer(Protocol):
    uses_gt: bool                  # True → result is an oracle/upper bound, flagged in run.yaml
    def score(self, cands: list[Candidate], recon: "ReconState") -> np.ndarray: ...  # float, ≥ 0

class DepthSource(Protocol):     # selected by sensing.plan_depth (+ tier); architecture.md §2a
    name: str                      # mono | mono_metric_noanchor | mono_stereo_anchor | stereo_learned | stereo_zed | gt
    uses_gt: bool
    def depth(self, frame: Frame, recon: "ReconState") -> tuple[np.ndarray, np.ndarray]: ...
                                   # (H, W) metres, (H, W) confidence 0..1; NaN = no depth

class Stop(Protocol):
    def done(self, recon: "ReconState", budget: "Budget") -> bool: ...

class Densifier(Protocol):
    def densify(self, run_dir: Path, sfm_dir: Path, out_dir: Path) -> Path: ...  # returns mesh/cloud .ply
```

`ReconState` (concrete class, not a protocol):

```python
class ReconState:
    def add(self, frame: Frame) -> None
    @property registered -> list[int]                 # frame idx registered in the model
    @property points -> np.ndarray                     # (N, 3) in map (after Sim3 alignment)
    @property point_stats -> dict[str, np.ndarray]     # track_len, tri_angle_deg, reproj_err, observing frame idx
    def cam_pose(self, idx: int) -> Pose | None        # SfM pose in map
    def project(self, pose: Pose, K: Intrinsics) -> np.ndarray   # visibility of points from a candidate
    def snapshot(self, out_dir: Path) -> None          # COLMAP binary model
```

Errors: rigs raise `RigError(kind=...)` with kinds `unreachable | timeout | no_image | aborted`.
The loop logs the error into `steps.jsonl`, marks the candidate infeasible for the rest of the
session and continues. `aborted` (RC loss, operator) ends the session cleanly.

## 4. ROS 2 interface (T2/T3: `rig/mavros.py`)

A strict subset of the bisg vehicle interface contract (`bisg_isaac/docs/interface-contract.md`),
for drone `n` (default 1), namespace `/drone_<n>`:

| Direction | Name | Type | Use |
|---|---|---|---|
| sub | `mavros/state` | `mavros_msgs/State` | connected / armed / mode |
| sub | `mavros/local_position/pose` | `geometry_msgs/PoseStamped` | vehicle pose in `map` → `Frame.pose` (∘ mount) |
| sub | `zed/zed_node/left/color/rect/image` | `sensor_msgs/Image` | `Frame.image` (every mode) |
| sub | `zed/zed_node/left/color/rect/camera_info` | `sensor_msgs/CameraInfo` | `Frame.K` (never hard-code K) |
| sub (stereo_rig*) | `zed/zed_node/right/color/rect/image` + `right/.../camera_info` | `Image`, `CameraInfo` | `Frame.right_image`, baseline `-P[0,3]/P[0,0]` |
| sub (plan_depth=stereo) | `zed/zed_node/depth/depth_registered` | `Image` 32FC1 m | `stereo_zed` depth source |
| sub | `tf_static` | | `T_base_cam` (base_link → zed_left_camera_optical_frame) |
| sub (sim, eval only) | `state/pose` | `geometry_msgs/PoseStamped` | Pegasus ground truth → `Frame.gt_pose` |
| pub | `mavros/setpoint_position/local` | `geometry_msgs/PoseStamped` | 20 Hz while OFFBOARD |
| srv | `mavros/cmd/arming`, `mavros/set_mode` | | arm, OFFBOARD |

**Topic whitelist per sensing mode** (ADR-K07, checked by a unit test): the rig subscribes to the
rows marked "every mode", plus the stereo rows only when `sensing.recon_input` / `sensing.plan_depth`
asks for them. So in `mono` the right image and depth are not subscribed at all. Never subscribed in any
mode: `point_cloud/*`, `disparity/*`, `mapping/*`, `zed_node/odom`. The `zed_mapping` baseline is a
separate script that records `mapping/fused_cloud`, not a `MavrosRig` mode.

Image pairing: in stereo modes left, right and depth must have the **same stamp** (the wrapper
publishes them from one grab). The rig waits for a matching triple after settle.

`capture_at` sequence: stream the setpoint → wait until the position error is below `tol_pos` (0.10 m),
yaw error below `tol_yaw` (3°) and speed below 0.05 m/s for `settle_s` (1.0 s) → take the **first image
stamped after settle** → pose = `local_position/pose` interpolated at the image stamp ∘ `T_base_cam`.
Timeout `move_timeout_s` (60 s sim time) → `RigError(timeout)`. QoS: sensor data for images,
matching MAVROS for state. `use_sim_time:=true` in sim.

The bisg contract names `bisg_vehicle/offboard_controller` as the setpoint producer, but that package
is not built yet. Until it is, `MavrosRig` streams setpoints itself. Only one setpoint producer may
run per drone. When bisg ships a `vehicle/cmd` goto, `MavrosRig` switches to it (decisions.md ADR-K01).

## 5. T1 render server (`sim/render_server.py` ↔ `rig/isaac_render.py`)

Runs inside `bisg/sim:6.0.0` with a Kinetix scene USD. No drone, no PX4: a free camera with the
ZED Mini intrinsics from `pool.yaml` / config.

- **batch mode** (`--views views.jsonl --out <pool_dir>`): renders left RGB, **right RGB** (left pose ∘
  `[baseline, 0, 0]` in the optical frame) and GT depth (+ optional instance mask) for every pose, then writes a pool ([data-format.md](data-format.md) §2). This is
  the first deliverable (K1).
- **serve mode** (K4): request/response over ROS 2 under `/kinetix/render/`:
  - `goal` `geometry_msgs/PoseStamped` (frame `map`, `T_map_cam`, header.stamp = request id)
  - `image` `sensor_msgs/Image`, `camera_info`, `depth` (`32FC1`, m): reply stamped with the request id
  - Renders `settle_frames` (default 3) after the teleport before replying, so RTX accumulation and
    auto-exposure have settled.

## 6. CLI (`kinetix`, via `uv run kinetix ...`)

```
kinetix check                                    # = scripts/check_env.py
kinetix pool make  --scene S --layout rings:4x60|tammes:128 [--test-split 0.1]   # views.jsonl → T1 batch render
kinetix run   --scene S --method M --sensing mono|stereo_plan|... --rig pool|isaac|mavros --seed K [key=value ...]  → runs/<id>/
kinetix depth-study --scene S --sources stereo_learned,mono,gt [--zed]  → docs/research/depth-study.md data
kinetix eval  runs/<id> [--densify gsplat|colmap_mvs]                  → runs/<id>/metrics.json
kinetix bench --matrix config/bench/<name>.yaml [--jobs N]             → bench/<name>/report.md
```

Exit code 0 = success; non-zero on any failed step, so the CLI is scriptable from tests and bench.
