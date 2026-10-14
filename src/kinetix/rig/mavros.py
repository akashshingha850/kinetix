"""MavrosRig (T2 sim flight / T3 real): flies the camera to each target through MAVROS and grabs one ZED left image.

Topics are the `mono` whitelist of interfaces.md §4 (bisg contract, namespace `/drone_<n>`): mavros/state,
mavros/local_position/pose, zed/zed_node/left/color/rect/{image,camera_info}, tf_static (T_base_cam), state/pose (sim GT,
logged only); publishes mavros/setpoint_position/local (20 Hz wall while OFFBOARD); calls mavros/set_mode, mavros/cmd/arming.
The left image is subscribed only for the grab (lazy), so the wrapper does no image work between captures.

capture_at: stream the body setpoint that puts the camera at the target -> wait until position error < tol_pos, yaw error
< tol_yaw and speed < tol_speed for settle_s (sim time) -> take the first image RECEIVED after that (the ZED wrapper stamps
with the wall clock in the sim, so a stamp comparison against sim time is meaningless) -> pose = local_position/pose at
reception ∘ T_base_cam.
"""
from __future__ import annotations

import threading
import time
from collections import deque
from pathlib import Path

import numpy as np

from kinetix.core.geometry import (R_LINK_OPT, R_to_quat, body_from_camera, camera_from_body, camera_yaw, quat_to_R,
                                   rot_y, rot_z, wrap)
from kinetix.core.types import Frame, Intrinsics, Pose, RigCaps, RigError


class MavrosRig:
    def __init__(self, cfg: dict, scene, run_dir):
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from mavros_msgs.msg import State
        from mavros_msgs.srv import CommandBool, SetMode
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.parameter import Parameter
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import CameraInfo, Image
        from tf2_ros import Buffer, TransformListener

        self.cfg, self.rc, self.scene, self.run_dir = cfg, cfg["rig"], scene, Path(run_dir)
        cam = cfg["camera"]
        self.mount_pitch = np.radians(cam["mount_pitch_deg"])
        self.caps = RigCaps(continuous=True, has_gt=True, mount_pitch_rad=self.mount_pitch, speed_mps=self.rc["speed_mps"])
        self.t_base_cam = np.asarray(cam["t_base_cam"], dtype=float)   # replaced by TF in start() when available
        self.ns = self.rc["ns"].rstrip("/")
        self._Image, self._PoseStamped = Image, PoseStamped

        if not rclpy.ok():
            rclpy.init()
        self._rclpy = rclpy
        self.node = rclpy.create_node("kinetix_rig", parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        be10 = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.lock = threading.Lock()
        self.state = None
        self.body = None                        # (p, q) latest local_position/pose, map ENU
        self.gt = None                          # (p, q) latest Pegasus state/pose, world ENU
        self.K = None
        self.track = deque(maxlen=200)          # (t_sim, p) for the speed estimate
        self.odo = 0.0                          # metres flown (sum of pose increments)
        n = self.node
        n.create_subscription(State, f"{self.ns}/mavros/state", self._on_state, be10)
        n.create_subscription(PoseStamped, f"{self.ns}/mavros/local_position/pose", self._on_pose, be10)
        n.create_subscription(PoseStamped, f"{self.ns}/state/pose", self._on_gt, be10)
        n.create_subscription(CameraInfo, f"{self.ns}/zed/zed_node/left/color/rect/camera_info", self._on_info, be10)
        self.sp_pub = n.create_publisher(PoseStamped, f"{self.ns}/mavros/setpoint_position/local", 10)   # RELIABLE
        self.cli_mode = n.create_client(SetMode, f"{self.ns}/mavros/set_mode")
        self.cli_arm = n.create_client(CommandBool, f"{self.ns}/mavros/cmd/arming")
        self._SetMode, self._CommandBool = SetMode, CommandBool
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, n)

        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(n)
        self._spin = threading.Thread(target=self.executor.spin, daemon=True)
        self._spin.start()
        self.sp = None                          # (p, yaw) body setpoint being streamed
        self._stream_on = True
        self._stream = threading.Thread(target=self._stream_loop, daemon=True)
        self._stream.start()
        self.home = None
        self.started = False

    # --- callbacks ----------------------------------------------------------------------------------------------------
    def _on_state(self, m):
        self.state = m

    def _on_pose(self, m):
        p = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])
        q = np.array([m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w])
        with self.lock:
            if self.body is not None:
                self.odo += float(np.linalg.norm(p - self.body[0]))
            self.body = (p, q)
            self.track.append((self.now(), p))

    def _on_gt(self, m):
        self.gt = (np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z]),
                   np.array([m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w]))

    def _on_info(self, m):
        if m.width:
            self.K = Intrinsics(int(m.width), int(m.height), float(m.k[0]), float(m.k[4]), float(m.k[2]), float(m.k[5]))

    # --- helpers --------------------------------------------------------------------------------------------------------
    def now(self) -> float:
        return self.node.get_clock().now().nanoseconds * 1e-9

    def _stream_loop(self):
        period = 1.0 / float(self.rc["setpoint_hz"])
        while self._stream_on:
            sp = self.sp
            if sp is not None:
                m = self._PoseStamped()
                m.header.stamp = self.node.get_clock().now().to_msg()
                m.header.frame_id = "map"
                m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in sp[0])
                qx, qy, qz, qw = R_to_quat(rot_z(sp[1]))
                m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = qx, qy, qz, qw
                self.sp_pub.publish(m)
            time.sleep(period)

    def _wait(self, cond, timeout_sim: float, what: str, kind: str = "timeout"):
        """Poll `cond` until true; the timeout is in sim seconds, with a wall-clock backstop for a stalled /clock."""
        t_end = self.now() + timeout_sim
        wall_end = time.time() + timeout_sim / float(self.rc["min_rtf"])
        while not cond():
            if self.now() > t_end or time.time() > wall_end:
                raise RigError(kind, f"{what} after {timeout_sim:.0f} s sim")
            time.sleep(0.05)

    def _call(self, client, req, what: str):
        if not client.wait_for_service(timeout_sec=10.0):
            raise RigError("aborted", f"{what}: service {client.srv_name} not available")
        fut = client.call_async(req)
        t_end = time.time() + 10.0
        while not fut.done():
            if time.time() > t_end:
                raise RigError("aborted", f"{what}: no reply")
            time.sleep(0.02)
        return fut.result()

    def _set_mode(self, mode: str):
        req = self._SetMode.Request()
        req.custom_mode = mode
        return self._call(self.cli_mode, req, f"set_mode {mode}")

    def _arm(self, value: bool):
        req = self._CommandBool.Request()
        req.value = value
        return self._call(self.cli_arm, req, "arming")

    def _body_yaw(self) -> float:
        return float(np.arctan2(*quat_to_R(self.body[1])[[1, 0], 0]))

    def _speed(self) -> float:
        with self.lock:
            tr = list(self.track)
        if len(tr) < 2:
            return float("inf")
        t1, p1 = tr[-1]
        old = [(t, p) for t, p in tr if t1 - t <= 0.5]
        t0, p0 = old[0]
        return float(np.linalg.norm(p1 - p0) / (t1 - t0)) if t1 - t0 > 0.1 else float("inf")

    def _goto_body(self, p, yaw: float, timeout_sim: float, settle_s: float | None = None):
        """Stream a body setpoint and block until settled there."""
        self.sp = (np.asarray(p, dtype=float), float(yaw))
        settle = self.rc["settle_s"] if settle_s is None else settle_s
        tol_p, tol_y, tol_v = self.rc["tol_pos_m"], np.radians(self.rc["tol_yaw_deg"]), self.rc["tol_speed_mps"]
        since = [None]

        def settled():
            self._check_alive()
            with self.lock:
                bp = self.body[0]
            ok = (np.linalg.norm(bp - self.sp[0]) < tol_p and abs(wrap(self._body_yaw() - self.sp[1])) < tol_y
                  and self._speed() < tol_v)
            if not ok:
                since[0] = None
                return False
            since[0] = since[0] if since[0] is not None else self.now()
            return self.now() - since[0] >= settle
        self._wait(settled, timeout_sim, f"not settled at {np.round(p, 2).tolist()}")

    def _check_alive(self):
        if self.started and self.state is not None and (not self.state.armed or self.state.mode != "OFFBOARD"):
            raise RigError("aborted", f"vehicle left OFFBOARD (mode {self.state.mode}, armed {self.state.armed})")

    def _lookup_mount(self):
        """T_base_cam from TF (base_link -> zed_left_camera_frame_optical, bisg contract); config value if TF is missing."""
        frame = self.rc["camera_frame"]
        t_end = time.time() + 5.0
        while time.time() < t_end:
            try:
                tr = self.tf_buffer.lookup_transform("base_link", frame, self._rclpy.time.Time())
                t = tr.transform.translation
                r = tr.transform.rotation
                R = quat_to_R([r.x, r.y, r.z, r.w])
                if np.abs(R - rot_y(self.mount_pitch) @ R_LINK_OPT).max() > 0.05:
                    print(f"[mavros-rig] WARNING TF mount rotation differs from mount_pitch_deg: {np.round(R, 2).tolist()}")
                return np.array([t.x, t.y, t.z]), "tf"
            except Exception:  # noqa: BLE001 (tf2 raises several lookup exceptions)
                time.sleep(0.2)
        return self.t_base_cam, "config"

    # --- Rig protocol ---------------------------------------------------------------------------------------------------
    def start(self) -> None:
        self._wait(lambda: self.state is not None and self.state.connected, 30.0, "MAVROS not connected", "aborted")
        self._wait(lambda: self.body is not None, 30.0, "no mavros/local_position/pose", "aborted")
        self._wait(lambda: self.K is not None, 60.0, "no ZED camera_info (is `./bisg zed up` running?)", "aborted")
        self.t_base_cam, src = self._lookup_mount()
        print(f"[mavros-rig] T_base_cam from {src}: {np.round(self.t_base_cam, 4).tolist()}; K fx={self.K.fx:.1f} "
              f"{self.K.width}x{self.K.height}", flush=True)
        with self.lock:
            self.home = self.body[0].copy()
        yaw0 = self._body_yaw()
        self.sp = (self.home.copy(), yaw0)
        time.sleep(1.5)                                      # PX4 needs a setpoint stream before OFFBOARD
        for _ in range(10):
            if self.state.mode != "OFFBOARD":
                self._set_mode("OFFBOARD")
            if not self.state.armed:
                self._arm(True)
            time.sleep(1.0)
            if self.state.armed and self.state.mode == "OFFBOARD":
                break
        else:
            raise RigError("aborted", f"could not arm/OFFBOARD (mode {self.state.mode}, armed {self.state.armed})")
        self.started = True
        # climb straight up, then go to the standby camera pose
        sb = self.scene.standby
        yaw_sb = float(np.arctan2(*(self.scene.roi.center - sb)[[1, 0]]))
        p_sb = body_from_camera(sb, yaw_sb, self.t_base_cam)
        self._goto_body([self.home[0], self.home[1], p_sb[2]], yaw0, self.rc["move_timeout_s"], settle_s=0.5)
        self._goto_body(p_sb, yaw_sb, self.rc["move_timeout_s"], settle_s=0.5)

    def pose(self) -> Pose:
        with self.lock:
            p, q = self.body
        return camera_from_body(p, q, self.t_base_cam, self.mount_pitch)

    def capture_at(self, target: Pose, idx: int, pool_id: str | None = None) -> Frame:
        yaw = camera_yaw(target)
        with self.lock:
            odo0 = self.odo
        try:
            self._goto_body(body_from_camera(target.p, yaw, self.t_base_cam), yaw, self.rc["move_timeout_s"])
        except RigError as e:
            if e.kind == "timeout":                          # hold where we are; the loop drops this candidate
                with self.lock:
                    self.sp = (self.body[0].copy(), self._body_yaw())
            raise
        t_settled = self.now()
        got = {}

        def on_img(m):
            if "msg" not in got and self.now() >= t_settled:
                with self.lock:
                    got["body"] = (self.body[0].copy(), self.body[1].copy())
                got["gt"] = self.gt
                got["t"] = self.now()
                got["msg"] = m
        sub = self.node.create_subscription(self._Image, f"{self.ns}/zed/zed_node/left/color/rect/image", on_img,
                                            self.sensor_qos)
        try:
            self._wait(lambda: "msg" in got, self.rc["image_timeout_s"], "no left image", "no_image")
        finally:
            self.node.destroy_subscription(sub)
        img = to_bgr(got["msg"])
        rel = Path("frames") / f"{idx:06d}.png"
        import cv2
        cv2.imwrite(str(self.run_dir / rel), img)
        pose = camera_from_body(*got["body"], self.t_base_cam, self.mount_pitch)
        gt_pose = None
        if got["gt"] is not None:
            gp, gq = got["gt"]
            gt_pose = camera_from_body(gp + self.scene.world_to_map, gq, self.t_base_cam, self.mount_pitch)
        with self.lock:
            travel = self.odo - odo0
        return Frame(idx=idx, t=got["t"], image=rel, K=self.K, target=target, pose=pose, gt_pose=gt_pose,
                     travel_m=travel, pool_id=pool_id)

    def finish(self) -> None:
        """Climb above the ROI clearance, fly back over home, land, disarm."""
        try:
            if self.started and self.state.armed and self.state.mode == "OFFBOARD":
                with self.lock:
                    p = self.body[0].copy()
                yaw = self._body_yaw()
                z_safe = max(p[2], self.scene.roi.hi[2] + float(self.cfg["planner"]["roi_clearance_m"]) + 0.3)
                self._goto_body([p[0], p[1], z_safe], yaw, self.rc["move_timeout_s"], settle_s=0.3)
                self._goto_body([self.home[0], self.home[1], z_safe], yaw, self.rc["move_timeout_s"], settle_s=0.3)
                self.started = False
                self._set_mode("AUTO.LAND")
                self._wait(lambda: not self.state.armed, 120.0, "still armed after landing")
                print("[mavros-rig] landed and disarmed", flush=True)
        except RigError as e:
            print(f"[mavros-rig] finish: {e}", flush=True)
        finally:
            self._stream_on = False
            self.sp = None
            self.executor.shutdown()
            self.node.destroy_node()
            if self._rclpy.ok():
                self._rclpy.shutdown()


def to_bgr(msg) -> np.ndarray:
    """sensor_msgs/Image (bgra8 | bgr8 | rgba8 | rgb8) -> HxWx3 uint8 BGR, without cv_bridge."""
    import cv2
    ch = {"bgra8": 4, "rgba8": 4, "bgr8": 3, "rgb8": 3}.get(msg.encoding)
    if ch is None:
        raise RigError("no_image", f"unsupported encoding {msg.encoding}")
    a = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.step)[:, : msg.width * ch]
    a = a.reshape(msg.height, msg.width, ch)
    conv = {"bgra8": cv2.COLOR_BGRA2BGR, "rgba8": cv2.COLOR_RGBA2BGR, "rgb8": cv2.COLOR_RGB2BGR}.get(msg.encoding)
    return cv2.cvtColor(a, conv) if conv is not None else a.copy()
