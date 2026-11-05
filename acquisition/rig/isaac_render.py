"""IsaacRenderRig (T1, interfaces.md §5): every capture is rendered by sim/render_server.py at the exact target pose. No flight:
no EKF2 error, no settle time, no motion blur, so a run is the reference a flight of the same method is measured against.
Talks to the server over ROS 2 (/kinetix/render/*); start it with scripts/render_server.sh serve <scene>."""
from __future__ import annotations

import time
from pathlib import Path

import numpy as np

from kinetix.geometry import camera_pose
from kinetix.types import Frame, Intrinsics, Pose, RigCaps, RigError


class IsaacRenderRig:
    def __init__(self, cfg: dict, scene, run_dir):
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import Image
        rc, cam = cfg["rig"], cfg["camera"]
        self.rc, self.run_dir = rc, Path(run_dir) if run_dir else None
        self.caps = RigCaps(continuous=True, has_gt=True, mount_pitch_rad=np.radians(cam["mount_pitch_deg"]), speed_mps=rc["speed_mps"],
                            sensor_depth=True)
        self.K = Intrinsics(**{k: cam[k] for k in ("width", "height", "fx", "fy", "cx", "cy")})
        self._pose = camera_pose(scene.standby, 0.0, self.caps.mount_pitch_rad)
        self._PoseStamped = PoseStamped
        self.transit, self.t, self.req = 0.0, 0.0, 0
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("kinetix_isaac_rig")
        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.node.create_publisher(PoseStamped, "/kinetix/render/goal", qos)
        self.got = {}
        self.node.create_subscription(Image, "/kinetix/render/image", lambda m: self.got.__setitem__(("img", m.header.stamp.sec), m), qos)
        self.node.create_subscription(Image, "/kinetix/render/depth", lambda m: self.got.__setitem__(("dep", m.header.stamp.sec), m), qos)

    def _spin_until(self, cond, timeout: float, what: str):
        import rclpy
        t_end = time.time() + timeout
        while not cond():
            if time.time() > t_end:
                raise RigError("no_image", f"{what} after {timeout:.0f} s (is scripts/render_server.sh serve running?)")
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def start(self) -> None:
        self._spin_until(lambda: self.pub.get_subscription_count() > 0, 120.0, "no render server on /kinetix/render/goal")

    def pose(self) -> Pose:
        return self._pose

    def move_to(self, target: Pose) -> None:
        self.transit += float(np.linalg.norm(target.p - self._pose.p))
        self._pose = target

    def capture_at(self, target: Pose, idx: int, pool_id: str | None = None) -> Frame:
        import cv2
        self.req += 1
        g = self._PoseStamped()
        g.header.frame_id, g.header.stamp.sec = "map", self.req
        g.pose.position.x, g.pose.position.y, g.pose.position.z = (float(x) for x in target.p)
        g.pose.orientation.x, g.pose.orientation.y, g.pose.orientation.z, g.pose.orientation.w = (float(x) for x in target.q)
        self.pub.publish(g)
        key_i, key_d = ("img", self.req), ("dep", self.req)
        self._spin_until(lambda: key_i in self.got and key_d in self.got, self.rc["render_timeout_s"], "no rendered image")
        im, dm = self.got.pop(key_i), self.got.pop(key_d)
        self.got.clear()
        rgb = np.frombuffer(im.data, np.uint8).reshape(im.height, im.width, 3)
        depth = np.frombuffer(dm.data, np.float32).reshape(dm.height, dm.width).copy()
        travel = float(np.linalg.norm(target.p - self._pose.p)) + self.transit
        self.transit, self._pose = 0.0, target
        self.t += travel / self.caps.speed_mps + 1.0
        rel = Path("frames") / f"{idx:06d}.png"
        (self.run_dir / "frames").mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(self.run_dir / rel), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        depth_file = None
        if self.rc.get("save_depth"):
            depth_file = Path("depth") / f"{idx:06d}.npy"
            (self.run_dir / "depth").mkdir(exist_ok=True)
            np.save(self.run_dir / depth_file, depth)
        return Frame(idx=idx, t=self.t, image=rel, K=self.K, target=target, pose=target, gt_pose=target, travel_m=travel,
                     pool_id=pool_id, depth=depth, depth_file=depth_file)

    def finish(self) -> None:
        self.node.destroy_node()
