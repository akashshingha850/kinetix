"""DryRig: kinematic, no images. The camera teleports to each target (optional pose noise); used to test the loop and to
compare planners on the coverage model in seconds, without the sim."""
from __future__ import annotations

import numpy as np

from kinetix.core.geometry import camera_pose
from kinetix.core.types import Frame, Intrinsics, Pose, RigCaps


class DryRig:
    def __init__(self, cfg: dict, scene, run_dir):
        rc = cfg["rig"]
        cam = cfg["camera"]
        self.caps = RigCaps(continuous=True, has_gt=True, mount_pitch_rad=np.radians(cam["mount_pitch_deg"]),
                            speed_mps=rc["speed_mps"])
        self.K = Intrinsics(**{k: cam[k] for k in ("width", "height", "fx", "fy", "cx", "cy")})
        self.noise = float(rc.get("dry_noise_m", 0.0))
        self.rng = np.random.default_rng(0)
        self._pose = camera_pose(scene.standby, 0.0, self.caps.mount_pitch_rad)
        self.t = 0.0

    def start(self) -> None:
        pass

    def pose(self) -> Pose:
        return self._pose

    def capture_at(self, target: Pose, idx: int, pool_id: str | None = None) -> Frame:
        travel = float(np.linalg.norm(target.p - self._pose.p))
        self.t += travel / self.caps.speed_mps + 1.0
        reported = Pose(target.p + self.rng.normal(0.0, self.noise, 3), target.q) if self.noise else target
        self._pose = target
        return Frame(idx=idx, t=self.t, image=None, K=self.K, target=target, pose=reported, gt_pose=target,
                     travel_m=travel, pool_id=pool_id)

    def fly_path(self, path, step_m: float, step_yaw: float, on_frame) -> None:
        """Continuous flight along camera poses `path`: a keyframe whenever the camera moved `step_m` or turned `step_yaw`
        since the last one (no stopping)."""
        from kinetix.core.geometry import camera_yaw, wrap
        last, idx, travel = None, 0, 0.0
        for p in path:
            travel += float(np.linalg.norm(p.p - self._pose.p))
            self.t += float(np.linalg.norm(p.p - self._pose.p)) / self.caps.speed_mps
            self._pose = p
            if last is None or np.linalg.norm(p.p - last.p) >= step_m or abs(wrap(camera_yaw(p) - camera_yaw(last))) >= step_yaw:
                on_frame(Frame(idx=idx, t=self.t, image=None, K=self.K, target=p, pose=p, gt_pose=p, travel_m=travel))
                idx, last, travel = idx + 1, p, 0.0

    def finish(self) -> None:
        pass
