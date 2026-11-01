"""DryRig: kinematic, no images. The camera teleports to each target (optional pose noise); used to test the loop and to
compare planners on the coverage model in seconds, without the sim."""
from __future__ import annotations

import numpy as np

from kinetix.geometry import camera_pose
from kinetix.types import Frame, Intrinsics, Pose, RigCaps


class DryRig:
    def __init__(self, cfg: dict, scene, run_dir, truth=None):
        rc = cfg["rig"]
        cam = cfg["camera"]
        self.caps = RigCaps(continuous=True, has_gt=True, mount_pitch_rad=np.radians(cam["mount_pitch_deg"]),
                            speed_mps=rc["speed_mps"])
        self.K = Intrinsics(**{k: cam[k] for k in ("width", "height", "fx", "fy", "cx", "cy")})
        self.noise = float(rc.get("dry_noise_m", 0.0))
        self.cruise = float(rc.get("cruise_mps", rc["speed_mps"]))
        self.rng = np.random.default_rng(0)
        self._pose = camera_pose(scene.standby, 0.0, self.caps.mount_pitch_rad)
        self.t = 0.0
        self.transit = 0.0
        self.truth = truth if truth is not None else scene      # the room the synthetic depth is cast against
        self.depth = bool(cfg.get("explore", {}).get("enabled")) or bool(rc.get("save_depth"))
        self.depth_noise = float(rc.get("dry_depth_noise_frac", 0.0))
        self.depth_max = float(rc.get("dry_depth_max_m", 15.0))      # the ZED returns depth well beyond what the map trusts
        self.drift = float(rc.get("dry_drift_m_per_sqrt_m", 0.0))    # EKF2 drift: a random-walk offset of the reported pose
        self.bias = np.zeros(3)
        self.caps = RigCaps(continuous=True, has_gt=True, mount_pitch_rad=self.caps.mount_pitch_rad, speed_mps=self.caps.speed_mps,
                            sensor_depth=self.depth)

    def start(self) -> None:
        pass

    def pose(self) -> Pose:
        return self._pose

    def move_to(self, target: Pose) -> None:
        d = float(np.linalg.norm(target.p - self._pose.p))
        self.t += d / self.cruise
        self.transit += d                                     # counted in the next frame's travel_m
        self._pose = target

    def capture_at(self, target: Pose, idx: int, pool_id: str | None = None) -> Frame:
        leg = float(np.linalg.norm(target.p - self._pose.p))
        travel, self.transit = leg + self.transit, 0.0
        self.t += leg / self.cruise + 1.0                      # flight time at the rig's cruise speed + the grab
        if self.drift:
            self.bias = self.bias + self.rng.normal(0.0, self.drift * np.sqrt(max(travel, 0.0)), 3) * np.array([1.0, 1.0, 0.2])
        # the controller flies in the EKF frame: it reaches its target there, so the TRUE pose is the target minus the drift
        reported = Pose(target.p + self.rng.normal(0.0, self.noise, 3), target.q) if self.noise else target
        true = Pose(target.p - self.bias, target.q) if self.drift else target
        self._pose = reported
        depth = None
        if self.depth:
            from acquisition.rig.simdepth import render_depth
            room = self.truth.kind == "room"           # an object's ROI box is an obstacle inside a large empty room
            depth = render_depth(self.truth.roi if room else self.truth.workspace.inflated(10.0),
                                 self.truth.no_fly if room else (*self.truth.no_fly, self.truth.roi), true, self.K,
                                 noise_frac=self.depth_noise, max_range=self.depth_max, rng=self.rng)
        return Frame(idx=idx, t=self.t, image=None, K=self.K, target=target, pose=reported, gt_pose=true,
                     travel_m=travel, pool_id=pool_id, depth=depth)

    def fly_path(self, path, step_m: float, step_yaw: float, on_frame) -> None:
        """Continuous flight along camera poses `path`: a keyframe whenever the camera moved `step_m` or turned `step_yaw`
        since the last one (no stopping)."""
        from kinetix.geometry import camera_yaw, wrap
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
