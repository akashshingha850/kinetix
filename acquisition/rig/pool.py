"""PoolRig (T0, interfaces.md §3): replays a pre-rendered view pool (data-format.md §2, `kinetix pool make`). A capture is the
pool view with the candidate's pool_id, hard-linked into the run's frames/, with its GT depth. No flight and no rendering, so a
method x seed x budget matrix costs seconds per session; the pool is rendered once by the same Isaac renderer as the T1 rig."""
from __future__ import annotations

import json
import os
import shutil
from pathlib import Path

import numpy as np

from kinetix.geometry import camera_pose
from kinetix.types import Frame, Intrinsics, Pose, RigCaps, RigError


def _link(src: Path, dst: Path) -> None:
    try:
        os.link(src, dst)
    except OSError:                                    # another filesystem
        shutil.copy(src, dst)


class PoolRig:
    def __init__(self, cfg: dict, scene, run_dir):
        rc, cam = cfg["rig"], cfg["camera"]
        if not rc.get("pool"):
            raise ValueError("rig.pool: the pool folder (kinetix run --pool, default <data>/pools/<scene>/rings)")
        self.pool = Path(rc["pool"])
        self.views = {v["id"]: v for v in map(json.loads, (self.pool / "views.jsonl").read_text().splitlines()) if v.get("split", "candidate") == "candidate"}
        self.rc, self.run_dir = rc, Path(run_dir)
        self.caps = RigCaps(continuous=False, has_gt=True, mount_pitch_rad=np.radians(cam["mount_pitch_deg"]), speed_mps=rc["speed_mps"],
                            sensor_depth=True)
        self.K = Intrinsics(**{k: cam[k] for k in ("width", "height", "fx", "fy", "cx", "cy")})
        self._pose = camera_pose(scene.standby, 0.0, self.caps.mount_pitch_rad)
        self.transit, self.t = 0.0, 0.0

    def start(self) -> None:
        pass

    def pose(self) -> Pose:
        return self._pose

    def move_to(self, target: Pose) -> None:
        self.transit += float(np.linalg.norm(target.p - self._pose.p))
        self._pose = target

    def capture_at(self, target: Pose, idx: int, pool_id: str | None = None) -> Frame:
        v = self.views.get(pool_id)
        if v is None:
            raise RigError("unreachable", f"{pool_id} is not a candidate view of {self.pool}")
        p, q = np.asarray(v["pose"]["p"], float), np.asarray(v["pose"]["q"], float)
        if np.linalg.norm(p - target.p) > 1e-3:
            raise ValueError(f"pool view {pool_id} is {np.linalg.norm(p - target.p):.3f} m from the candidate: the pool was made "
                             "with other planner.rings (kinetix pool make again)")
        pose = Pose(p, q)
        travel = float(np.linalg.norm(p - self._pose.p)) + self.transit
        self.transit, self._pose = 0.0, pose
        self.t += travel / self.caps.speed_mps + 1.0
        rel = Path("frames") / f"{idx:06d}.png"
        _link(self.pool / "images" / f"{pool_id}.png", self.run_dir / rel)
        depth = np.load(self.pool / "depth" / f"{pool_id}.npy")
        depth_file = None
        if self.rc.get("save_depth"):
            depth_file = Path("depth") / f"{idx:06d}.npy"
            (self.run_dir / "depth").mkdir(exist_ok=True)
            _link(self.pool / "depth" / f"{pool_id}.npy", self.run_dir / depth_file)
        return Frame(idx=idx, t=self.t, image=rel, K=self.K, target=target, pose=pose, gt_pose=pose, travel_m=travel,
                     pool_id=pool_id, depth=depth, depth_file=depth_file)

    def finish(self) -> None:
        pass
