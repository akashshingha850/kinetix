"""Core types (interfaces.md §2). Poses are T_map_cam: ENU `map`, OpenCV optical camera, quaternion xyzw."""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class Pose:
    p: np.ndarray                  # (3,)
    q: np.ndarray                  # (4,) xyzw, unit

    def to_dict(self) -> dict:
        return {"p": [round(float(x), 5) for x in self.p], "q": [round(float(x), 6) for x in self.q]}


@dataclass(frozen=True)
class Intrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    dist: tuple[float, ...] = ()   # empty = rectified (the ZED rect image is)

    def to_dict(self) -> dict:
        return {"width": self.width, "height": self.height, "fx": self.fx, "fy": self.fy, "cx": self.cx, "cy": self.cy}


@dataclass(frozen=True)
class Frame:
    idx: int                       # capture order, 0-based
    t: float
    image: Path | None             # relative to the run dir; None only on the `dry` rig (no camera)
    K: Intrinsics
    target: Pose                   # what the planner asked for
    pose: Pose                     # what the rig reports (EKF2 ∘ mount on T2)
    gt_pose: Pose | None           # sim only; never read by planner/recon
    travel_m: float                # path length flown since the previous frame
    pool_id: str | None = None

    def to_dict(self) -> dict:
        return {"idx": self.idx, "t": round(self.t, 3), "image": str(self.image) if self.image else None,
                "right_image": None, "depth": None, "depth_source": None, "K": self.K.to_dict(),
                "target": self.target.to_dict(), "pose": self.pose.to_dict(),
                "gt_pose": self.gt_pose.to_dict() if self.gt_pose else None,
                "travel_m": round(self.travel_m, 3), "pool_id": self.pool_id}


@dataclass(frozen=True)
class Candidate:
    pose: Pose
    pool_id: str | None = None     # also the candidate's id on continuous rigs (e.g. "r1.40_z1.35_a180")


@dataclass(frozen=True)
class Box:
    center: np.ndarray
    size: np.ndarray
    yaw: float = 0.0               # rad, about map z (only 0 is supported by the geometry helpers so far)

    @property
    def lo(self) -> np.ndarray:
        return self.center - self.size / 2

    @property
    def hi(self) -> np.ndarray:
        return self.center + self.size / 2

    def inflated(self, m: float) -> "Box":
        return Box(self.center, self.size + 2 * m, self.yaw)


@dataclass(frozen=True)
class Scene:
    name: str
    roi: Box                       # kind "room": the room's inner box (walls, floor, ceiling are the surface, ADR-K11)
    workspace: Box
    no_fly: tuple[Box, ...]
    standoff: tuple[float, float]  # camera distance to the ROI surface, m
    altitude: tuple[float, float]  # camera height in map, m
    gt_mesh: Path | None = None
    standby: np.ndarray = field(default_factory=lambda: np.zeros(3))  # camera position the rig climbs to first (map)
    world_to_map: np.ndarray = field(default_factory=lambda: np.zeros(3))  # sim only: map = world + this (EKF2 origin = spawn)
    kind: str = "object"           # "object" (outside-in, around the ROI) | "room" (inside-out, the ROI is the room)


def shifted(scene: Scene, d) -> Scene:
    """The scene moved by `d` in map (every box, standby, world_to_map): the rig measured the EKF2 origin elsewhere than the
    scene's nominal `world_to_map` (sim: the barometric height origin differs by ~0.25 m between boots)."""
    from dataclasses import replace
    d = np.asarray(d, dtype=float)
    mv = lambda b: Box(b.center + d, b.size, b.yaw)                     # noqa: E731
    return replace(scene, roi=mv(scene.roi), workspace=mv(scene.workspace), no_fly=tuple(mv(b) for b in scene.no_fly),
                   standby=scene.standby + d, world_to_map=scene.world_to_map + d)


@dataclass(frozen=True)
class RigCaps:
    continuous: bool
    has_gt: bool
    mount_pitch_rad: float
    speed_mps: float
    stereo: bool = False
    baseline_m: float | None = None
    sensor_depth: bool = False


class RigError(RuntimeError):
    """kind: unreachable | timeout | no_image | aborted (interfaces.md §3)."""

    def __init__(self, kind: str, msg: str = ""):
        super().__init__(f"{kind}: {msg}" if msg else kind)
        self.kind = kind
