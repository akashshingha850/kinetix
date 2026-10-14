"""Rig protocol (interfaces.md §3). The planner imports only this module; adapters are loaded by name (`load_rig`)."""
from __future__ import annotations

from typing import Protocol

from kinetix.core.types import Frame, Pose, RigCaps


class Rig(Protocol):
    caps: RigCaps

    def start(self) -> None: ...
    def pose(self) -> Pose: ...
    def capture_at(self, target: Pose, idx: int, pool_id: str | None = None) -> Frame: ...   # move, settle, grab
    def finish(self) -> None: ...


def load_rig(name: str, cfg: dict, scene, run_dir):
    """Adapters are imported lazily so T0/unit tests never import rclpy."""
    if name == "dry":
        from kinetix.rig.dry import DryRig
        return DryRig(cfg, scene, run_dir)
    if name == "mavros":
        from kinetix.rig.mavros import MavrosRig
        return MavrosRig(cfg, scene, run_dir)
    raise ValueError(f"unknown rig {name!r} (dry | mavros)")
