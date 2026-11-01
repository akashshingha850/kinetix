"""The session entry point (acquisition/architecture.md §5): picks the method's loop. Never imports rclpy or a rig adapter (dependency rule).
Stepwise NBV (roi_coverage, random, fvs, volumetric_ig, orbit, orbit_rings; depth_coverage and coverage_oracle = roi_coverage on the depth / GT surface): planner/nbv.py. Planned tour, optionally after exploring the room: planner/tour.py.
Coarse orbit, then viewpoints along the coarse surface's normals (surface_normal, a baseline after Liu 2026): planner/surface.py."""
from __future__ import annotations

import numpy as np

from kinetix.types import Scene, shifted

METHODS = ("roi_coverage", "random", "orbit", "orbit_rings", "fvs", "volumetric_ig", "tour", "surface_normal", "depth_coverage",
           "coverage_oracle")


def apply_shift(rig, scene: Scene, recon, log) -> Scene:
    shift = np.asarray(getattr(rig, "map_shift", np.zeros(3)), dtype=float)
    if np.linalg.norm(shift) > 0.01:                  # the rig found the map origin elsewhere: move the scene with it
        scene = shifted(scene, shift)
        if hasattr(recon, "shift"):
            recon.shift(shift)
        log.info(f"scene shifted by {np.round(shift, 3).tolist()} m (measured map origin vs the scene's world_to_map)")
    return scene


def run_session(cfg: dict, scene: Scene, rig, recon, method: str, log, seed: int = 0, truth: Scene | None = None) -> dict:
    if method == "tour":
        if scene.kind != "room":
            raise ValueError("tour is a room method (grid viewpoints + TSP + continuous capture)")
        from acquisition.planner.tour import run_tour
        return run_tour(cfg, scene, rig, recon, log, truth)
    if method == "surface_normal":
        from acquisition.planner.surface import run_surface
        return run_surface(cfg, scene, rig, recon, log)
    from acquisition.planner.nbv import run_nbv
    return run_nbv(cfg, scene, rig, recon, method, log, seed)
