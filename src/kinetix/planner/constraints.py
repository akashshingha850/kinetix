"""Feasibility (architecture.md §8): inside the workspace, at an allowed altitude, outside the no-fly boxes, inside the
ROI standoff shell, and the straight segment from the current camera position clears every box. v1 has no path planner:
a blocked segment drops the candidate."""
from __future__ import annotations

import numpy as np

from kinetix.core.geometry import inside, point_box_distance, segment_hits_box
from kinetix.core.types import Candidate, Pose, Scene


class BoxConstraints:
    def __init__(self, clearance_m: float = 0.35, roi_clearance_m: float = 0.45):
        self.clearance = clearance_m            # drone radius + margin around no-fly boxes
        self.roi_clearance = roi_clearance_m    # never fly closer than this to the ROI box (segments included)

    def feasible(self, cands: list[Candidate], current: Pose, scene: Scene, blocked: set | None = None) -> np.ndarray:
        boxes = [b.inflated(self.clearance) for b in scene.no_fly] + [scene.roi.inflated(self.roi_clearance)]
        mask = np.zeros(len(cands), dtype=bool)
        for i, c in enumerate(cands):
            p = c.pose.p
            if blocked and c.pool_id in blocked:
                continue
            if not inside(p, scene.workspace) or not (scene.altitude[0] <= p[2] <= scene.altitude[1]):
                continue
            if not (scene.standoff[0] <= point_box_distance(p, scene.roi) <= scene.standoff[1]):
                continue
            if any(inside(p, b) for b in boxes):
                continue
            if any(segment_hits_box(current.p, p, b) for b in boxes):
                continue
            mask[i] = True
        return mask
