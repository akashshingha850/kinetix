"""Every capture method in one table (methodology.md §2): which loop runs it and what it reads. The CLI, the session loop and
the NBV loop look a method up here instead of testing its name, so a method's information boundary (GT or not) is one row.

surface: what the coverage gain is computed on. `box` = the ROI box proxy (GT-free), `gt` = the scene's GT surface + mesh
occlusion (the oracle: sim only, an upper bound), `depth` = the box until `planner.depth_surface.after_views`, then the surface
the frames' depth shows. needs_depth: the frames must carry depth (`rig.save_depth`). That depth is the renderer's GT on the pool
and isaac rigs and the ZED on mavros, so a depth method reads GT only on a rendered-depth rig (`uses_gt`)."""
from __future__ import annotations

from typing import NamedTuple


class Method(NamedTuple):
    loop: str = "nbv"              # nbv (planner/nbv.py) | tour (planner/tour.py) | surface (planner/surface.py)
    fixed: bool = False            # a fixed view sequence (no scorer)
    surface: str = "box"           # box | gt | depth
    needs_depth: bool = False


METHODS = {
    "roi_coverage": Method(),
    "random": Method(),
    "fvs": Method(),
    "orbit": Method(fixed=True),
    "orbit_rings": Method(fixed=True),
    "volumetric_ig": Method(needs_depth=True),
    "depth_coverage": Method(surface="depth", needs_depth=True),
    "coverage_oracle": Method(surface="gt"),
    "surface_normal": Method(loop="surface", needs_depth=True),
    "tour": Method(loop="tour"),
}
GT_DEPTH_RIGS = ("pool", "isaac")  # rigs whose frame depth is the renderer's GT depth (data-format.md §3 uses_gt)


def uses_gt(method: str, rig: str) -> bool:
    m = METHODS[method]
    return m.surface == "gt" or (m.needs_depth and rig in GT_DEPTH_RIGS)
