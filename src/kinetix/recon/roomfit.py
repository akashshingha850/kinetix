"""Fit a room `Scene` (room box, workspace, obstacle boxes) to an OccupancyMap: what the planner otherwise reads from the scene file.

The room box comes from the wall, floor and ceiling planes: along each axis the occupied voxels pile up in a few slices (a wall is
one slice thick), so the room's inner extent is the outermost slice that holds a real share of the densest slices' count. Everything
occupied inside the box, away from its faces, is an obstacle: 26-connected components become axis-aligned boxes, and one standing
on the floor is extended down to it (depth sees the top, not what is under it). The workspace and altitude band are the room
shrunk by the flight margins.

`report["faces"]` is the share of each of the six faces that has been seen (a cell of the face is seen when an occupied voxel lies
within one voxel of the plane): exploration stops when every face reaches the target, i.e. the room is closed.
"""
from __future__ import annotations

from dataclasses import replace

import numpy as np

from kinetix.core.types import Box, Scene
from kinetix.recon.depthmap import OccupancyMap

AXIS_FACE = {0: ("x-", "x+"), 1: ("y-", "y+"), 2: ("floor", "ceiling")}


_OFFS = np.array([(a, b, c) for a in (-1, 0, 1) for b in (-1, 0, 1) for c in (-1, 0, 1) if (a, b, c) != (0, 0, 0)])


def components(mask: np.ndarray) -> list[np.ndarray]:
    """26-connected components of a boolean voxel grid: a list of (n_i, 3) integer index arrays. Pure numpy: a breadth-first
    search that expands one whole layer of voxels per step (the sparse inner voxels of a room are few)."""
    idx = np.argwhere(mask)
    if not len(idx):
        return []
    lin = np.ravel_multi_index(idx.T, mask.shape)
    order = np.argsort(lin)
    lin_s = lin[order]
    comp = np.full(len(idx), -1)
    n = 0
    for seed in range(len(idx)):
        if comp[seed] >= 0:
            continue
        comp[seed] = n
        frontier = np.array([seed])
        while len(frontier):
            nb = (idx[frontier][:, None, :] + _OFFS[None]).reshape(-1, 3)
            ok = np.all((nb >= 0) & (nb < np.array(mask.shape)), axis=1)
            q = np.ravel_multi_index(nb[ok].T, mask.shape)
            pos = np.minimum(np.searchsorted(lin_s, q), len(lin_s) - 1)
            hit = order[pos[lin_s[pos] == q]]
            hit = np.unique(hit[comp[hit] < 0])
            comp[hit] = n
            frontier = hit
        n += 1
    return [idx[comp == k] for k in range(n)]


def _extent(counts: np.ndarray, frac: float, min_voxels: int):
    """(first, last) slice index holding at least max(min_voxels, frac * the 90th-percentile slice count), or None."""
    nz = counts[counts > 0]
    if not len(nz):
        return None
    thr = max(min_voxels, frac * np.percentile(nz, 90))
    idx = np.flatnonzero(counts >= thr)
    return (int(idx[0]), int(idx[-1])) if len(idx) else None


def _plane(counts: np.ndarray, edge: int, inward: int, origin: float, v: float, width: int = 2) -> float:
    """Position of a wall whose outermost slice is `edge`: the centroid of the voxel counts of that slice and the `width` slices
    inward of it. Depth noise spills hits into the voxels either side of the true plane; the centroid of a plane's hits sits on
    the plane (a hit on a voxel boundary splits between two voxels), the outermost slice alone is biased outward."""
    ks = np.arange(edge, edge + inward * (width + 1), inward)
    ks = ks[(ks >= 0) & (ks < len(counts))]
    w = counts[ks].astype(float)
    return float(origin + ((ks * w).sum() / w.sum() + 0.5) * v)


def _wall_sheet(blo: np.ndarray, bhi: np.ndarray, lo: np.ndarray, hi: np.ndarray, near: float) -> bool:
    """A thin slab parallel to, and within `near` m of, a wall: the same wall seen again after the pose drifted, not an obstacle.
    (A real obstacle against a wall is compact; a sheet is thin in one axis and wide in the other two.)"""
    size = bhi - blo
    for ax in range(3):
        thin = size[ax] <= 1.0 and all(size[k] >= 3.0 * max(size[ax], 0.4) for k in range(3) if k != ax)
        close = min(blo[ax] - lo[ax], hi[ax] - bhi[ax]) <= near
        if thin and close:
            return True
    return False


def face_coverage(occ: np.ndarray, box_idx: tuple[tuple[int, int], ...]) -> dict:
    """Share of each face of the (index-space) box that has an occupied voxel within one voxel of its plane."""
    out = {}
    for ax in range(3):
        a0, a1 = box_idx[ax]
        for side, name in zip((a0, a1), AXIS_FACE[ax]):
            sl = [slice(box_idx[k][0], box_idx[k][1] + 1) for k in range(3)]
            sl[ax] = slice(max(side - 1, 0), min(side + 2, occ.shape[ax]))
            cells = occ[tuple(sl)].any(axis=ax)
            out[name] = float(cells.mean()) if cells.size else 0.0
    return out


def face_openings(free: np.ndarray, box_idx: tuple[tuple[int, int], ...]) -> dict:
    """Free voxels just outside each face of the (index-space) box, 2-4 voxels past its plane: space seen through the face. A wall
    has none; a gap beside a partition or a doorway does, and then the face is not the room's wall (or not all of it)."""
    out = {}
    for ax in range(3):
        (a0, a1) = box_idx[ax]
        for side, name, sgn in ((a0, AXIS_FACE[ax][0], -1), (a1, AXIS_FACE[ax][1], +1)):
            sl = [slice(box_idx[k][0], box_idx[k][1] + 1) for k in range(3)]
            s0, s1 = sorted((side + 2 * sgn, side + 4 * sgn))
            sl[ax] = slice(max(s0, 0), max(min(s1 + 1, free.shape[ax]), 0))
            out[name] = int(free[tuple(sl)].sum())
    return out


def fit_scene(omap: OccupancyMap, hint: Scene, fcfg: dict) -> tuple[Scene | None, dict]:
    """The scene the map describes, plus a report. `hint` supplies only what the map cannot know: the name, the take-off point
    (standby) and the sim's world_to_map. Returns (None, report) while the map is too sparse to bound the room."""
    occ = omap.occupied
    v = omap.voxel
    report = {"occupied_voxels": int(occ.sum()), "known_fraction": float((omap.L != 0).mean())}
    ext, lo, hi = [None] * 3, np.zeros(3), np.zeros(3)
    for ax in (2, 0, 1):    # floor and ceiling first: a wall is what stands between them (the floor fills every x/y slice)
        src = occ if ax == 2 else occ[:, :, ext[2][0] + int(np.ceil(fcfg["floor_margin_m"] / v)) + 1:
                                         ext[2][1] - int(np.ceil(fcfg["ceiling_margin_m"] / v))]
        counts = src.sum(axis=tuple(k for k in range(3) if k != ax))
        e = _extent(counts, fcfg["slab_frac"], fcfg["slab_min_voxels"])
        if e is None or e[1] <= e[0]:
            return None, {**report, "closed": False, "faces": {}, "why": f"no extent along axis {ax}"}
        if ax < 2:          # a wall is the densest slice within wall_artifact_m inward of the outermost one: a weaker sheet beyond
            n = int(round(fcfg["wall_artifact_m"] / v))     # it is that wall seen again after pose drift (fitting to it left the
            e = (e[0] + int(np.argmax(counts[e[0]:e[0] + n + 1])),        # real wall inside the room as a wall-sized obstacle)
                 e[1] - int(np.argmax(counts[max(e[1] - n, 0):e[1] + 1][::-1])))
            if e[1] <= e[0]:
                return None, {**report, "closed": False, "faces": {}, "why": f"no extent along axis {ax}"}
        else:               # the floor too, within floor_margin_m: the ZED reads 4-9 % long at 1-6 m, so from 4 m up the floor smears
            n = int(round(fcfg["floor_margin_m"] / v))      # over ~0.3 m and its lowest slice is sparse (run 20261027-182638: 'floor
            floor0 = e[0]                                   # seen 8 %', fitted 0.23 m low, no tour). Obstacles start from the
            e = (e[0] + int(np.argmax(counts[e[0]:e[0] + n + 1])), e[1])  # lowest slice (a low crate keeps its voxels), but at
            floor0 = max(floor0, e[0] - 1)                  # most one below this one: a floor smeared 0.6 m deep put the band on the
            # real floor, and the floor became one 18 x 25 x 2 m obstacle (flight 20261028-121155; the tour flew above it, coverage 0.71)
        ext[ax] = e
        lo[ax], hi[ax] = (_plane(counts, e[0], +1, omap.lo[ax], v), _plane(counts, e[1], -1, omap.lo[ax], v))
    room = Box((lo + hi) / 2, hi - lo)
    faces = face_coverage(occ, tuple(ext))
    openings = face_openings(omap.free, tuple(ext))
    report.update(roi_lo=lo.round(3).tolist(), roi_hi=hi.round(3).tolist(), faces={k: round(x, 3) for k, x in faces.items()},
                  openings=openings, closed=all(x >= fcfg["closed_target"] for x in faces.values())
                  and all(n <= fcfg["max_opening_voxels"] for n in openings.values()))

    # obstacles: occupied voxels inside the room, clear of its faces
    m = np.array([fcfg["wall_margin_m"], fcfg["wall_margin_m"], 0.0]) / v
    inner = np.zeros(occ.shape, dtype=bool)
    s0 = [int(np.ceil(ext[0][0] + m[0] + 1)), int(np.ceil(ext[1][0] + m[1] + 1)), int(np.ceil(floor0 + fcfg["floor_margin_m"] / v + 1))]
    s1 = [int(np.floor(ext[0][1] - m[0])), int(np.floor(ext[1][1] - m[1])), int(np.floor(ext[2][1] - fcfg["ceiling_margin_m"] / v))]
    if all(a < b for a, b in zip(s0, s1)):
        inner[s0[0]:s1[0], s0[1]:s1[1], s0[2]:s1[2]] = occ[s0[0]:s1[0], s0[1]:s1[1], s0[2]:s1[2]]
    boxes, report["blobs"] = [], []
    for vox in components(inner):
        blo = omap.lo + vox.min(axis=0) * v
        bhi = omap.lo + (vox.max(axis=0) + 1) * v
        if len(vox) < fcfg["min_obstacle_voxels"]:
            if len(vox) >= fcfg["min_obstacle_voxels"] // 5:              # too little seen to be an obstacle yet: exploration
                report["blobs"].append((blo.round(2).tolist(), bhi.round(2).tolist()))   # looks again (explore.face_targets)
            continue
        if _wall_sheet(blo, bhi, lo, hi, fcfg["wall_artifact_m"]):
            continue
        if blo[2] - lo[2] < fcfg["floor_extend_m"]:                   # stands on the floor: depth never saw under it
            blo[2] = lo[2]
        boxes.append(Box((blo + bhi) / 2, bhi - blo))
    # workspace + altitude: the room shrunk by the flight margins
    wm = fcfg["workspace_margin_m"]
    wlo, whi = lo.copy(), hi.copy()
    for ax in (0, 1):
        mm = min(wm, 0.4 * (hi[ax] - lo[ax]))                         # a small room keeps a flyable core
        wlo[ax], whi[ax] = lo[ax] + mm, hi[ax] - mm
    zlo = lo[2] + fcfg["altitude_above_floor_m"][0]
    zhi = min(lo[2] + fcfg["altitude_above_floor_m"][1], hi[2] - fcfg["ceiling_margin_m"] - 0.4)
    if zhi <= zlo:
        return None, {**report, "closed": False, "why": "room too low for the altitude band"}
    wlo[2], whi[2] = zlo, zhi
    # only what the drone can reach is an obstacle: roof beams above the workspace are not (Warehouse 2026-10-27, z 8-9 m)
    near = fcfg["obstacle_reach_m"]
    boxes = [b for b in boxes if np.all(b.lo < whi + near) and np.all(b.hi > wlo - near)]
    report["n_obstacles"] = len(boxes)
    scene = replace(hint, name=f"{hint.name}_explored", roi=room, workspace=Box((wlo + whi) / 2, whi - wlo), no_fly=tuple(boxes),
                    standoff=(0.0, float("inf")), altitude=(float(zlo), float(zhi)), gt_mesh=None, kind="room")
    return scene, report


def translated(s: Scene, d) -> Scene:
    """The scene's room, workspace, obstacles and altitude band moved by d (a drift offset between two frames)."""
    d = np.asarray(d, dtype=float)
    mv = lambda b: Box(b.center + d, b.size, b.yaw)                       # noqa: E731
    return replace(s, roi=mv(s.roi), workspace=mv(s.workspace), no_fly=tuple(mv(b) for b in s.no_fly),
                   altitude=(s.altitude[0] + float(d[2]), s.altitude[1] + float(d[2])))


def compare(fitted: Scene, truth: Scene) -> dict:
    """How far the fitted scene is from a known one (the scene file): wall offsets and which true obstacles were found."""
    d_lo, d_hi = fitted.roi.lo - truth.roi.lo, fitted.roi.hi - truth.roi.hi
    found = 0
    for t in truth.no_fly:
        for f in fitted.no_fly:
            inter = np.maximum(np.minimum(t.hi, f.hi) - np.maximum(t.lo, f.lo), 0).prod()
            union = t.size.prod() + f.size.prod() - inter
            if union > 0 and inter / union >= 0.25:
                found += 1
                break
    return {"roi_lo_err_m": d_lo.round(3).tolist(), "roi_hi_err_m": d_hi.round(3).tolist(),
            "roi_max_err_m": round(float(max(np.abs(d_lo).max(), np.abs(d_hi).max())), 3),
            "obstacles_true": len(truth.no_fly), "obstacles_found": found, "obstacles_fitted": len(fitted.no_fly)}


def scene_to_dict(s: Scene) -> dict:
    """A scene in the scene-file format (config/scenes/*.yaml), so a fitted room can be saved, inspected and reused."""
    box = lambda b: {"center": [round(float(x), 3) for x in b.center], "size": [round(float(x), 3) for x in b.size]}   # noqa: E731
    return {"name": s.name, "kind": s.kind, "roi": box(s.roi), "workspace": box(s.workspace), "no_fly": [box(b) for b in s.no_fly],
            "altitude_m": [round(float(x), 3) for x in s.altitude], "standby": [round(float(x), 3) for x in s.standby],
            "sim": {"world_to_map": [round(float(x), 3) for x in s.world_to_map]}}
