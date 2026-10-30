"""YAML layering + key=value overrides, and the scene loader (architecture.md §10, data-format.md §1)."""
from __future__ import annotations

import copy
from pathlib import Path

import numpy as np
import yaml

from kinetix.types import Box, Scene

ROOT = Path(__file__).resolve().parents[1]          # repo root (kinetix/config.py)
CONFIG = ROOT / "acquisition" / "config"          # common.yaml, then object.yaml or room.yaml by the scene's kind (ADR-K11)
SCENES = ROOT / "kinetix" / "scenes"                # scene files, shared: acquisition flies them, reconstruction scores against them
DATA = ROOT / "data"                                # run dirs: acquisition writes the flight, reconstruction its <method>/ subdirs


def merge(a: dict, b: dict) -> dict:
    """Deep merge, b wins; lists are replaced."""
    out = copy.deepcopy(a)
    for k, v in (b or {}).items():
        out[k] = merge(out[k], v) if isinstance(v, dict) and isinstance(out.get(k), dict) else copy.deepcopy(v)
    return out


def apply_overrides(cfg: dict, overrides: list[str]) -> dict:
    """`planner.budget=24` style overrides; values are parsed as YAML (numbers, lists, bools)."""
    cfg = copy.deepcopy(cfg)
    for item in overrides:
        key, sep, raw = item.partition("=")
        if not sep:
            raise ValueError(f"override {item!r} is not key=value")
        node = cfg
        *parents, leaf = key.split(".")
        for p in parents:
            node = node.setdefault(p, {})
        node[leaf] = yaml.safe_load(raw)
    return cfg


def load_config(paths: list[Path | str] = (), overrides: list[str] = (), kind: str = "object") -> dict:
    """common.yaml < object.yaml | room.yaml (by `kind`) < `paths` in order < key=value overrides."""
    cfg = {}
    for name in ("common", kind):
        cfg = merge(cfg, yaml.safe_load((CONFIG / f"{name}.yaml").read_text()) or {})
    for p in paths:
        cfg = merge(cfg, yaml.safe_load(Path(p).read_text()) or {})
    return apply_overrides(cfg, list(overrides))


def altitude_band(floor: float, ceiling: float, fit: dict) -> tuple[float, float]:
    """A room's camera height band (map z) from its floor and ceiling: `fit.altitude_above_floor_m` [low, cap | null] over the
    floor, and ceiling_margin_m + 0.4 m under the ceiling. For fitted rooms (roomfit) and scene files with `altitude_m: auto`."""
    low, cap = fit["altitude_above_floor_m"]
    zhi = ceiling - fit["ceiling_margin_m"] - 0.4
    return floor + low, zhi if cap is None else min(zhi, floor + cap)


def _box(d: dict) -> Box:
    return Box(np.asarray(d["center"], dtype=float), np.asarray(d["size"], dtype=float), np.radians(d.get("yaw_deg", 0.0)))


def load_scene(path: Path | str, cfg: dict | None = None) -> Scene:
    """A scene file. `altitude_m: auto` (room scenes): the band follows the room box (altitude_band, with cfg's explore.fit or
    room.yaml's), and so does the workspace's z."""
    path = Path(path)
    if path.is_dir():
        path = path / "scene.yaml"
    d = yaml.safe_load(path.read_text())
    gt = d.get("gt_mesh")
    kind = d.get("kind", "object")
    if kind not in ("object", "room"):
        raise ValueError(f"{path}: kind {kind!r} (object | room)")
    roi, ws, alt = _box(d["roi"]), _box(d["workspace"]), d["altitude_m"]
    if alt == "auto":
        if kind != "room":
            raise ValueError(f"{path}: altitude_m auto needs kind room (the band comes from the room box)")
        alt = altitude_band(roi.lo[2], roi.hi[2], (cfg or load_config(kind="room"))["explore"]["fit"])
        if alt[1] <= alt[0]:
            raise ValueError(f"{path}: room too low for the altitude band {alt}")
        lo, hi = ws.lo.copy(), ws.hi.copy()
        lo[2], hi[2] = alt
        ws = Box((lo + hi) / 2, hi - lo, ws.yaw)
    return Scene(
        name=d["name"],
        roi=roi,
        workspace=ws,
        no_fly=tuple(_box(b) for b in d.get("no_fly", [])),
        standoff=tuple(float(x) for x in d.get("standoff_m", [0.0, float("inf")])),   # unused in room scenes
        altitude=tuple(float(x) for x in alt),
        gt_mesh=(path.parent / gt) if gt else None,
        standby=np.asarray(d.get("standby", [0.0, 0.0, 1.2]), dtype=float),
        world_to_map=np.asarray(d.get("sim", {}).get("world_to_map", [0.0, 0.0, 0.0]), dtype=float),
        kind=kind,
    )
