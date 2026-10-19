"""YAML layering + key=value overrides, and the scene loader (architecture.md §10, data-format.md §1)."""
from __future__ import annotations

import copy
from pathlib import Path

import numpy as np
import yaml

from kinetix.core.types import Box, Scene

ROOT = Path(__file__).resolve().parents[3]          # repo root (src/kinetix/core/config.py)
DEFAULT = ROOT / "config" / "default.yaml"
ROOM = ROOT / "config" / "room.yaml"                # layered after default.yaml for `kind: room` scenes (ADR-K11)


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
    """default.yaml < room.yaml (room scenes only) < `paths` in order < key=value overrides."""
    cfg = yaml.safe_load(DEFAULT.read_text()) or {}
    if kind == "room":
        cfg = merge(cfg, yaml.safe_load(ROOM.read_text()) or {})
    for p in paths:
        cfg = merge(cfg, yaml.safe_load(Path(p).read_text()) or {})
    return apply_overrides(cfg, list(overrides))


def _box(d: dict) -> Box:
    return Box(np.asarray(d["center"], dtype=float), np.asarray(d["size"], dtype=float), np.radians(d.get("yaw_deg", 0.0)))


def load_scene(path: Path | str) -> Scene:
    path = Path(path)
    if path.is_dir():
        path = path / "scene.yaml"
    d = yaml.safe_load(path.read_text())
    gt = d.get("gt_mesh")
    kind = d.get("kind", "object")
    if kind not in ("object", "room"):
        raise ValueError(f"{path}: kind {kind!r} (object | room)")
    return Scene(
        name=d["name"],
        roi=_box(d["roi"]),
        workspace=_box(d["workspace"]),
        no_fly=tuple(_box(b) for b in d.get("no_fly", [])),
        standoff=tuple(float(x) for x in d.get("standoff_m", [0.0, float("inf")])),   # unused in room scenes
        altitude=tuple(float(x) for x in d["altitude_m"]),
        gt_mesh=(path.parent / gt) if gt else None,
        standby=np.asarray(d.get("standby", [0.0, 0.0, 1.2]), dtype=float),
        world_to_map=np.asarray(d.get("sim", {}).get("world_to_map", [0.0, 0.0, 0.0]), dtype=float),
        kind=kind,
    )
