"""`python -m kinetix run ...` (interfaces.md §6; `uv run kinetix` once tool.uv.package = true)."""
from __future__ import annotations

import argparse
from pathlib import Path

from kinetix.core.config import ROOT, load_config, load_scene
from kinetix.core.runlog import RunLog
from kinetix.core.types import Intrinsics
from kinetix.planner.loop import METHODS, run_session
from kinetix.recon.coverage import CoverageState
from kinetix.rig.base import load_rig


def cmd_run(a) -> int:
    cfg = load_config(a.config or [], a.overrides)
    scene = load_scene(a.scene)
    log = RunLog(a.out, scene.name, a.method, a.rig, a.seed, cfg, uses_gt=False, repo_root=ROOT)
    log.info(f"run {log.run_id}: scene {scene.name}, method {a.method}, rig {a.rig}, budget {cfg['planner']['budget']}")
    cam, cc = cfg["camera"], cfg["coverage"]
    recon = CoverageState(scene.roi, Intrinsics(**{k: cam[k] for k in ("width", "height", "fx", "fy", "cx", "cy")}),
                          spacing=cc["spacing_m"], depth_range=tuple(cc["depth_range_m"]),
                          max_incidence_deg=cc["max_incidence_deg"], tri_min_deg=cc["tri_min_deg"],
                          novel_deg=cc["novel_deg"], max_views=cc["max_views"])
    rig = load_rig(a.rig, cfg, scene, log.dir)
    try:
        result = run_session(cfg, scene, rig, recon, a.method, log, a.seed)
    except KeyboardInterrupt:
        result = {"stop_reason": "aborted", "coverage": round(recon.coverage, 4), "seen": round(recon.seen, 4)}
    summary = log.close(result.pop("stop_reason") or "unknown", result)
    print(f"run dir: {log.dir}")
    return 0 if summary["frames"] > 0 and summary["stop_reason"] != "aborted" else 1


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="kinetix")
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run", help="one capture session")
    r.add_argument("--scene", required=True, help="scene dir or scene.yaml (e.g. config/scenes/simple_room_blocks.yaml)")
    r.add_argument("--method", default="roi_coverage", choices=METHODS)
    r.add_argument("--rig", default="dry", choices=["dry", "mavros"])
    r.add_argument("--seed", type=int, default=0)
    r.add_argument("--config", action="append", help="extra config YAML over config/default.yaml (repeatable)")
    r.add_argument("--out", default=str(ROOT / "runs"))
    r.add_argument("overrides", nargs="*", help="key=value, e.g. planner.budget=12")
    a = ap.parse_args(argv)
    a.scene = Path(a.scene)
    return cmd_run(a)
