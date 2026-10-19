"""`python -m kinetix run ...` (interfaces.md §6; `uv run kinetix` once tool.uv.package = true)."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from kinetix.core.config import ROOT, load_config, load_scene
from kinetix.core.runlog import RunLog
from kinetix.core.types import Intrinsics
from kinetix.planner.loop import METHODS, run_session
from kinetix.recon.coverage import CoverageState
from kinetix.rig.base import load_rig


def cmd_run(a) -> int:
    scene = load_scene(a.scene)
    cfg = load_config(a.config or [], a.overrides, kind=scene.kind)
    log = RunLog(a.out, scene.name, a.method, a.rig, a.seed, cfg, uses_gt=False, repo_root=ROOT)
    log.info(f"run {log.run_id}: scene {scene.name} ({scene.kind}), method {a.method}, rig {a.rig}, "
             f"budget {cfg['planner']['budget']}")
    cam = cfg["camera"]
    recon = CoverageState.for_scene(scene, Intrinsics(**{k: cam[k] for k in ("width", "height", "fx", "fy", "cx", "cy")}),
                                    cfg["coverage"])
    log.info(f"coverage model: {len(recon.P)} surface points, {len(recon.occluders)} occluders")
    rig = load_rig(a.rig, cfg, scene, log.dir)
    try:
        result = run_session(cfg, scene, rig, recon, a.method, log, a.seed)
    except KeyboardInterrupt:
        result = {"stop_reason": "aborted", "coverage": round(recon.coverage, 4), "seen": round(recon.seen, 4)}
    summary = log.close(result.pop("stop_reason") or "unknown", result)
    print(f"run dir: {log.dir}")
    return 0 if summary["frames"] > 0 and summary["stop_reason"] != "aborted" else 1


def cmd_sfm(a) -> int:
    from kinetix.offline.sfm import run_sfm
    res = run_sfm(a.run, a.scene, max_features=a.max_features, pairing=a.pairing, min_shared=a.min_shared,
                  rig_check=not a.no_rig_check, prior_sigma_m=None if a.prior_sigma <= 0 else a.prior_sigma,
                  pose_source=a.pose_source)
    print(json.dumps(res, indent=1))
    return 0 if res.get("registered", 0) >= 3 else 1


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
    f = sub.add_parser("sfm", help="offline SfM over a run's frames (pycolmap) + ATE and proxy completeness")
    f.add_argument("run", help="run directory")
    f.add_argument("--scene", default=None, help="scene yaml (default: config/scenes/<run scene>.yaml)")
    f.add_argument("--max-features", type=int, default=8192)
    f.add_argument("--pairing", choices=["video", "covis", "exhaustive"], default=None,
                   help="image pairs to match (default: video above 150 frames, covis when the scene is known, else exhaustive)")
    f.add_argument("--min-shared", type=int, default=10, help="covis: shared proxy-surface points for a pair")
    f.add_argument("--no-rig-check", action="store_true", help="keep pairs whose relative pose disagrees with the rig poses")
    f.add_argument("--prior-sigma", type=float, default=0.5, help="sfm: position prior std (m) from the rig poses; 0 = none")
    f.add_argument("--pose-source", choices=["prior", "sfm"], default=None,
                   help="prior: triangulate at the rig poses + BA (default for tours / > 150 frames); sfm: incremental mapping")
    a = ap.parse_args(argv)
    if a.cmd == "sfm":
        return cmd_sfm(a)
    a.scene = Path(a.scene)
    return cmd_run(a)
