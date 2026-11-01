"""`python -m kinetix run | sfm | eval | pool make | bench ...` (interfaces.md §6; `uv run kinetix` once tool.uv.package = true)."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from acquisition.planner.loop import METHODS, run_session
from acquisition.rig.base import load_rig
from kinetix.config import DATA, ROOT, load_config, load_scene
from kinetix.coverage import CoverageState, MeshOcclusion, gt_surface
from kinetix.runlog import RunLog
from kinetix.types import Intrinsics

DEPTH_METHODS = ("surface_normal", "depth_coverage", "volumetric_ig")


def cmd_run(a) -> int:
    scene = load_scene(a.scene)
    truth = None
    if a.explore:
        if a.method != "tour" or scene.kind != "room":
            raise SystemExit("--explore: a room scene with --method tour (the room is explored, then toured)")
        a.overrides = list(a.overrides) + ["explore.enabled=true"]
    if a.method in DEPTH_METHODS:                    # they plan on (or map) the depth of the views so far
        a.overrides = list(a.overrides) + ["rig.save_depth=true"]
    if a.rig == "pool":
        a.overrides = list(a.overrides) + [f"rig.pool={a.pool or default_pool(a.scene)}"]
    cfg = load_config(a.config or [], a.overrides, kind=scene.kind)
    scene = load_scene(a.scene, cfg)    # again with this run's config: `altitude_m: auto` takes its band settings
    if a.explore:                       # the scene file only scores the fit: planner and rig get a take-off point and a search volume
        from acquisition.planner.explore import hint_scene
        truth, scene = scene, hint_scene(scene, cfg["explore"]["volume_m"], cfg["explore"].get("altitude_m"))
    tag = ("explore-opt" if cfg["explore"]["reuse_views"] else "explore-trad") if a.explore else ""
    oracle = a.method == "coverage_oracle"            # plans on the GT surface: an upper bound, flagged in run.yaml
    # the pool and isaac rigs' depth is the renderer's GT depth: a depth method there reads GT too (data-format.md §3 uses_gt)
    uses_gt = oracle or (a.method in DEPTH_METHODS and a.rig in ("pool", "isaac"))
    log = RunLog(a.out, scene.name, a.method, a.rig, a.seed, cfg, uses_gt=uses_gt, repo_root=ROOT, tag=tag,
                 scene_path=a.scene.resolve() if a.scene.exists() else None)
    a.run_dir = log.dir
    log.info(f"run {log.run_id}: scene {scene.name} ({scene.kind}), method {a.method}, rig {a.rig}, "
             f"budget {cfg['planner']['budget']}")
    cam = cfg["camera"]
    recon = CoverageState.for_scene(scene, Intrinsics(**{k: cam[k] for k in ("width", "height", "fx", "fy", "cx", "cy")}),
                                    cfg["coverage"], gt_surface(scene, cfg["coverage"]["spacing_m"]) if oracle else None,
                                    MeshOcclusion(scene.gt_mesh) if oracle else None)   # the GT mesh hides what it hides
    log.info(f"coverage model: {len(recon.P)} surface points, {len(recon.occluders)} occluders")
    rig = load_rig(a.rig, cfg, scene, log.dir, truth)
    try:
        result = run_session(cfg, scene, rig, recon, a.method, log, a.seed, truth)
    except KeyboardInterrupt:
        result = {"stop_reason": "aborted", "coverage": round(recon.coverage, 4), "seen": round(recon.seen, 4)}
    if getattr(rig, "map_shift", None) is not None:    # measured - scene world_to_map: the scene's GT + map_shift = this run's map
        result["map_shift"] = [round(float(x), 4) for x in rig.map_shift]
    summary = log.close(result.pop("stop_reason") or "unknown", result)
    print(f"run dir: {log.dir}")
    return 0 if summary["frames"] > 0 and summary["stop_reason"] not in ("aborted", "stalled") else 1


def default_pool(scene: Path) -> Path:
    """$KINETIX_DATA/pools/<scene>/rings for a scene folder in $KINETIX_DATA/scenes (data-format.md §2)."""
    d = Path(scene).resolve()
    d = d if d.is_dir() else d.parent
    return d.parent.parent / "pools" / d.name / "rings"


def cmd_pool(a) -> int:
    """views.jsonl of the planner's ring-stack candidates (object.yaml planner.rings: the views a session can pick), then the
    T1 batch render of left + right + GT depth (scripts/render_server.sh; stop the flight sim first)."""
    import subprocess

    import numpy as np
    import yaml

    from acquisition.planner.sampler import ring_stack
    from kinetix.types import RigCaps
    scene = load_scene(a.scene)
    if scene.kind != "object":
        raise SystemExit("pool make: object scenes (ring stack)")
    cfg = load_config([], a.overrides, kind=scene.kind)
    rc = cfg["planner"]["rings"]
    caps = RigCaps(continuous=False, has_gt=True, mount_pitch_rad=np.radians(cfg["camera"]["mount_pitch_deg"]), speed_mps=1.0)
    cands = ring_stack(scene, caps, rc["radii"], rc["heights"], rc["n_az"])
    out = Path(a.out) if a.out else default_pool(a.scene)
    out.mkdir(parents=True, exist_ok=True)
    req = out / "request.jsonl"                       # the render server copies it to views.jsonl
    req.write_text("".join(json.dumps({"id": c.pool_id, "split": "candidate", "reachable": True, "pose": c.pose.to_dict(),
                                       "image": f"images/{c.pool_id}.png", "right": f"right/{c.pool_id}.png",
                                       "depth": f"depth/{c.pool_id}.npy"}) + "\n" for c in cands))
    print(f"{req}: {len(cands)} views ({len(rc['radii'])} radii x {len(rc['heights'])} heights x {rc['n_az']} azimuths)")
    if a.views_only:
        return 0
    d = Path(a.scene).resolve()
    subprocess.run([str(ROOT / "scripts" / "render_server.sh"), "batch", (d if d.is_dir() else d.parent).name, str(req), str(out)],
                   check=True)
    meta = yaml.safe_load((out / "pool.yaml").read_text())
    meta["format"] = 1
    meta["generator"].update(layout="rings", rings=rc, git=_git_sha())
    meta["splits"] = {"candidate": len(cands), "test": 0}
    (out / "pool.yaml").unlink()                      # written by the container's user: replace it, not edit it
    (out / "pool.yaml").write_text(yaml.safe_dump(meta, sort_keys=False))
    return 0


def _git_sha() -> str | None:
    import subprocess
    r = subprocess.run(["git", "-C", str(ROOT), "rev-parse", "--short", "HEAD"], capture_output=True, text=True)
    return r.stdout.strip() or None


def cmd_eval(a) -> int:
    from reconstruction.evaluate import evaluate
    res = evaluate(a.run, a.scene)
    print(json.dumps(res, indent=1))
    return 0 if res.get("eval_valid") else 1


def cmd_bench(a) -> int:
    from bench.matrix import run_matrix
    from bench.report import write_report
    if a.report:
        write_report([Path(d) for d in a.report], Path(a.out) if a.out else Path(a.report[0]))
        return 0
    if not a.matrix:
        raise SystemExit("--matrix, or --report <bench dirs>")
    out = run_matrix(Path(a.matrix), jobs=a.jobs, out=Path(a.out) if a.out else None, sessions_only=a.sessions_only)
    if not a.sessions_only:
        write_report([out], out)
    return 0


def cmd_sfm(a) -> int:
    from reconstruction.sfm import run_sfm
    res = run_sfm(a.run, a.scene, max_features=a.max_features, pairing=a.pairing, min_shared=a.min_shared,
                  rig_check=not a.no_rig_check, prior_sigma_m=None if a.prior_sigma <= 0 else a.prior_sigma,
                  pose_source=a.pose_source)
    print(json.dumps(res, indent=1))
    return 0 if res.get("registered", 0) >= 3 else 1


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="kinetix")
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run", help="one capture session")
    r.add_argument("--scene", required=True, help="scene dir or scene.yaml (e.g. kinetix/scenes/simple_room_blocks.yaml)")
    r.add_argument("--method", default="roi_coverage", choices=METHODS)
    r.add_argument("--rig", default="dry", choices=["dry", "pool", "isaac", "mavros"])
    r.add_argument("--pool", default=None, help="pool rig: the pool folder (default $KINETIX_DATA/pools/<scene>/rings)")
    r.add_argument("--seed", type=int, default=0)
    r.add_argument("--explore", action="store_true",
                   help="room + tour: explore the room with the ZED depth first (the scene file's room and obstacles are not used, only "
                        "its take-off point and altitude band; it scores the fit)")
    r.add_argument("--config", action="append", help="extra config YAML over acquisition/config/ (e.g. acquisition/config/experiments/close.yaml; repeatable)")
    r.add_argument("--out", default=str(DATA))
    r.add_argument("overrides", nargs="*", help="key=value, e.g. planner.budget=12")
    f = sub.add_parser("sfm", help="offline SfM over a run's frames (pycolmap) + ATE and proxy completeness")
    f.add_argument("run", help="run directory")
    f.add_argument("--scene", default=None, help="scene yaml (default: kinetix/scenes/<run scene>.yaml)")
    f.add_argument("--max-features", type=int, default=8192)
    f.add_argument("--pairing", choices=["video", "covis", "exhaustive"], default=None,
                   help="image pairs to match (default: video above 150 frames, covis when the scene is known, else exhaustive)")
    f.add_argument("--min-shared", type=int, default=10, help="covis: shared proxy-surface points for a pair")
    f.add_argument("--no-rig-check", action="store_true", help="keep pairs whose relative pose disagrees with the rig poses")
    f.add_argument("--prior-sigma", type=float, default=0.5, help="sfm: position prior std (m) from the rig poses; 0 = none")
    f.add_argument("--pose-source", choices=["prior", "sfm"], default=None,
                   help="prior: triangulate at the rig poses + BA (default for tours / > 150 frames); sfm: incremental mapping")
    pm = sub.add_parser("pool", help="view pools (T0)").add_subparsers(dest="pool_cmd", required=True).add_parser(
        "make", help="the ring-stack candidates rendered by Isaac (left, right, GT depth)")
    pm.add_argument("--scene", required=True, type=Path)
    pm.add_argument("--out", default=None, help="pool folder (default $KINETIX_DATA/pools/<scene>/rings)")
    pm.add_argument("--views-only", action="store_true", help="write the request views, do not render")
    pm.add_argument("overrides", nargs="*", help="key=value over the object config (planner.rings...)")
    e = sub.add_parser("eval", help="score a run's SfM model against the scene GT (runs kinetix sfm first if needed)")
    e.add_argument("run", type=Path)
    e.add_argument("--scene", default=None)
    b = sub.add_parser("bench", help="a scenes x methods x seeds x budgets matrix -> results.csv + report.md")
    b.add_argument("--matrix", default=None, help="config/bench/<name>.yaml")
    b.add_argument("--jobs", type=int, default=4, help="cells in parallel (each: session, SfM, budget prefixes)")
    b.add_argument("--out", default=None, help="bench folder (default $KINETIX_DATA/bench/<matrix name>)")
    b.add_argument("--sessions-only", action="store_true", help="run the sessions only (T1/T2 in the kinetix container); "
                   "score them with the same command on the host afterwards")
    b.add_argument("--report", nargs="+", default=None, help="only rebuild report.md from these bench folders (several: tier comparison)")
    a = ap.parse_args(argv)
    if a.cmd == "sfm":
        return cmd_sfm(a)
    if a.cmd == "pool":
        return cmd_pool(a)
    if a.cmd == "eval":
        return cmd_eval(a)
    if a.cmd == "bench":
        return cmd_bench(a)
    a.scene = Path(a.scene)
    return cmd_run(a)
