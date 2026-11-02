"""Sanity checks on a run directory (data-format.md §3), the same live (RunLog.frame / step) and offline (`kinetix check <run>`).
A problem is a string; a run with none is consistent, not good: results are `kinetix eval`'s job."""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import yaml

BAD_STOPS = {"aborted", "stalled", "no_image", "error", "unknown", "explore_failed", "explore_incomplete"}   # the session did not finish


def _pose_ok(d: dict | None, name: str) -> list[str]:
    if d is None:
        return []
    p, q = np.asarray(d["p"], float), np.asarray(d["q"], float)
    out = [] if np.isfinite(p).all() and np.isfinite(q).all() else [f"{name} not finite"]
    if not out and abs(np.linalg.norm(q) - 1) > 1e-3:
        out.append(f"{name} quaternion norm {np.linalg.norm(q):.4f}")
    return out


def frame_problems(f: dict, run_dir: Path) -> list[str]:
    """One frames.jsonl line: finite poses, unit quaternions, the image and depth files it names exist."""
    out = _pose_ok(f["pose"], "pose") + _pose_ok(f["target"], "target") + _pose_ok(f.get("gt_pose"), "gt_pose")
    for key in ("image", "right_image", "depth"):
        if f.get(key) and not ((run_dir / f[key]).exists() and (run_dir / f[key]).stat().st_size > 0):
            out.append(f"{key} {f[key]} missing or empty")
    if f["travel_m"] < 0:
        out.append(f"travel_m {f['travel_m']} < 0")
    return [f"frame {f['idx']}: {p}" for p in out]


def step_problems(s: dict) -> list[str]:
    """One steps.jsonl line: a chosen view has a finite gain (surface_normal's steps carry none)."""
    if s.get("chosen") is not None and "gain" in s and not (s["gain"] is not None and np.isfinite(s["gain"])):
        return [f"step {s['step']}: gain {s.get('gain')} for {s['chosen']}"]
    return []


def run_problems(run_dir: Path) -> tuple[list[str], list[str]]:
    """(problems, notes) for a finished run directory. Notes are facts worth reading (stop reason, step errors, pose error)."""
    run_dir = Path(run_dir)
    if not (run_dir / "run.yaml").exists():
        return ["no run.yaml: not a run directory, or RunLog never started"], []
    head = yaml.safe_load((run_dir / "run.yaml").read_text())
    summ = head.get("summary")
    if summ is None:
        return ["run.yaml has no summary: the process died before RunLog.close (see log.txt)"], []
    read = lambda n: [json.loads(x) for x in (run_dir / n).read_text().splitlines()] if (run_dir / n).exists() else []
    frames, steps = read("frames.jsonl"), read("steps.jsonl")
    probs = [p for f in frames for p in frame_problems(f, run_dir)] + [p for s in steps for p in step_problems(s)]
    if len(frames) != summ["frames"]:
        probs.append(f"frames.jsonl has {len(frames)} lines, summary says {summ['frames']}")
    if summ["stop_reason"] in BAD_STOPS:
        probs.append(f"stop_reason {summ['stop_reason']}" + (f": {summ['error']}" if summ.get("error") else ""))
    if summ.get("warnings"):
        probs.append(f"{summ['warnings']} warnings in log.txt (grep WARN)")
    notes = [f"{head['method']} on {head['rig']['name']}, {summ['frames']} frames, stop {summ['stop_reason']}, uses_gt {head['uses_gt']}"]
    errs = [s for s in steps if s.get("error")]
    if errs:
        notes.append(f"{len(errs)} failed moves, first: step {errs[0]['step']} {errs[0]['error']}")
    for name, a, b in (("reached - target", "pose", "target"), ("pose - gt_pose", "pose", "gt_pose")):
        d = [np.linalg.norm(np.subtract(f[a]["p"], f[b]["p"])) for f in frames if f.get(a) and f.get(b)]
        if d:
            notes.append(f"{name}: median {np.median(d):.3f} m, max {np.max(d):.3f} m")
    return probs, notes
