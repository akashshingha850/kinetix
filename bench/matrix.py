"""`kinetix bench --matrix config/bench/<name>.yaml` (paper/benchmark.md, data-format.md §4): every scene x method x seed cell is
one session at the largest budget, its SfM, and the score of each budget's prefix (a greedy or fixed-order session's first N
views are what a budget-N session takes; surface_normal plans to its budget and is not prefix-scored). Per scene, a dense
reference (`orbit_rings` over the whole pool) gives F_ref for N@q. A cell's rows are kept in cells/<cell>.json, so a stopped
bench resumes where it was; results.csv is rebuilt from them.
`--sessions-only` runs the sessions and stops (cells/<cell>.run names the run): the T1/T2 rigs need ROS (the kinetix container),
the scoring needs open3d (the host venv), so a T1 bench is run twice with the same matrix, once in each."""
from __future__ import annotations

import argparse
import csv
import json
import traceback
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

import yaml

from kinetix.config import SCENES, kinetix_data, load_config

COLUMNS = ("matrix", "rig", "scene", "label", "method", "seed", "budget", "n_images", "run_id", "uses_gt", "stop_reason", "travel_m",
           "registered_frac", "ate_cm", "n_points", "accuracy_mm", "completeness_mm", "chamfer_mm", "precision_1pct",
           "fscore_0.5pct", "fscore_1pct", "completeness_ratio_1pct", "eval_valid", "error")


def scene_path(name: str) -> Path:
    """A scene by name: $KINETIX_DATA/scenes/<name>, else kinetix/scenes/<name>.yaml; or a path."""
    for p in (Path(name), kinetix_data() / "scenes" / name, SCENES / f"{name}.yaml"):
        if (p / "scene.yaml").exists() or (p.suffix == ".yaml" and p.exists()):
            return p
    raise SystemExit(f"scene {name!r}: not in {kinetix_data() / 'scenes'} nor {SCENES}")


def cells(m: dict) -> list[dict]:
    rig, budgets = m.get("rig", "pool"), sorted(m["budgets"])
    out = []
    for scene in m["scenes"]:
        if m.get("reference"):
            rc = load_config(kind="object")["planner"]["rings"]
            n = len(rc["radii"]) * len(rc["heights"]) * rc["n_az"]
            fr = m["reference"].get("fractions", [1.0])
            out.append(dict(scene=scene, label="reference", method=m["reference"].get("method", "orbit_rings"), seed=0, rig=rig,
                            densify=m.get("densify"), runs_from=m.get("runs_from"),
                            overrides=list(m.get("overrides", [])), budgets=sorted({max(3, round(f * n)) for f in fr})))
        for spec in m["methods"]:
            spec = {"label": spec, "method": spec} if isinstance(spec, str) else {"label": spec["label"], **spec}
            for seed in m["seeds"]:
                out.append(dict(scene=scene, label=spec["label"], method=spec["method"], seed=seed, rig=rig,
                                densify=m.get("densify"), runs_from=m.get("runs_from"),
                                overrides=list(m.get("overrides", [])) + list(spec.get("overrides", [])),
                                budgets=[budgets[-1]] if spec["method"] == "surface_normal" else budgets))
    return out


def cell_id(c: dict) -> str:
    return f"{c['scene']}__{c['label']}__s{c['seed']}"


def run_cell(c: dict, out: Path, name: str, sessions_only: bool = False) -> list[dict]:
    """One session at max(budgets) + SfM + a score per budget. Never raises: an error becomes one row with `error`."""
    from kinetix.cli import cmd_run
    f = out / "cells" / f"{cell_id(c)}.json"
    marker = f.with_suffix(".run")
    if c.get("runs_from"):                              # score another bench's sessions (same runs: paired sparse vs dense)
        src = kinetix_data() / "bench" / c["runs_from"]
        marker = src / "cells" / marker.name
    if f.exists():
        return json.loads(f.read_text())
    base = dict(matrix=name, rig=c["rig"], scene=c["scene"], label=c["label"], method=c["method"], seed=c["seed"])
    try:
        sp = scene_path(c["scene"])
        src_rows = marker.with_suffix(".json")
        if c.get("runs_from") and not marker.exists() and not src_rows.exists():
            raise RuntimeError(f"{marker.stem}: no such session in bench {c['runs_from']}")
        if c.get("runs_from") and not marker.exists():  # a finished cell keeps its run id in its rows
            run = marker.parent.parent / "runs" / next(r["run_id"] for r in json.loads(src_rows.read_text()) if r.get("run_id"))
        elif marker.exists():                           # the session ran earlier (--sessions-only): score it
            run = marker.parent.parent / "runs" / marker.read_text().strip()
        else:
            a = argparse.Namespace(scene=sp, method=c["method"], rig=c["rig"], seed=c["seed"], explore=False, config=None, pool=None,
                                   out=str(out / "runs"), overrides=c["overrides"] + [f"planner.budget={max(c['budgets'])}"])
            cmd_run(a)
            run = a.run_dir
            marker.write_text(run.name + "\n")
        if sessions_only:
            return []
        from reconstruction.evaluate import evaluate, evaluate_prefix
        from reconstruction.sfm import run_sfm
        head = yaml.safe_load((run / "run.yaml").read_text())
        summ = head["summary"]
        n = summ["frames"]
        if n < 3:
            raise RuntimeError(f"{n} frames ({summ['stop_reason']})")
        if not (run / "sfm" / "metrics.json").exists():
            run_sfm(run, sp, pairing="covis", pose_source="sfm")
        dz = c.get("densify")
        full = evaluate(run, sp, densify=dz)
        rows = []
        for b in c["budgets"]:
            res = full if b >= n else evaluate_prefix(run, b, sp, densify=dz)
            rows.append({**base, "budget": b, "n_images": min(b, n), "run_id": run.name, "uses_gt": head["uses_gt"],
                         "stop_reason": summ["stop_reason"], "travel_m": summ["travel_m"], **flat(res)})
    except Exception as e:                              # noqa: BLE001 - a failed cell must not stop an unattended bench
        rows = [{**base, "error": f"{type(e).__name__}: {e}"}]
        (out / "cells" / f"{cell_id(c)}.err").write_text(traceback.format_exc())
        return rows                                     # not cached: a rerun retries it
    f.write_text(json.dumps(rows))
    return rows


def flat(res: dict) -> dict:
    g = lambda d, k: (res.get(d) or {}).get(k)          # noqa: E731
    return {"registered_frac": res.get("registered_frac"), "ate_cm": res.get("ate_cm"), "n_points": res.get("n_points"),
            "accuracy_mm": res.get("accuracy_mm"), "completeness_mm": res.get("completeness_mm"), "chamfer_mm": res.get("chamfer_mm"),
            "precision_1pct": g("precision", "tau_1pct"), "fscore_0.5pct": g("fscore", "tau_0.5pct"),
            "fscore_1pct": g("fscore", "tau_1pct"), "completeness_ratio_1pct": g("completeness_ratio", "tau_1pct"),
            "eval_valid": res.get("eval_valid")}


def run_matrix(matrix: Path, jobs: int = 4, out: Path | None = None, sessions_only: bool = False) -> Path:
    m = yaml.safe_load(matrix.read_text())
    out = out or kinetix_data() / "bench" / m["name"]
    (out / "cells").mkdir(parents=True, exist_ok=True)
    (out / "matrix.yaml").write_text(matrix.read_text())
    todo = cells(m)
    print(f"bench {m['name']}: {len(todo)} cells -> {out}", flush=True)
    rows = []
    with ProcessPoolExecutor(max_workers=jobs) as ex:
        futs = {ex.submit(run_cell, c, out, m["name"], sessions_only): c for c in todo}
        for k, fut in enumerate(as_completed(futs), 1):
            r = fut.result()
            rows += r
            print(f"[{k}/{len(todo)}] {cell_id(futs[fut])}: " + (r[-1].get('error') or f"F@1% {r[-1].get('fscore_1pct')}" if r else "session done"),
                  flush=True)
    if sessions_only:
        return out
    write_csv(out / "results.csv", rows)
    (out / "runs.txt").write_text("".join(sorted({r["run_id"] + "\n" for r in rows if r.get("run_id")})))
    return out


def write_csv(path: Path, rows: list[dict]) -> None:
    with open(path, "w", newline="") as fh:
        w = csv.DictWriter(fh, COLUMNS, extrasaction="ignore")
        w.writeheader()
        for r in sorted(rows, key=lambda r: (r["scene"], r["label"], r["seed"], r.get("budget") or 0)):
            w.writerow(r)
