"""Compare flights by their SfM result: which acquisition setting gave the better reconstruction for its cost.

    uv run python -m acquisition.compare                    every data/<run>/ with an sfm/metrics.json
    uv run python -m acquisition.compare data/RUN_A data/RUN_B --sort ate --csv out.csv

One row per run: the experiment (acquisition/config/experiments/*.yaml), flight cost (images, metres, minutes) and SfM quality
(registered, points, track length, ATE vs ground truth, proxy completeness). Below the table: every config key whose value
differs between the listed runs, so a difference in the result can be traced to a difference in the settings. Reads the run
directories only (run.yaml + sfm/metrics.json), never imports reconstruction.
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import yaml

from kinetix.config import DATA

# column -> (path in run.yaml summary | sfm metrics.json, lower is better)
COLS = {
    "frames": ("summary.frames", None), "travel_m": ("summary.travel_m", True), "wall_min": ("summary.wall_s", True),
    "registered": ("sfm.registered_frac", False), "models": ("sfm.models", True), "points3D": ("sfm.points3D", False),
    "track": ("sfm.mean_track_length", False), "reproj_px": ("sfm.mean_reproj_error_px", True),
    "ate": ("sfm.ate_vs_gt.rmse_m", True), "ate_ekf2": ("sfm.ate_vs_ekf2.rmse_m", True),
    "complete": ("sfm.completeness_proxy.all", False), "walls": ("sfm.completeness_proxy.walls", False),
    "floor": ("sfm.completeness_proxy.floor", False), "ceiling": ("sfm.completeness_proxy.ceiling", False),
    "sfm_min": ("sfm.time_s.total", True),
}


def _get(d, path):
    for k in path.split("."):
        if not isinstance(d, dict) or k not in d:
            return None
        d = d[k]
    return d


def _flat(d: dict, pre: str = "") -> dict:
    out = {}
    for k, v in d.items():
        out.update(_flat(v, f"{pre}{k}.") if isinstance(v, dict) else {f"{pre}{k}": v})
    return out


def load_run(run: Path) -> dict:
    head = yaml.safe_load((run / "run.yaml").read_text())
    m = run / "sfm" / "metrics.json"
    src = {"summary": head.get("summary") or {}, "sfm": json.loads(m.read_text()) if m.exists() else {}}
    row = {"run": run.name, "experiment": (head.get("config") or {}).get("experiment", "-")}
    for col, (path, _) in COLS.items():
        v = _get(src, path)
        row[col] = round(v / 60, 1) if col.endswith("_min") and v is not None else v
    return {"row": row, "config": _flat(head.get("config") or {})}


def config_diff(runs: list[dict]) -> dict[str, list]:
    keys = sorted(set().union(*(r["config"] for r in runs)))
    vals = {k: [r["config"].get(k) for r in runs] for k in keys}
    return {k: v for k, v in vals.items() if any(x != v[0] for x in v)}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="acquisition.compare", description=__doc__.split("\n")[0])
    ap.add_argument("runs", nargs="*", help="run dirs (default: every data/<run>/ with sfm/metrics.json)")
    ap.add_argument("--sort", default="ate", choices=list(COLS), help="column to sort by, best first (default ate)")
    ap.add_argument("--csv", help="also write the table to this CSV file")
    a = ap.parse_args(argv)
    dirs = [Path(r) for r in a.runs] or sorted(p.parent.parent for p in DATA.glob("*/sfm/metrics.json"))
    if not dirs:
        print(f"no runs with sfm/metrics.json in {DATA}", file=sys.stderr)
        return 1
    runs = [load_run(d) for d in dirs]
    low = COLS[a.sort][1] is not False
    runs.sort(key=lambda r: (r["row"][a.sort] is None, (r["row"][a.sort] or 0) * (1 if low else -1)))
    rows = [r["row"] for r in runs]
    cols = list(rows[0])
    fmt = lambda v: "-" if v is None else f"{v:.3g}" if isinstance(v, float) else str(v)    # noqa: E731
    print("| " + " | ".join(cols) + " |\n|" + "---|" * len(cols))
    for row in rows:
        print("| " + " | ".join(fmt(row[c]) for c in cols) + " |")
    diff = config_diff(runs)
    if diff:
        print(f"\nconfig keys that differ ({len(diff)}), values in table order:")
        for k, v in diff.items():
            print(f"  {k}: " + " | ".join(fmt(x) for x in v))
    if a.csv:
        with open(a.csv, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=cols)
            w.writeheader()
            w.writerows(rows)
    return 0


if __name__ == "__main__":
    sys.exit(main())
