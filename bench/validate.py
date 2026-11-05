"""V3 proxy validity (methodology.md §4): does the coverage a planner optimises predict what the evaluation measures?

For every run of a bench folder, the frames are replayed at their GT poses into the occlusion-aware GT coverage model
(`CoverageState` on gt_observable + `MeshOcclusion` on gt_mesh), and its coverage after each budget's first N frames is set against
that row's measured completeness ratio and F-score (Spearman rho over all rows). `--k-min` sets the views a point needs.
Usage: `uv run python -m bench.validate <bench dir> [--k-min 2 3]` -> <bench dir>/validate.json."""
from __future__ import annotations

import argparse
import json
from collections import defaultdict
from pathlib import Path

import numpy as np
import yaml

from bench.report import f_of, load
from kinetix.config import ROOT, load_config, load_scene
from kinetix.coverage import CoverageState, MeshOcclusion, gt_surface
from kinetix.types import Frame, Intrinsics, Pose


def spearman(a, b) -> float:
    def rank(x):
        x = np.asarray(x, float)
        order = np.argsort(x, kind="stable")
        r = np.empty(len(x))
        r[order] = np.arange(len(x))
        for v in np.unique(x):                         # ties: average rank
            r[x == v] = r[x == v].mean()
        return r
    ra, rb = rank(a), rank(b)
    return float(np.corrcoef(ra, rb)[0, 1]) if len(ra) > 2 and ra.std() and rb.std() else float("nan")


def replay(run: Path, scene_file: Path, budgets: list[int], k_min: int, cc: dict) -> dict:
    """{N: (coverage, min overlap of frames 2..N with what came before)} at GT poses."""
    scene = load_scene(scene_file)
    frames = [json.loads(line) for line in (run / "frames.jsonl").read_text().splitlines() if line.strip()]
    K = Intrinsics(**{k: frames[0]["K"][k] for k in ("width", "height", "fx", "fy", "cx", "cy")})
    s = CoverageState.for_scene(scene, K, {**cc, "k_min": k_min}, gt_surface(scene, cc["spacing_m"]), MeshOcclusion(scene.gt_mesh))
    out, worst = {}, 1.0
    for i, f in enumerate(frames):
        g = f.get("gt_pose") or f["pose"]
        pose = Pose(np.array(g["p"], float), np.array(g["q"], float))
        if i:
            worst = min(worst, s.overlap(pose))
        s.add(Frame(i, 0.0, None, K, pose, pose, None, 0.0))
        if i + 1 in budgets:
            out[i + 1] = (s.coverage, worst)
    return out


def validate(bench: Path, k_mins=(2, 3)) -> dict:
    rows = [r for r in load([bench]) if r["label"] != "reference"]
    cc = load_config([])["coverage"]
    by_run = defaultdict(list)
    for r in rows:
        by_run[(r["run_id"], r["scene"])].append(r)
    src = yaml.safe_load((bench / "matrix.yaml").read_text()).get("runs_from")
    runs = bench.parent / src / "runs" if src else bench / "runs"   # a runs_from bench scores another bench's sessions
    res = {}
    for k in k_mins:
        cov, comp, F, ovl, reg = [], [], [], [], []
        for (run_id, scene), rs in by_run.items():
            run = runs / run_id
            sf = Path(yaml.safe_load((run / "run.yaml").read_text()).get("scene_path") or ROOT / "kinetix" / "scenes" / f"{scene}.yaml")
            pred = replay(run, sf, sorted({int(r["n_images"]) for r in rs}), k, cc)
            for r in rs:
                c, o = pred.get(int(r["n_images"]), (np.nan, np.nan))
                cov.append(c); ovl.append(o)
                comp.append(float(r["completeness_ratio_1pct"] or 0) if r["eval_valid"] == "True" else 0.0)
                F.append(f_of(r)); reg.append(float(r["registered_frac"] or 0))
        res[f"k_min_{k}"] = {"rows": len(cov), "rho_coverage_completeness": round(spearman(cov, comp), 3),
                             "rho_coverage_F": round(spearman(cov, F), 3),
                             "rho_min_overlap_registered": round(spearman(ovl, reg), 3)}
    (bench / "validate.json").write_text(json.dumps(res, indent=1))
    return res


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("bench", type=Path)
    ap.add_argument("--k-min", type=int, nargs="+", default=[2, 3])
    a = ap.parse_args(argv)
    print(json.dumps(validate(a.bench, a.k_min), indent=1))


if __name__ == "__main__":
    main()
