"""report.md from one or more bench folders' results.csv (paper/benchmark.md §2): F-score@1 % vs images per method (mean ± std
over seeds), N@q against the scene's dense reference, image reduction against the best fixed and best adaptive baseline, and,
with several folders on different rigs (tiers), Kendall's tau between their method rankings.
A cell whose evaluation is invalid (registered < 50 % or alignment off, benchmark.md §3) counts as F = 0: a reconstruction that
failed has not reached any quality, and dropping it would favour methods that fail often. Failures are counted in the tables."""
from __future__ import annotations

import csv
from collections import defaultdict
from pathlib import Path

import numpy as np

FIXED = ("orbit", "orbit_rings")
SIMPLE_ADAPTIVE = ("random", "fvs", "volumetric_ig")
QS = (0.9, 0.95)
METRIC = "fscore_1pct"


def load(dirs: list[Path]) -> list[dict]:
    rows = []
    for d in dirs:
        with open(d / "results.csv") as fh:
            rows += [r for r in csv.DictReader(fh) if not r.get("error")]
    return rows


def f_of(r: dict) -> float:
    return float(r[METRIC] or 0.0) if r.get("eval_valid") == "True" else 0.0


def curves(rows: list[dict]) -> dict:
    """{(rig, scene, label): {budget: [F per seed]}} without the reference, and {(rig, scene): F_ref} (its largest budget)."""
    cur, ref = defaultdict(lambda: defaultdict(list)), {}
    for r in rows:
        key = (r["rig"], r["scene"], r["label"])
        if r["label"] == "reference":
            b = int(r["budget"])
            if b >= ref.get((r["rig"], r["scene"]), (0, 0))[0]:
                ref[(r["rig"], r["scene"])] = (b, f_of(r))
        else:
            cur[key][int(r["budget"])].append(r)
    return cur, {k: v[1] for k, v in ref.items()}


def n_at_q(budgets: list[int], F: list[float], target: float) -> float | None:
    """Smallest N with F(N) >= target, linear between the measured budgets; None if never reached."""
    for k, (b, f) in enumerate(zip(budgets, F)):
        if f >= target:
            if k == 0:
                return float(b)
            b0, f0 = budgets[k - 1], F[k - 1]
            return b0 + (b - b0) * (target - f0) / (f - f0)
    return None


def kendall_tau(a: list, b: list) -> float:
    """Kendall's tau-a between two rankings of the same items (lists ordered best first)."""
    items = [x for x in a if x in b]
    ra, rb = {x: i for i, x in enumerate(items)}, {x: i for i, x in enumerate([x for x in b if x in items])}
    n, s = len(items), 0
    for i in range(n):
        for j in range(i + 1, n):
            x, y = items[i], items[j]
            s += np.sign(ra[x] - ra[y]) * np.sign(rb[x] - rb[y])
    return float(s / (n * (n - 1) / 2)) if n > 1 else float("nan")


def fmt(x, nd=1):
    return "–" if x is None else f"{x:.{nd}f}"


def write_report(dirs: list[Path], out: Path) -> Path:
    rows = load(dirs)
    cur, ref = curves(rows)
    lines = [f"# Benchmark report\n", f"Sources: {', '.join(str(d) for d in dirs)}. Metric: F-score at 1 % of the ROI diagonal (sparse SfM "
             "points vs gt_observable / gt_mesh), mean ± std over seeds; an invalid evaluation counts as 0 (failures in brackets). "
             "N@q: images to reach q x the dense reference's F (orbit_rings over the whole pool); – = not reached within the budgets. "
             "Rows marked † read GT (uses_gt: the oracle, or a depth method on a rendered-depth rig).\n"]
    by_tier = defaultdict(dict)                        # rig -> {label: mean N@0.9 over scenes / mean F at the largest budget}
    for (rig, scene) in sorted({k[:2] for k in cur}):
        labels = sorted({k[2] for k in cur if k[:2] == (rig, scene)})
        budgets = sorted({b for lab in labels for b in cur[(rig, scene, lab)]})
        fref = ref.get((rig, scene))
        lines.append(f"\n## {scene} ({rig})\n\nF_ref = {fmt(fref, 3)}\n")
        lines.append("| method | " + " | ".join(f"N={b}" for b in budgets) + " | " + " | ".join(f"N@{q}" for q in QS) + " | reg. frac @max |")
        lines.append("|---" * (len(budgets) + len(QS) + 2) + "|")
        nq = {}
        for lab in labels:
            c = cur[(rig, scene, lab)]
            gt = any(r.get("uses_gt") == "True" for rs in c.values() for r in rs)
            means, cells_ = [], []
            for b in budgets:
                rs = c.get(b, [])
                F = [f_of(r) for r in rs]
                fails = sum(r.get("eval_valid") != "True" for r in rs)
                means.append(float(np.mean(F)) if F else None)
                cells_.append(f"{np.mean(F):.3f} ± {np.std(F):.3f}" + (f" ({fails})" if fails else "") if F else "–")
            qs = [n_at_q([b for b, m in zip(budgets, means) if m is not None], [m for m in means if m is not None], q * fref)
                  if fref else None for q in QS]
            nq[lab] = qs[0]
            reg = [float(r["registered_frac"] or 0) for r in c.get(max(c), [])]
            lines.append(f"| {lab}{' †' if gt else ''} | " + " | ".join(cells_) + " | " + " | ".join(fmt(x) for x in qs)
                         + f" | {fmt(np.mean(reg) if reg else None, 2)} |")
            by_tier[rig].setdefault(lab, []).append((qs[0], means[-1]))
        fixed = [nq[x] for x in FIXED if nq.get(x)]
        adapt = [nq[x] for x in SIMPLE_ADAPTIVE if nq.get(x)]
        if fixed or adapt:
            lines.append("\nImage reduction at q = 0.9 (1 − N@q / N@q of the best fixed | best simple adaptive baseline):\n")
            for lab, v in sorted(nq.items()):
                if v and lab not in FIXED + SIMPLE_ADAPTIVE:
                    lines.append(f"- {lab}: {fmt(100 * (1 - v / min(fixed)), 0) if fixed else '–'} % | "
                                 f"{fmt(100 * (1 - v / min(adapt)), 0) if adapt else '–'} %")
    if len(by_tier) > 1:
        lines.append("\n## Tier transfer\n\nScenes and methods every tier ran; ranking by mean F at the largest budget, mean over those "
                     "scenes; Kendall's tau between tiers.\n")
        common = set.intersection(*({(r["scene"], r["label"]) for r in rows if r["rig"] == rig and r["label"] != "reference"}
                                    for rig in by_tier))
        scenes = sorted({s for s, _ in common})
        labs = sorted({lab for _, lab in common if all((s, lab) in common for s in scenes)})
        mean_f = lambda rig, lab: np.mean([np.mean([f_of(r) for r in cur[(rig, s, lab)][max(cur[(rig, s, lab)])]]) for s in scenes])  # noqa: E731
        lines.append(f"Scenes: {', '.join(scenes)}.\n")
        lines.append("| method | " + " | ".join(sorted(by_tier)) + " |")
        lines.append("|---" * (len(by_tier) + 1) + "|")
        for lab in labs:
            lines.append(f"| {lab} | " + " | ".join(f"{mean_f(rig, lab):.3f}" for rig in sorted(by_tier)) + " |")
        lines.append("")
        rank = {rig: sorted(labs, key=lambda lab, rig=rig: -mean_f(rig, lab)) for rig in by_tier}
        rigs = sorted(rank)
        for rig in rigs:
            lines.append(f"- {rig}: {' > '.join(rank[rig])}")
        for i in range(len(rigs)):
            for j in range(i + 1, len(rigs)):
                lines.append(f"- tau({rigs[i]}, {rigs[j]}) = {kendall_tau(rank[rigs[i]], rank[rigs[j]]):.2f}")
    out.mkdir(parents=True, exist_ok=True)
    (out / "report.md").write_text("\n".join(lines) + "\n")
    print(f"{out / 'report.md'}")
    return out / "report.md"
