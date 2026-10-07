# Benchmark protocol

What is compared, how it is measured, and what result supports the paper's claim
("50–80 % fewer images at equal quality", [research/idea.md](research/idea.md)). The harness is
`kinetix bench` ([interfaces.md](interfaces.md) §6), and the outputs are in [data-format.md](data-format.md) §4.

## 1. Experimental axes

| Axis | Values |
|---|---|
| Scenes | K2: 3 sim objects. Paper: 8–10 sim objects spanning texture (rich/poor), geometry (convex/concave/thin parts), size (0.3–2 m), material (matte/glossy), plus 3 real objects (K6) |
| Methods | `orbit_1ring`, `orbit_3ring`, `grid_hemisphere` (CPP-style), `random`, `coverage_oracle` (upper bound, uses GT), `kinetix_combo` + ablations |
| Budgets | images N ∈ {10, 20, 40, 80} (fixed-budget mode) and `stop: gain_plateau` (free mode) |
| Tier | T0 for the full matrix. A T1 and T2 subset for transfer. T3 for real |
| Seeds | 3 (K2–K3), 5 (paper). The seed controls the seed-view phase, random tie-breaks and pose noise |

Fixed-sequence baselines take the first N poses of an evenly spread sequence (a golden-angle
ordering on the rings), so every prefix is a fair N-image baseline.

## 2. Reference and headline metric

- **Dense reference** per scene: every candidate view of the pool (~200+), reconstructed with the same
  offline pipeline. Its F-score@1 % is `F_ref`.
- **Headline: images-to-quality** `N@q` = the smallest N at which a method reaches `q · F_ref`
  (q = 0.95), interpolated on the budget curve. Image reduction = `1 − N@q(kinetix) / N@q(baseline)`,
  reported against the best baseline, not the weakest.
- **Efficiency curve**: F-score@1 % vs N per method (mean ± std over seeds), one panel per scene.

## 3. Metrics (`offline/metrics.py`)

All geometric metrics are computed after Sim3 alignment of the reconstruction to GT camera centres,
cropped to the ROI box, against `gt_observable.ply` (GT surface visible from some feasible pose).
Thresholds are relative to the ROI diagonal `D`, so objects of different sizes are comparable.

| Metric | Definition |
|---|---|
| accuracy | mean distance recon → GT (mm) |
| completeness | mean distance GT → recon (mm) |
| Chamfer | (accuracy + completeness) / 2 |
| F-score@τ | harmonic mean of precision/recall at τ ∈ {0.5 %, 1 %} of D |
| completeness ratio@τ | fraction of GT points within τ of the recon |
| registered fraction | registered frames / captured frames |
| ATE | RMSE of camera centres after Sim3 (cm) |
| NVS | PSNR / SSIM / LPIPS on the pool's `test` views (3DGS densifier only) |
| cost | images, travel distance (m), flight time (T2/T3, s), planning time per step (s) |

Sanity guards are run by `evaluate.py` on every evaluation: if alignment residual > 5 % of D, or
registered fraction < 50 %, the run is marked `eval_valid: false` and kept out of means (reported as failure count).

## 4. Ablations (K3+)

| Ablation | Values |
|---|---|
| scorer terms | sfm_uncertainty only · depth_frontier only · combo |
| pose source | `sfm` · `prior` · `gt` |
| travel cost λ | 0 · default · 4× default |
| seed views | 3 · 6 · 12 |
| depth model | DA-V2 small · base · large |
| camera mount pitch | 0° · −20° · −35° (Q1) |
| pose noise (T0) | 0 · 2 cm/1° · 5 cm/2° (emulates EKF2 error) |

## 5. Reporting rules

- Every number in the paper comes from a `results.csv` produced by `kinetix bench`. No hand-edited tables.
- Report mean ± std over seeds, plus failure counts. Oracle rows are visually marked as upper bounds.
- `run.yaml` provenance (git sha, versions, host, GPU) is kept for every run behind a reported number.
- Timings are reported with the host (workstation B, RTX 6000 Ada). Sim-time vs wall-time is stated for T2.
