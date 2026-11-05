# config/bench

Benchmark matrices for `kinetix bench` (code: `bench/matrix.py`). Each file is one experiment: scenes × methods × budgets × seeds.
They are **not** acquisition settings (those are `acquisition/config/`); a matrix only picks what to run and passes overrides to it.

```
uv run kinetix bench --matrix config/bench/<name>.yaml [--jobs N]     # results: $KINETIX_DATA/bench/<name>/report.md
```

| Matrix | Question | Rig | Depends on |
|---|---|---|---|
| `baselines.yaml` | K2 main table: 9 methods on `office_buddha`, sparse SfM scoring. | `pool` (T0) | `kinetix pool make --scene <s>` for each scene |
| `dense.yaml` | Same sessions as `baselines`, scored on dense MVS points instead of sparse (V1). | `pool` | `baselines` run first (`runs_from: baselines`: same images, paired) |
| `oracle.yaml` | GT-surface oracle with `k_min` 2 vs 3, fresh sessions, dense scoring (V4); compare with `dense` rows. | `pool` | none |
| `tiers_t1.yaml` | T1 half of the T0 <-> T1 check: `office_buddha` rendered live by Isaac. | `isaac` | render server running; compare with `baselines` (steps in the file header) |

Keys: `name` (the output folder), `rig`, `scenes`, `budgets`, `seeds`, `methods` (a name, or `{label, method, overrides}`), `overrides` (`key=value` for every cell, as on the `kinetix run` command line), `reference` (dense reference used for scoring), `densify` and `runs_from` (dense scoring only).

New experiment: copy the closest file, change `name` and the lists. Not here yet but named in the docs: `smoke.yaml`, `ablation.yaml`, `sensing.yaml`.
