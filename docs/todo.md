# Todo

Current phase: **K0 → K1**. Phase definitions and exit tests: [roadmap.md](roadmap.md).

## Done
- [x] K0 doc set (architecture, interfaces, data-format, roadmap, testing, benchmark, decisions, setup) — 2026-10-08
- [x] K0 Python env (`pyproject.toml`, `uv.lock`), `scripts/check_env.py` passes on workstation B — 2026-10-08
- [x] Mono-vs-stereo study (`docs/research/mono-vs-stereo.md`). Plan updated: ADR-K07 sensing axes, stereo in pools/interfaces/benchmark/roadmap — 2026-10-08
- [x] Novelty search (`docs/research/reports/Kinetix novelty search.md`) and plan updated: contributions C1–C5, baselines fvs/volumetric_ig/fisherrf/ma_scvp, Tammes pool, N@q metrics, anchoring ablation, `docs/publication-plan.md` (ADR-K08) — 2026-10-08

## Next (K1)
- [ ] Review the doc set; answer Q1 (mount pitch), Q2 (object set), Q6 (T0 stereo source): architecture.md §12
- [ ] Review the contribution wording C1–C5 (`publication-plan.md` §2) and the venue plan (§5)
- [x] References cross-checked (Crossref + arXiv), DOI links on all 49 entries; fixed [3] [11] [20] [21] [27] [34] [45] — 2026-10-08
- [ ] Check asset licences (SimReady, Objaverse++, GSO, OmniObject3D) before fixing the object set
- [ ] Arrange the real-object scanner for K6 GT (Q4), needed by G3
- [ ] Create `$KINETIX_DATA` and `.env`
- [ ] `src/kinetix/core/` (types, geometry, config, runlog) + unit tests; switch `tool.uv.package = true`
- [ ] First scene: pick the object, build `scene.usd` / `gt_mesh.ply` / `scene.yaml`
- [ ] Q3: bisg change so the sim container can read `$KINETIX_DATA/scenes` (done in bisg_isaac)
- [ ] `sim/render_server.py` batch mode + `kinetix pool make`
- [ ] `rig/pool.py`, `planner/loop.py` + orbit/grid/random, `recon/state.py` v1, `kinetix run`
- [ ] `tests/fixtures/pool_tiny/` (git-lfs)

## Watch list (check monthly; novelty sweeps at G2, G4, G5)
- ObjView-Bench code + object pool release (arXiv 2605.10707). It may become the default object protocol
- ActMVS code (GitHub TrickyGo/ActMVS): a possible monocular baseline
- Hestia / VIN-NBV code release (object-NBV baselines)
- AREA3D (CVPR 2026, arXiv 2512.05131): backbone and sensors not yet verified
- New arXiv preprints on Isaac Sim + PX4 NBV, monocular/foundation-depth NBV, or mono-vs-stereo active reconstruction
