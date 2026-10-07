# Todo

Current phase: **K0 → K1**. Phase definitions and exit tests: [roadmap.md](roadmap.md).

## Done
- [x] K0 doc set (architecture, interfaces, data-format, roadmap, testing, benchmark, decisions, setup) — 2026-10-08
- [x] K0 Python env (`pyproject.toml`, `uv.lock`), `scripts/check_env.py` passes on workstation B — 2026-10-08

## Next (K1)
- [ ] Review the doc set; answer Q1 (mount pitch) and Q2 (object set): architecture.md §12
- [ ] Create `$KINETIX_DATA` and `.env`
- [ ] `src/kinetix/core/` (types, geometry, config, runlog) + unit tests; switch `tool.uv.package = true`
- [ ] First scene: pick the object, build `scene.usd` / `gt_mesh.ply` / `scene.yaml`
- [ ] Q3: bisg change so the sim container can read `$KINETIX_DATA/scenes` (done in bisg_isaac)
- [ ] `sim/render_server.py` batch mode + `kinetix pool make`
- [ ] `rig/pool.py`, `planner/loop.py` + orbit/grid/random, `recon/state.py` v1, `kinetix run`
- [ ] `tests/fixtures/pool_tiny/` (git-lfs)
