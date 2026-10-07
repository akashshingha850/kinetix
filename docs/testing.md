# Testing

Test levels follow the execution tiers ([architecture.md](architecture.md) §3). A lower level never
needs the infrastructure of a higher one. pytest markers are declared in `pyproject.toml`.

| Level | Marker | Needs | Runtime | When |
|---|---|---|---|---|
| Unit | (none) | host venv | < 30 s total | every change |
| Component | `t0` | host venv + `tests/fixtures/` | < 2 min | every change |
| Render integration | `t1` | `bisg/sim:6.0.0`, GPU | ~5 min | changes to `sim/` or `rig/isaac_render.py` |
| Flight integration | `t2` | bisg stack `./bisg all headless` | ~15 min | changes to `rig/mavros.py`, before a hardware session |
| Hardware bench | `t3` | real drone, tether | manual checklist | before every flight day |
| Bench regression | — | pools on `$KINETIX_DATA` | ~10 min | before merging planner/recon/offline changes |

```bash
uv run pytest                                  # unit + t0 (default: excludes t1/t2/t3 via addopts, K1)
uv run pytest -m t1                            # render integration
uv run pytest -m t2                            # flight integration (sim must be up)
uv run kinetix bench --matrix config/bench/smoke.yaml --check   # bench regression vs golden
```

## 1. Unit tests (`tests/unit/`): pure functions, no I/O beyond tmp dirs

| Module | What is pinned |
|---|---|
| `core.geometry` | SE3 compose/inverse round-trip, quaternion normalisation, look-at, ENU ↔ optical ↔ COLMAP `cam_from_world` ↔ OpenGL (nerfstudio) round-trips, 4-DoF pose with a fixed mount pitch |
| `core.config` | layering order, `key=value` override types, unknown key → error, registry lookup by name |
| `core.runlog` | write → read round-trip, unknown keys ignored, `format` version check |
| `recon.align` | Sim3/Umeyama recovers a random transform from noisy correspondences (≤ 1e-6 noise-free) |
| `planner.sampler` | all samples within standoff/altitude, pitch = mount pitch, yaw points at the ROI |
| `planner.constraints` | boxes, standoff shell, segment-vs-box intersection on hand-made cases |
| `planner.select` | λ = 0 → argmax gain. Big λ → nearest. Ties broken by seed. |
| `planner.scorers.*` | on synthetic `ReconState` fakes: a view of only well-triangulated points scores below a view of a gap. Random scorer is seed-deterministic. |
| `offline.metrics` | identical clouds → 0 / F = 1. Sphere offset by d → Chamfer d. Empty recon → completeness = 0, no crash. |
| architecture | import rule (architecture.md §4): `planner/` and `core/` import no `rclpy` / adapter / `offline` (AST scan) |
| monocular rule | `rig/mavros.py` subscription list contains no forbidden topic (interfaces.md §4) |

## 2. Component tests (`tests/component/`, `t0`): real libraries, fixture data

Fixture `tests/fixtures/pool_tiny/`: 24 views of one scene, 320×240, with GT depth and `gt_mesh.ply`
(decimated). Generated once by `kinetix pool make` (K1) and committed with git-lfs. Regenerating it
is a deliberate act, recorded in the fixture's `pool.yaml`.

- `ReconState`: adding 8 orbit frames registers ≥ 7. Aligned camera centres are within 2 % of the ROI diagonal of the pool poses.
- Session loop with `PoolRig`, `orbit` and `random`, budget 8: the run dir passes the schema check, the run is deterministic per seed (same `chosen` sequence twice), and `stop_reason` is recorded.
- `kinetix eval` on that run with `densify=none` (sparse points only) writes a valid `metrics.json`.
- `depth_prior` (marker `gpu`): DA-V2 runs on one fixture image, and the scale fit to sparse points has a residual below a threshold.

## 3. Integration tests (`tests/integration/`)

**`t1` render.** Starts `render_server.py` in `bisg/sim:6.0.0` on the fixture scene (headless).
- Batch: 4 poses → 4 images + depth. Depth at the image centre matches a raycast on `gt_mesh.ply` within 1 cm.
- Serve: a request round-trip under 3 s warm, and the reply stamp equals the request id.

**`t2` flight.** Preconditions: `SIM_SCENARIO=<kinetix scene> ./bisg all headless` is up, and `./bisg smoke` passes.
- `MavrosRig.start()` arms, takes off and reaches the standby pose. `capture_at` on 3 poses: image
  received, pose error vs `state/pose` < 0.10 m / 3°, image stamp after settle, K equals `camera_info`.
- `finish()` lands. The test always lands in a `finally`.
- Margins allow for 0.3–0.5× real time (bisg migration-errors M8). All timeouts are in sim time.

## 4. Bench regression

`config/bench/smoke.yaml`: 1 scene × {orbit, kinetix_combo} × budget 20 × 1 seed on T0.
`--check` compares with `tests/golden/smoke.json`: F-score must not drop by more than 0.02, and the
image count to reach the target must not rise. The golden file is updated only in a commit that
explains why.

## 5. Hardware bench checklist (T3, K6)

Before any untethered flight; the results go into `runs/<id>/log.txt` of a dry run:

1. bisg `./bisg zed check --no-gt` passes on the drone (camera stream, geometry).
2. `camera_info` resolution equals the config, and `K` is the unit's factory calibration (not the sim's).
3. Image latency: stamp vs receive < 100 ms. Pose/image time offset measured (LED or tap test).
4. Tethered hover: `capture_at` on 4 poses around an empty ROI, pose error < 0.10 m / 3°, settle < 3 s.
5. Workspace box measured in the real room and written into the real `scene.yaml`. No-fly boxes for
   furniture. Geofence in PX4 params at least 0.3 m inside the room.
6. Abort path: RC switch out of OFFBOARD during `capture_at` → `RigError(aborted)`, the session ends, the run dir is valid.
7. Battery: a session budget fits ≤ 60 % of the battery at the measured flight time per view.
