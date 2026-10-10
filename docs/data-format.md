# Data formats

Every artifact is plain files: YAML for configs, JSONL for per-item records, PNG/NPY for images and
depth, PLY for geometry, COLMAP binary for SfM. Poses follow [interfaces.md](interfaces.md) §1.

Data root: `$KINETIX_DATA` (set in `.env`, default `~/kinetix-data`), **outside the repo**. Only
small test fixtures live in the repo (`tests/fixtures/`, git-lfs).

```
$KINETIX_DATA/
  scenes/<scene>/          one object in one room          (§1)
  pools/<scene>/<pool>/    pre-rendered candidate views    (§2)
  runs/<run_id>/           one session                     (§3)
  bench/<matrix>/          one benchmark                   (§4)
```

## 1. Scene — `scenes/<scene>/`

```
scene.yaml
scene.usd          room + object (+ lights), loaded by bisg sim via world.usd_path (Q3)
object.usd         the object alone (provenance: source asset, licence)
gt_mesh.ply        object surface in map frame, metres (exported from object.usd at its placed pose)
gt_observable.ply  GT points visible from at least one feasible pose (raycast), used for completeness
```

```yaml
# scene.yaml
name: chair_01
source: {asset: "SimReady/.../chair.usd", dataset: simready | objaverse_pp | gso | omniobject3d, licence: "...", redistributable: true, scale: 1.0}
roi:       {center: [0.0, 0.0, 0.45], size: [0.6, 0.6, 0.9], yaw_deg: 0}
workspace: {center: [0.0, 0.0, 1.5],  size: [6.0, 6.0, 2.6], yaw_deg: 0}   # room minus margin
no_fly: []                     # extra boxes (furniture, lamps)
standoff_m: [0.8, 2.0]         # camera distance to the ROI surface
altitude_m: [0.4, 2.2]
gt_mesh: gt_mesh.ply
```

## 2. View pool — `pools/<scene>/<pool>/`

```
pool.yaml
views.jsonl        one view per line
images/<id>.png    left RGB, 8-bit
right/<id>.png     right RGB, 8-bit (stereo modes)
depth/<id>.npy     float32 metres, GT (eval / oracle / gt depth source only)
stereo/<id>.npy    float32 metres, learned-stereo depth cache (written on first use, keyed by model in pool.yaml)
zed/<id>.npy       float32 metres, real ZED SDK depth of the view (`--zed-depth`, ADR-K10); NaN = no depth
zed_conf/<id>.npy  float32 0–100 SDK confidence (higher = less confident; > depth_confidence already NaN in zed/)
```

```yaml
# pool.yaml
scene: chair_01
generator: {tool: render_server, git: <sha>, layout: "rings:4x60", seed: 0}   # or "tammes:128" (ObjView-Bench-compatible)
camera: {width: 1280, height: 720, fx: 529.8, fy: 529.8, cx: 640, cy: 360, mount_pitch_deg: 0, baseline_m: 0.063}
stereo_cache: {model: foundation_stereo, version: "<tag>"}   # absent until first computed
renderer: {isaac: 6.0.0, mode: RaytracedLighting, settle_frames: 3}
zed_depth: {sdk: 5.4.1, depth_mode: NEURAL_PLUS, settle_s: 0.6, bisg: <sha>}   # absent when rendered without --zed-depth
n_views: 240
splits: {candidate: 216, test: 24}
```

```json
{"id": "r2_037", "split": "candidate", "reachable": true, "pose": {"p": [1.2, 0.3, 0.9], "q": [..]}, "image": "images/r2_037.png", "right": "right/r2_037.png", "depth": "depth/r2_037.npy"}
```

`reachable: false` marks views a drone cannot take (below the floor, inside walls or no-fly boxes, outside
the 4-DoF mount constraint). They are kept so the pool stays comparable to sphere-based benchmarks, but they are never
selectable. `test` views are never selectable either. They are held out for novel-view metrics (PSNR/SSIM) on 3DGS outputs.

## 3. Run — `runs/<run_id>/`

`run_id = <YYYYmmdd-HHMMSS>_<scene>_<method>_<rig>_s<seed>`.

```
run.yaml           resolved config + provenance (written at start, summary appended at close)
frames.jsonl       one line per captured frame
steps.jsonl        one line per planning step
frames/<idx>.png   captured left images (T0: hard links into the pool)
frames/<idx>_R.png right images (stereo_rig* modes only)
depth/<idx>.npy    depth used for planning (+ <idx>_conf.npy), when plan_depth ≠ none
sfm/online/        final online model (COLMAP binary)
sfm/offline/       offline SfM over all frames
recon/             dense.ply | mesh.ply | gs/ (per densifier)
metrics.json       written by `kinetix eval`
log.txt            human log
```

```yaml
# run.yaml (excerpt)
kinetix: {git: <sha>, dirty: false, version: 0.1.0}
env: {host: ict-em018kc6, gpu: "RTX 6000 Ada", torch: 2.11.0, pycolmap: 4.2.1, isaac: 6.0.0}
scene: chair_01
rig: {name: pool, pool: rings_4x60}
method: kinetix_combo
sensing: {preset: mono, plan_depth: mono, recon_input: left, depth_model: da_v2_base}
uses_gt: false            # true if the scorer OR the depth source reads GT
seed: 0
config: { ... the full resolved config ... }
summary: {frames: 24, registered: 24, travel_m: 31.2, wall_s: 210.4, sim_s: null, stop_reason: gain_plateau}
```

```json
// frames.jsonl
{"idx": 5, "t": 102.4, "image": "frames/000005.png", "right_image": null, "depth": "depth/000005.npy", "depth_source": "mono",
 "K": {...}, "target": {...}, "pose": {...}, "gt_pose": {...}, "travel_m": 1.8, "pool_id": "r2_037"}
// steps.jsonl
{"step": 3, "n_cands": 216, "n_feasible": 180, "chosen": "r2_037", "gain": 0.41, "top5": [["r2_037",0.41],["r3_002",0.39]],
 "t_sample_s": 0.01, "t_score_s": 0.8, "t_recon_s": 3.2, "error": null}
```

```json
// metrics.json
{"n_images": 24, "registered_frac": 1.0, "travel_m": 31.2, "flight_s": null,
 "accuracy_mm": 4.1, "completeness_mm": 6.3, "chamfer_mm": 5.2,
 "fscore": {"tau_0.5pct": 0.71, "tau_1pct": 0.88}, "completeness_ratio": {"tau_1pct": 0.91},
 "ate_cm": 0.8, "nvs": {"psnr": 27.1, "ssim": 0.91},
 "eval": {"densify": "gsplat", "align": "sim3_gt_centres", "gt": "gt_observable.ply"}}
```

Readers must ignore unknown keys. Writers add keys and never repurpose them. A breaking change bumps
`format: N` in `run.yaml` and `pool.yaml`.

## 4. Benchmark — `bench/<matrix>/`

```
matrix.yaml        scenes × methods × budgets × seeds (copy of the input)
runs.txt           run ids belonging to this benchmark
results.csv        one row per run: run id, scene, method, sensing preset, budget, seed + every metrics.json scalar
report.md          tables + efficiency curves (figures/*.png)
```

## 5. Export

- COLMAP: `sfm/*` is already COLMAP binary, and `frames/` is the image folder.
- nerfstudio / gsplat: `kinetix export <run> --format nerfstudio` writes `transforms.json` (OpenGL
  camera convention: the conversion lives in `core.geometry`).
