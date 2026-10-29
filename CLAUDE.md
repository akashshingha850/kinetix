# kinetix: instructions for Claude Code

Adaptive NBV photogrammetry for indoor objects and whole rooms (ADR-K11/K12) with a monocular drone. Read `docs/architecture.md` first,
then `docs/acquisition/todo.md` for the current phase. Update `docs/acquisition/todo.md` when work is finished.
Layout (no `src/`): `kinetix/` shared library, `acquisition/` (author A; todo `docs/acquisition/todo.md`, branch `acquisition`), `reconstruction/`
(author B; todo `docs/reconstruction/todo.md`, branch `reconstruction`), flights in `data/<run>/`, outputs in `results/<run>/<method>/`
(`docs/collaboration.md`). The areas meet only at the run directory (data-format.md §3) and `kinetix/`. Shared paths (`kinetix/`,
data-format/interfaces/publication-plan, deps) change by a separate PR. Tests live in each package's `tests/`.

- Build phase by phase (`docs/roadmap.md`, K0–K6). Do not start a later phase or widen scope unless asked.
- Paper claims are C1–C5 in `docs/paper/publication-plan.md` (ADR-K08). Never call the image saving a novelty, and every "first" must survive a novelty sweep.
- APIs, ROS topics and frames are defined in `docs/interfaces.md`. Change it there first, then the code.
- The sim/flight platform is `~/bisg_isaac` (ADR-K01): use only its interface contract, never copy its code.
  A change bisg needs is made in bisg under its own CLAUDE.md rules.
- Sensing modes (ADR-K07): `mono` is the method (left image + `camera_info` + vehicle pose). Stereo presets are
  comparisons through the same loop. Topics come from the per-mode whitelist (interfaces.md §4). Never ZED point cloud/mapping/odom.
- Dependency rule (architecture.md §4, `kinetix/tests/test_boundaries.py`): neither area imports the other, `kinetix/` imports neither
  (only `cli.py` dispatches), and `acquisition/planner/` + `mapping/` never import `rclpy` or rig adapters.
- Poses: `T_map_cam`, ENU map, OpenCV optical camera. Convert to COLMAP/OpenGL only in `kinetix.geometry`.
- Python env is `uv` (`uv run ...`). No host ROS. ROS code runs in the `kinetix` Docker image (K4).
- Large data lives in `$KINETIX_DATA` (outside the repo). Only `tests/fixtures/` is in git (git-lfs).
- Before a sim run on workstation B, check `nvidia-smi` and stop `lc-chat` (vLLM) if it holds VRAM.
- Tests: `uv run pytest` (unit + t0). `-m t1` / `-m t2` need the sim (`docs/testing.md`).
- Rooms (`kind: room` scenes): method `tour` + `kinetix sfm` (known-pose triangulation) is the capture path; each flight records an
  MCAP rosbag to `data/<run>/rosbag/` (git-ignored, GBs). Room numbers in the docs are from the bisg Warehouse at its default lighting.
