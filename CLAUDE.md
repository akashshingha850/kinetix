# kinetix: instructions for Claude Code

Adaptive NBV photogrammetry for indoor objects with a monocular drone. Read `docs/architecture.md` first,
then `docs/todo.md` for the current phase. Update `docs/todo.md` when work is finished.

- Build phase by phase (`docs/roadmap.md`, K0–K6). Do not start a later phase or widen scope unless asked.
- APIs, ROS topics and frames are defined in `docs/interfaces.md`. Change it there first, then the code.
- The sim/flight platform is `~/bisg_isaac` (ADR-K01): use only its interface contract, never copy its code.
  A change bisg needs is made in bisg under its own CLAUDE.md rules.
- Monocular rule (ADR-K03): only the left ZED image, `camera_info` and the vehicle pose.
- Dependency rule (architecture.md §4): `planner/` and `core/` never import `rclpy`, rig adapters or `offline/`.
- Poses: `T_map_cam`, ENU map, OpenCV optical camera. Convert to COLMAP/OpenGL only in `core.geometry`.
- Python env is `uv` (`uv run ...`). No host ROS. ROS code runs in the `kinetix` Docker image (K4).
- Large data lives in `$KINETIX_DATA` (outside the repo). Only `tests/fixtures/` is in git (git-lfs).
- Before a sim run on workstation B, check `nvidia-smi` and stop `lc-chat` (vLLM) if it holds VRAM.
- Tests: `uv run pytest` (unit + t0). `-m t1` / `-m t2` need the sim (`docs/testing.md`).
