# Setup

## Host (workstation B, `ict-em018kc6`, verified 2026-10-08)

| Item | Version |
|---|---|
| OS | Ubuntu 24.04.3, no host ROS (by design) |
| GPU | 2× RTX 6000 Ada 48 GB, driver 580.105.08 (CUDA 13.0 capable) |
| Python | 3.12.3 (system), venv managed by `uv` |
| Docker | 29.1.3 + NVIDIA Container Toolkit; images `bisg/sim:6.0.0`, `bisg/ros:jazzy` present |

## Python environment

```bash
cd ~/kinetix
uv sync --extra ml                 # creates .venv from uv.lock
uv run python scripts/check_env.py # every line must say ok
```

Pinned by `uv.lock` (resolved 2026-10-08): torch 2.11.0+cu128, torchvision 0.26.0, pycolmap 4.2.1
(CPU build), open3d 0.20.0, trimesh 5.1.1, opencv-python-headless 5.0.0, transformers 5.19.0, numpy 2.5.3.
Change a version with `uv add`/`uv lock --upgrade-package` and note the reason in the commit message.

Extras planned per phase (installed when the phase starts, not before):

| Phase | Addition | Why |
|---|---|---|
| K2 | `gsplat` (CUDA build against torch cu128) | 3DGS densifier + NVS metrics |
| K2 | `colmap/colmap` Docker image (CUDA) | dense MVS (`colmap_mvs` densifier) |
| K2 | `lpips` | NVS metric |
| K4 | `docker/Dockerfile` → `kinetix:jazzy` image | `rclpy` for T1 serve / T2 / T3 |

## Machine settings (`.env`, git-ignored)

```bash
KINETIX_DATA=/home/akashshingha/kinetix-data   # scenes, pools, runs, bench (outside the repo)
BISG_DIR=/home/akashshingha/bisg_isaac
CUDA_VISIBLE_DEVICES=1                          # planner/densify GPU; Isaac uses GPU 0
ROS_DOMAIN_ID=0                                 # must match the bisg stack
```

## Before a sim run (T1/T2)

1. `nvidia-smi`: the user's vLLM service `lc-chat` takes ~46 GB per GPU. Stop it with `systemctl --user stop lc-chat`.
2. In `~/bisg_isaac`: `./bisg check`, then `./bisg smoke` once after any bisg pull.
3. Only for T2: `SIM_SCENARIO=<kinetix scenario> ./bisg all headless`.
