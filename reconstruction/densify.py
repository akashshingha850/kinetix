"""Dense points from an SfM model: COLMAP patch-match stereo + fusion in the CUDA COLMAP image (the pycolmap wheel has no CUDA;
architecture.md §11). `colmap_mvs(rec, image_root, work)` -> fused points (N, 3) in the model's frame; `work` keeps fused.ply only."""
from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

import numpy as np

IMAGE = "colmap/colmap:20260907.8077"


def colmap_mvs(rec, image_root: Path, work: Path, max_image_size: int = 1280, num_src: int = 20, geom: bool = True,
               num_iterations: int = 5, gpu: str = "0", timeout: int = 3600) -> np.ndarray:
    """COLMAP's defaults except `max_image_size`; bench presets trade them for speed (config/bench/*.yaml `densify`)."""
    import open3d as o3d
    image_root, work = Path(image_root).resolve(), Path(work).resolve()
    if work.exists():
        shutil.rmtree(work)
    (work / "sparse").mkdir(parents=True)
    rec.write(str(work / "sparse"))
    dense = work / "dense"
    steps = [
        ["image_undistorter", "--image_path", image_root, "--input_path", work / "sparse", "--output_path", dense,
         "--output_type", "COLMAP", "--max_image_size", max_image_size, "--num_patch_match_src_images", num_src],
        ["patch_match_stereo", "--workspace_path", dense, "--PatchMatchStereo.geom_consistency", int(geom),
         "--PatchMatchStereo.max_image_size", max_image_size, "--PatchMatchStereo.num_iterations", num_iterations],
        ["stereo_fusion", "--workspace_path", dense, "--output_path", work / "fused.ply",
         "--input_type", "geometric" if geom else "photometric"],
    ]
    mounts = sorted({str(image_root), str(work)})
    base = ["docker", "run", "--rm", "--gpus", f"device={gpu}", "--user", f"{os.getuid()}:{os.getgid()}",
            *[a for m in mounts for a in ("-v", f"{m}:{m}")], IMAGE, "colmap"]
    try:
        for s in steps:
            r = subprocess.run(base + [str(x) for x in s], capture_output=True, text=True, timeout=timeout)
            if r.returncode != 0:
                raise RuntimeError(f"colmap {s[0]} failed: {r.stderr[-2000:] or r.stdout[-2000:]}")
        pts = np.asarray(o3d.io.read_point_cloud(str(work / "fused.ply")).points)
    finally:
        shutil.rmtree(dense, ignore_errors=True)       # depth/normal maps: ~10 MB a view
        shutil.rmtree(work / "sparse", ignore_errors=True)
    return pts
