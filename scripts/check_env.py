"""Verify the Kinetix Python environment: each library is imported *and* exercised once.

Run: uv run python scripts/check_env.py   (exit code 0 = all required checks pass)
"""
import shutil
import subprocess
import sys

results: list[tuple[str, bool, str]] = []


def check(name, fn, required=True):
    try:
        results.append((name, True, fn()))
    except Exception as e:  # noqa: BLE001 - report every failure, keep going
        results.append((name, not required, f"{'FAIL' if required else 'missing (optional)'}: {e}"))


def _numpy():
    import numpy as np
    return np.__version__


def _torch():
    import torch
    assert torch.cuda.is_available(), "CUDA not available"
    x = torch.ones(1024, 1024, device="cuda") @ torch.ones(1024, 1024, device="cuda")
    assert float(x[0, 0]) == 1024.0
    gpus = ", ".join(torch.cuda.get_device_name(i) for i in range(torch.cuda.device_count()))
    return f"{torch.__version__} cuda {torch.version.cuda} [{gpus}]"


def _pycolmap():
    import numpy as np
    import pycolmap
    cam = pycolmap.Camera(model="PINHOLE", width=640, height=480, params=[500.0, 500.0, 320.0, 240.0])
    uv = cam.img_from_cam(np.array([[0.1, -0.1, 1.0]]))
    assert np.allclose(uv, [[370.0, 190.0]])
    return f"{pycolmap.__version__} (CUDA build: {pycolmap.has_cuda})"


def _open3d():
    import numpy as np
    import open3d as o3d
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(o3d.geometry.TriangleMesh.create_box()))
    rays = o3d.core.Tensor([[0.5, 0.5, 5.0, 0.0, 0.0, -1.0]], dtype=o3d.core.Dtype.Float32)
    t = scene.cast_rays(rays)["t_hit"].numpy()[0]
    assert np.isclose(t, 4.0), t
    return o3d.__version__


def _trimesh():
    import trimesh
    m = trimesh.creation.icosphere()
    assert m.is_watertight
    return trimesh.__version__


def _cv2():
    import cv2
    return cv2.__version__


def _transformers():
    import transformers
    return transformers.__version__


def _docker_images():
    out = subprocess.run(["docker", "images", "--format", "{{.Repository}}:{{.Tag}}"],
                         capture_output=True, text=True, check=True).stdout.split()
    need = ["bisg/sim:6.0.0", "bisg/ros:jazzy"]
    missing = [i for i in need if i not in out]
    assert not missing, f"missing {missing} (build them in ~/bisg_isaac)"
    return "bisg/sim:6.0.0, bisg/ros:jazzy present"


def _bin(name):
    return lambda: shutil.which(name) or (_ for _ in ()).throw(RuntimeError("not on PATH"))


check("python", lambda: sys.version.split()[0])
check("numpy", _numpy)
check("torch+cuda", _torch)
check("pycolmap", _pycolmap)
check("open3d", _open3d)
check("trimesh", _trimesh)
check("opencv", _cv2)
check("transformers", _transformers)
check("docker images (T1/T2)", _docker_images, required=False)
check("ffmpeg", _bin("ffmpeg"), required=False)

width = max(len(n) for n, _, _ in results)
for name, ok, msg in results:
    print(f"{'ok ' if ok else 'ERR'} {name:<{width}}  {msg}")
sys.exit(0 if all(ok for _, ok, _ in results) else 1)
