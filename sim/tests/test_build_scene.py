"""sim/build_scene.py on a synthetic glTF cube: units, Y-up -> Z-up, placement, map frame, USD text, observable GT."""
import importlib.util
import json
import sys
from pathlib import Path

import numpy as np
import trimesh
import yaml

from kinetix.config import load_scene

spec = importlib.util.spec_from_file_location("build_scene", Path(__file__).resolve().parents[1] / "build_scene.py")
build_scene = importlib.util.module_from_spec(spec)
spec.loader.exec_module(build_scene)


def write_cube_gltf(d: Path):
    """A 1 m cube in millimetres, outward winding, centred at the origin, as an uncompressed one-mesh glTF."""
    m = trimesh.creation.box(extents=(1000, 1000, 1000))
    P, F = m.vertices.astype(np.float32), m.faces.astype(np.uint32)
    N = (P / np.linalg.norm(P, axis=1, keepdims=True)).astype(np.float32)
    UV = np.zeros((len(P), 2), np.float32)
    blobs = [P.tobytes(), N.tobytes(), UV.tobytes(), F.tobytes()]
    offs = np.cumsum([0] + [len(b) for b in blobs])
    (d / "cube.bin").write_bytes(b"".join(blobs))
    for n in ("diffuse.jpg", "normal.jpg"):
        (d / n).write_bytes(b"\xff\xd8\xff\xd9")
    acc = lambda i, n, t: {"bufferView": i, "componentType": 5126 if i < 3 else 5125, "count": n, "type": t}  # noqa: E731
    (d / "cube.gltf").write_text(json.dumps({
        "asset": {"version": "2.0"}, "nodes": [{"mesh": 0}],
        "meshes": [{"primitives": [{"attributes": {"POSITION": 0, "NORMAL": 1, "TEXCOORD_0": 2}, "indices": 3, "material": 0}]}],
        "accessors": [acc(0, len(P), "VEC3"), acc(1, len(P), "VEC3"), acc(2, len(P), "VEC2"), acc(3, F.size, "SCALAR")],
        "bufferViews": [{"buffer": 0, "byteOffset": int(offs[i]), "byteLength": len(b)} for i, b in enumerate(blobs)],
        "buffers": [{"uri": "cube.bin", "byteLength": int(offs[-1])}],
        "images": [{"uri": "diffuse.jpg"}, {"uri": "normal.jpg"}], "textures": [{"source": 0}, {"source": 1}],
        "materials": [{"pbrMetallicRoughness": {"baseColorTexture": {"index": 0}}, "normalTexture": {"index": 1}}],
    }))


def test_build_cube(tmp_path, monkeypatch):
    data = tmp_path / "data"
    src = data / "sources" / "cube"
    src.mkdir(parents=True)
    write_cube_gltf(src)                       # the cache fetch_gltf would fill: no download
    recipe = {
        "name": "cube",
        "room": {"usd": "https://example.invalid/room.usd", "licence": "test", "floor_z": 0.0},
        "object": {"gltf_zip": "https://example.invalid/cube.zip", "title": "cube", "dataset": "none", "licence": "test",
                   "redistributable": True, "unit_m": 0.001, "position": [1.0, 2.0, 0.0], "yaw_deg": 90},
        "spawn": {"position": [-1.0, 2.0, 0.11], "yaw_deg": 0.0},
        "world_to_map": [0.5, 0.0, 0.1],
        "workspace_half_m": 2.0, "altitude_m": [0.4, 1.8], "standoff_m": [0.3, 1.5], "roi_margin_m": 0.05,
        "observable": {"spacing_m": 0.02, "grid_m": 0.25, "max_cameras": 300, "vfov_deg": 68.4, "mount_pitch_deg": 0.0,
                       "max_incidence_deg": 80.0},
        "lights": [{"position": [1.0, 2.0, 2.9], "size": [1.0, 1.0], "intensity": 1000}],
        "sim": {"light_scale": 1.0, "ambient_light": None},
    }
    rp = tmp_path / "cube.yaml"
    rp.write_text(yaml.safe_dump(recipe))
    monkeypatch.setenv("KINETIX_DATA", str(data))
    monkeypatch.setattr(sys, "argv", ["build_scene.py", str(rp)])
    build_scene.main()

    out = data / "scenes" / "cube"
    gt = trimesh.load(out / "gt_mesh.ply", process=False)
    np.testing.assert_allclose(gt.bounds, [[1.0, 1.5, 0.1], [2.0, 2.5, 1.1]], atol=1e-5)   # world box + world_to_map
    usd = (out / "object.usd").read_text()
    assert usd.count(", 3") >= 11 and "faceVertexCounts = [3, 3" in usd and "@textures/normal.jpg@" in usd
    scene_usd = (out / "scene.usd").read_text()
    assert "@https://example.invalid/room.usd@" in scene_usd and 'RectLight "Light0"' in scene_usd
    loc = yaml.safe_load((out / "bisg.yaml").read_text())
    assert loc["world"] == "scene.usd" and loc["spawn"]["position"] == [-1.0, 2.0, 0.11]

    s = load_scene(out)
    np.testing.assert_allclose(s.roi.size, [1.1, 1.1, 1.1], atol=1e-6)
    np.testing.assert_allclose(s.roi.center[:2], [1.5, 2.0], atol=1e-6)
    assert s.gt_mesh.exists()

    import open3d as o3d
    pc = o3d.io.read_point_cloud(str(out / "gt_observable.ply"))
    P, N = np.asarray(pc.points), np.asarray(pc.normals)
    assert len(P) > 0 and (N[:, 2] > -0.5).all()           # the bottom face sits on the floor: never observable
    assert (P[:, 2] > 1.09).any()                          # the top is (cameras up to 0.7 m above it)
