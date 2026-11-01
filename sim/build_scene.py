"""Build a scene folder (data-format.md §1) from a recipe in sim/scenes/: one scanned object (glTF) placed in a USD room.

    uv run python sim/build_scene.py sim/scenes/office_buddha.yaml [--out DIR]

Writes $KINETIX_DATA/scenes/<name>/ (default ~/kinetix-data/scenes/<name>/):
    object.usd + textures/   the object alone, metres, Z up, base centre at the origin, a static triangle-mesh collider
    scene.usd                the room (referenced by URL, not copied) + the object at its placed pose; bisg loads it as the world
    bisg.yaml                bisg locator: extends bisg's docker/sim/sim.yaml, sets this world, spawn and lighting
    gt_mesh.ply              the object surface in map (world + world_to_map), the same triangles the sim renders
    gt_observable.ply        GT points visible from at least one feasible camera position (raycast), for completeness
    gt_observable.npz        the same points + normals (P, N) for numpy-only readers (the planner's coverage_oracle)
    scene.yaml               what kinetix flies and scores against
USD is written as text, so no USD library is needed; the glTF is read directly (one mesh, one primitive, no Draco).
"""
from __future__ import annotations

import argparse
import io
import json
import os
import shutil
import urllib.request
import zipfile
from pathlib import Path

import numpy as np
import open3d as o3d
import trimesh
import yaml


def fetch_gltf(url: str, cache: Path) -> Path:
    """Download and unpack a zipped glTF once; returns the .gltf path."""
    cache.mkdir(parents=True, exist_ok=True)
    if not list(cache.glob("*.gltf")):
        with urllib.request.urlopen(url) as r:
            zipfile.ZipFile(io.BytesIO(r.read())).extractall(cache)
    (gltf,) = cache.glob("*.gltf")
    return gltf


def read_gltf(path: Path):
    """Positions, normals, UVs (glTF convention), triangles and the base-colour / normal texture files of a one-mesh glTF."""
    j = json.loads(path.read_text())
    (buffer,) = j["buffers"]
    buf = (path.parent / buffer["uri"]).read_bytes()
    (node,) = j["nodes"]
    if any(k in node for k in ("matrix", "translation", "rotation", "scale")) or len(j["meshes"]) != 1:
        raise SystemExit(f"{path}: only one untransformed mesh is supported")
    (prim,) = j["meshes"][0]["primitives"]
    if "extensions" in prim or prim.get("mode", 4) != 4:
        raise SystemExit(f"{path}: compressed or non-triangle primitive")

    def accessor(i):
        a = j["accessors"][i]
        v = j["bufferViews"][a["bufferView"]]
        width = {"SCALAR": 1, "VEC2": 2, "VEC3": 3}[a["type"]]
        dtype = {5126: np.float32, 5125: np.uint32, 5123: np.uint16}[a["componentType"]]
        if v.get("byteStride", width * np.dtype(dtype).itemsize) != width * np.dtype(dtype).itemsize:
            raise SystemExit(f"{path}: interleaved buffer views are not supported")
        off = v.get("byteOffset", 0) + a.get("byteOffset", 0)
        return np.frombuffer(buf, dtype, a["count"] * width, off).reshape(a["count"], width)

    att = prim["attributes"]
    mat = j["materials"][prim["material"]]
    tex = lambda t: path.parent / j["images"][j["textures"][t["index"]]["source"]]["uri"]  # noqa: E731
    pbr = mat.get("pbrMetallicRoughness", {})
    return dict(
        P=accessor(att["POSITION"]).astype(float), N=accessor(att["NORMAL"]).astype(float), UV=accessor(att["TEXCOORD_0"]).astype(float),
        F=accessor(prim["indices"]).reshape(-1, 3).astype(np.int64),
        diffuse=tex(pbr["baseColorTexture"]) if "baseColorTexture" in pbr else None, normal=tex(mat["normalTexture"]) if "normalTexture" in mat else None,
        roughness=pbr.get("roughnessFactor", 1.0), metallic=pbr.get("metallicFactor", 1.0),
    )


def rot_z(deg: float) -> np.ndarray:
    c, s = np.cos(np.radians(deg)), np.sin(np.radians(deg))
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def arr(a: np.ndarray, fmt: str = "{:.6g}") -> str:
    if a.ndim == 1:
        return "[" + ", ".join(str(int(x)) for x in a) + "]"
    return "[" + ", ".join("(" + ", ".join(fmt.format(x) for x in row) + ")" for row in a) + "]"


def object_usd(P, N, UV, F, rec, g) -> str:
    lo, hi = P.min(0), P.max(0)
    normal = "" if g["normal"] is None else """
                normal3f inputs:normal.connect = </Object/Looks/Surface/Normal.outputs:rgb>"""
    normal_shader = "" if g["normal"] is None else """
            def Shader "Normal"
            {
                uniform token info:id = "UsdUVTexture"
                asset inputs:file = @textures/normal.jpg@
                token inputs:sourceColorSpace = "raw"
                float4 inputs:scale = (2, 2, 2, 1)
                float4 inputs:bias = (-1, -1, -1, 0)
                float2 inputs:st.connect = </Object/Looks/Surface/St.outputs:result>
                float3 outputs:rgb
            }"""
    if g["diffuse"] is not None:
        diffuse = "color3f inputs:diffuseColor.connect = </Object/Looks/Surface/Diffuse.outputs:rgb>"
        diffuse_shader = """
            def Shader "Diffuse"
            {
                uniform token info:id = "UsdUVTexture"
                asset inputs:file = @textures/diffuse.jpg@
                token inputs:sourceColorSpace = "sRGB"
                float2 inputs:st.connect = </Object/Looks/Surface/St.outputs:result>
                float3 outputs:rgb
            }"""
    elif "diffuse_color" in rec:                             # a geometry-only scan (no colour texture): one flat colour
        diffuse, diffuse_shader = f"color3f inputs:diffuseColor = ({', '.join(f'{x:.3g}' for x in rec['diffuse_color'])})", ""
    else:
        raise SystemExit(f"{rec['gltf_zip']}: no base colour texture; set object.diffuse_color [r, g, b] in the recipe")
    return f"""#usda 1.0
(
    defaultPrim = "Object"
    metersPerUnit = 1
    upAxis = "Z"
    customLayerData = {{
        string source = "{rec['gltf_zip']}"
        string title = "{rec['title']}"
        string licence = "{rec['licence']}"
        string written_by = "kinetix sim/build_scene.py"
    }}
)

def Xform "Object" (
    kind = "component"
)
{{
    def Mesh "mesh" (
        prepend apiSchemas = ["MaterialBindingAPI", "PhysicsCollisionAPI", "PhysicsMeshCollisionAPI"]
    )
    {{
        float3[] extent = {arr(np.array([lo, hi]))}
        int[] faceVertexCounts = {arr(np.full(len(F), 3))}
        int[] faceVertexIndices = {arr(F.ravel())}
        rel material:binding = </Object/Looks/Surface>
        normal3f[] normals = {arr(N, "{:.4f}")} (
            interpolation = "vertex"
        )
        uniform token physics:approximation = "none"
        point3f[] points = {arr(P)}
        texCoord2f[] primvars:st = {arr(UV, "{:.6f}")} (
            interpolation = "vertex"
        )
        uniform token subdivisionScheme = "none"
    }}

    def Scope "Looks"
    {{
        def Material "Surface"
        {{
            token outputs:surface.connect = </Object/Looks/Surface/Shader.outputs:surface>

            def Shader "Shader"
            {{
                uniform token info:id = "UsdPreviewSurface"
                {diffuse}{normal}
                float inputs:metallic = {g['metallic']:.3g}
                float inputs:roughness = {g['roughness']:.3g}
                token outputs:surface
            }}

            def Shader "St"
            {{
                uniform token info:id = "UsdPrimvarReader_float2"
                string inputs:varname = "st"
                float2 outputs:result
            }}{diffuse_shader}{normal_shader}
        }}
    }}
}}
"""


def lights_usd(lights: list[dict]) -> str:
    """Ceiling rect lights (they shine down, along their -Z), world frame."""
    return "".join(f"""
    def RectLight "Light{i}"
    {{
        float inputs:height = {L['size'][1]}
        float inputs:intensity = {L['intensity']}
        float inputs:width = {L['size'][0]}
        double3 xformOp:translate = ({L['position'][0]}, {L['position'][1]}, {L['position'][2]})
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }}
""" for i, L in enumerate(lights))


def scene_usd(r: dict) -> str:
    o = r["object"]
    return f"""#usda 1.0
(
    defaultPrim = "Scene"
    metersPerUnit = 1
    upAxis = "Z"
    customLayerData = {{
        string kinetix_scene = "{r['name']}"
        string room_licence = "{r['room']['licence']}"
        string written_by = "kinetix sim/build_scene.py"
    }}
)

def Xform "Scene"
{{
    def Xform "Room" (
        prepend references = @{r['room']['usd']}@
    )
    {{
    }}

    def Xform "Object" (
        prepend references = @./object.usd@
    )
    {{
        double3 xformOp:translate = ({o['position'][0]}, {o['position'][1]}, {o['position'][2]})
        double xformOp:rotateZ = {o['yaw_deg']}
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:rotateZ"]
    }}
{plinth_usd(r)}{lights_usd(r.get('lights', []))}}}
"""


def plinth_usd(r: dict) -> str:
    """A plain grey box under the object (`plinth.size` [x, y, height], world): small objects sit at the camera rings' heights."""
    if "plinth" not in r:
        return ""
    (sx, sy, h), (x, y, _) = r["plinth"]["size"], r["object"]["position"]
    return f"""
    def Cube "Plinth" (
        prepend apiSchemas = ["PhysicsCollisionAPI"]
    )
    {{
        color3f[] primvars:displayColor = [(0.45, 0.45, 0.45)]
        double size = 1
        double3 xformOp:translate = ({x}, {y}, {r['room']['floor_z'] + h / 2})
        double3 xformOp:scale = ({sx}, {sy}, {h})
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
    }}
"""


def box(lo, hi) -> dict:
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return {"center": np.round((lo + hi) / 2, 4).tolist(), "size": np.round(hi - lo, 4).tolist(), "yaw_deg": 0}


def observable(mesh: trimesh.Trimesh, floor_z: float, ws_lo, ws_hi, standoff, ob: dict, seed: int = 0):
    """GT points with a clear, front-facing line of sight from some camera position in the workspace whose distance to the
    surface is within `standoff`, inside the vertical field of view of a camera at `mount_pitch_deg` (any yaw)."""
    rng = np.random.default_rng(seed)
    n = int(mesh.area / ob["spacing_m"] ** 2)
    pts, fid = trimesh.sample.sample_surface(mesh, n, seed=seed)
    nrm = mesh.face_normals[fid]
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.core.Tensor(mesh.vertices, o3d.core.float32), o3d.core.Tensor(mesh.faces, o3d.core.uint32))
    big = 50.0  # the floor occludes too
    fv = np.array([[-big, -big, floor_z], [big, -big, floor_z], [big, big, floor_z], [-big, big, floor_z]]) + [*mesh.centroid[:2], 0]
    scene.add_triangles(o3d.core.Tensor(fv, o3d.core.float32), o3d.core.Tensor([[0, 1, 2], [0, 2, 3]], o3d.core.uint32))
    axes = [np.arange(lo, hi + 1e-9, ob["grid_m"]) for lo, hi in zip(ws_lo, ws_hi)]
    cams = np.stack(np.meshgrid(*axes, indexing="ij"), -1).reshape(-1, 3)
    d = scene.compute_distance(o3d.core.Tensor(cams, o3d.core.float32)).numpy()
    cams = cams[(d >= standoff[0]) & (d <= standoff[1])]
    if len(cams) > ob["max_cameras"]:
        cams = cams[rng.choice(len(cams), ob["max_cameras"], replace=False)]
    seen = np.zeros(len(pts), bool)
    half, pitch = np.radians(ob["vfov_deg"]) / 2, np.radians(ob["mount_pitch_deg"])
    cos_inc = np.cos(np.radians(ob["max_incidence_deg"]))
    for c in cams:
        v = pts - c
        dist = np.linalg.norm(v, axis=1)
        u = v / dist[:, None]
        elev = np.arcsin(np.clip(u[:, 2], -1, 1))
        cand = ~seen & (np.einsum("ij,ij->i", -u, nrm) > cos_inc) & (np.abs(elev - pitch) <= half)
        if not cand.any():
            continue
        rays = np.hstack([np.repeat(c[None], cand.sum(), 0), u[cand]]).astype(np.float32)
        t = scene.cast_rays(o3d.core.Tensor(rays))["t_hit"].numpy()
        seen[np.flatnonzero(cand)[t >= dist[cand] - 0.003]] = True
    return pts[seen], nrm[seen], len(cams), len(pts)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("recipe", type=Path)
    ap.add_argument("--out", type=Path, help="scene folder (default $KINETIX_DATA/scenes/<name>)")
    a = ap.parse_args()
    r = yaml.safe_load(a.recipe.read_text())
    data = Path(os.environ.get("KINETIX_DATA", "~/kinetix-data")).expanduser()
    out = a.out or data / "scenes" / r["name"]
    (out / "textures").mkdir(parents=True, exist_ok=True)
    o = r["object"]

    g = read_gltf(fetch_gltf(o["gltf_zip"], data / "sources" / r["name"]))
    # object frame: metres, Z up (glTF Y up: x, y, z -> x, -z, y), base centre at the origin
    R_up = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], float)
    P, N = g["P"] * o["unit_m"] @ R_up.T, g["N"] @ R_up.T
    lo, hi = P.min(0), P.max(0)
    P -= [(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, lo[2]]
    UV = np.c_[g["UV"][:, 0], 1.0 - g["UV"][:, 1]]          # glTF v runs down, USD st up
    if g["diffuse"] is not None:
        shutil.copy(g["diffuse"], out / "textures" / "diffuse.jpg")
    if g["normal"] is not None:
        shutil.copy(g["normal"], out / "textures" / "normal.jpg")
    (out / "object.usd").write_text(object_usd(P, N, UV, g["F"], o, g))
    (out / "scene.usd").write_text(scene_usd(r))

    # the placed object in map: world = R(yaw) p + position, map = world + world_to_map
    w2m = np.asarray(r["world_to_map"], float)
    Vm = P @ rot_z(o["yaw_deg"]).T + o["position"] + w2m
    mesh = trimesh.Trimesh(Vm, g["F"], process=False)
    mesh.export(out / "gt_mesh.ply")
    m_lo, m_hi = Vm.min(0) - r["roi_margin_m"], Vm.max(0) + r["roi_margin_m"]
    c = (Vm.min(0) + Vm.max(0)) / 2
    alt = r["altitude_m"]
    ws_lo, ws_hi = [c[0] - r["workspace_half_m"], c[1] - r["workspace_half_m"], alt[0]], [c[0] + r["workspace_half_m"], c[1] + r["workspace_half_m"], alt[1]]
    floor_map = r["room"]["floor_z"] + w2m[2]
    no_fly = []
    if "plinth" in r:                                   # its top is the object's floor (it occludes like one), and it is an obstacle
        (sx, sy, h), (x, y, _) = r["plinth"]["size"], o["position"]
        if abs(o["position"][2] - r["room"]["floor_z"] - h) > 1e-6:
            raise SystemExit("object.position z must be the plinth top (floor_z + plinth height)")
        no_fly.append(box(np.array([x - sx / 2, y - sy / 2, r["room"]["floor_z"]]) + w2m, np.array([x + sx / 2, y + sy / 2, o["position"][2]]) + w2m))
    pts, nrm, n_cam, n_pts = observable(mesh, o["position"][2] + w2m[2], ws_lo, ws_hi, r["standoff_m"], r["observable"])
    pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts))
    pc.normals = o3d.utility.Vector3dVector(nrm)
    o3d.io.write_point_cloud(str(out / "gt_observable.ply"), pc)
    np.savez_compressed(out / "gt_observable.npz", P=pts.astype(np.float32), N=nrm.astype(np.float32))   # the same, for numpy-only readers

    sp = r["spawn"]
    sim = r.get("sim", {})
    (out / "bisg.yaml").write_text(
        f"# bisg locator for kinetix scene {r['name']} (bisg docs/configuration.md); written by kinetix sim/build_scene.py from\n"
        f"# {a.recipe}. Run: ./bisg all headless -c {out / 'bisg.yaml'}\n"
        + yaml.safe_dump({
            "scenario": "/workspace/docker/sim/sim.yaml", "world": "scene.usd",
            "light_scale": sim.get("light_scale", 1.0), "ambient_light": sim.get("ambient_light"), "keep_only": None, "drop": None,
            "spawn": {"position": sp["position"], "yaw_deg": sp["yaw_deg"]},
        }, sort_keys=False, default_flow_style=None))
    scene = {
        "name": r["name"],
        "source": {"asset": o["gltf_zip"], "title": o["title"], "dataset": o["dataset"], "licence": o["licence"],
                   "redistributable": o["redistributable"], "scale": 1.0,
                   "room": {"usd": r["room"]["usd"], "licence": r["room"]["licence"]}},
        "roi": box(m_lo, m_hi),
        "workspace": box(ws_lo, ws_hi),
        "no_fly": no_fly,
        "standoff_m": r["standoff_m"],
        "altitude_m": alt,
        "standby": [0.0, 0.0, 1.2],
        "gt_mesh": "gt_mesh.ply",
        "gt_observable": "gt_observable.ply",
        "sim": {"world_to_map": [float(x) for x in w2m], "bisg": "bisg.yaml"},
    }
    (out / "scene.yaml").write_text(
        f"# Scene (data-format.md §1), in map (ENU, EKF2 origin = the drone's spawn). Written by kinetix sim/build_scene.py from\n"
        f"# {a.recipe}; edit the recipe, not this file. Floor = map z {floor_map:.3f}.\n" + yaml.safe_dump(scene, sort_keys=False, default_flow_style=None))
    print(f"{out}: {len(g['F'])} triangles, object {np.round(Vm.max(0) - Vm.min(0), 3).tolist()} m, "
          f"gt_observable {len(pts)}/{n_pts} points from {n_cam} camera positions")


if __name__ == "__main__":
    main()
