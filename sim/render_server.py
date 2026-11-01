"""T1 render server (interfaces.md §5): a free ZED-Mini-like camera in a Kinetix scene, rendered by Isaac at exact poses.
No drone, no PX4. Runs in the bisg/sim image (Isaac's Python; scripts/render_server.sh starts it).

    batch:  render_server.py --scene /scenes/<name> --views views.jsonl --out <pool_dir>
            views.jsonl: {"id": "...", "pose": {"p": [x, y, z], "q": [x, y, z, w]}} per line (T_map_cam, OpenCV optical, map)
            writes images/<id>.png, right/<id>.png, depth/<id>.npy (float32 m, optical z, NaN = no return), views.jsonl, pool.yaml
    serve:  render_server.py --scene /scenes/<name> --serve
            /kinetix/render/goal (geometry_msgs/PoseStamped, frame map, header.stamp = request id) ->
            /kinetix/render/image (rgb8), /kinetix/render/depth (32FC1), /kinetix/render/camera_info, stamped with the request id

The scene's map frame is world + scene.yaml sim.world_to_map (exact here: there is no EKF2 to re-measure it), and its lighting is
the scene's bisg locator (bisg.yaml light_scale / ambient_light), so a render matches what the flight sim shows.
"""
import argparse
import json
import os
import shutil
import time

import numpy as np
import yaml

ap = argparse.ArgumentParser()
ap.add_argument("--scene", required=True, help="scene folder in the container (scene.yaml, scene.usd, bisg.yaml)")
ap.add_argument("--views", help="batch: views.jsonl")
ap.add_argument("--out", help="batch: pool folder")
ap.add_argument("--serve", action="store_true", help="answer /kinetix/render/goal requests until stopped")
ap.add_argument("--width", type=int, default=1280)
ap.add_argument("--height", type=int, default=720)
ap.add_argument("--fx", type=float, default=529.8)       # ZED Mini HD720, = acquisition/config/common.yaml camera
ap.add_argument("--baseline", type=float, default=0.063)
ap.add_argument("--settle", type=int, default=10, help="rendered frames after a move before reading (ground truth lags 2; the denoiser"
                                                     " needs ~10 for a clean frame)")
a = ap.parse_args()
if not a.serve and not (a.views and a.out):
    ap.error("--serve, or --views and --out")

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True, "renderer": "RaytracedLighting", "width": a.width, "height": a.height})

import carb  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, UsdGeom, UsdLux  # noqa: E402

scene = yaml.safe_load(open(os.path.join(a.scene, "scene.yaml")))
loc = yaml.safe_load(open(os.path.join(a.scene, scene["sim"].get("bisg", "bisg.yaml"))))
w2m = np.asarray(scene["sim"]["world_to_map"], float)

ctx = omni.usd.get_context()
ctx.open_stage(os.path.join(a.scene, scene.get("usd", "scene.usd")))
t0 = time.time()
while True:                                               # the room is referenced by URL: wait until every layer is in
    app.update()
    _, loaded, total = ctx.get_stage_loading_status()
    if total == 0 and time.time() - t0 > 5:
        break
stage = ctx.get_stage()
scale = float(loc.get("light_scale") or 1.0)
for prim in stage.Traverse():
    if scale != 1.0 and prim.HasAPI(UsdLux.LightAPI):
        attr = UsdLux.LightAPI(prim).GetIntensityAttr()
        attr.Set(attr.Get() * scale)
carb.settings.get_settings().set_float("/rtx/sceneDb/ambientLightIntensity", float(loc.get("ambient_light") or 0.0))

FOCAL = 10.0                                              # any focal works: fx = focal / aperture * width


def camera(path):
    cam = UsdGeom.Camera.Define(stage, path)
    cam.GetFocalLengthAttr().Set(FOCAL)
    cam.GetHorizontalApertureAttr().Set(FOCAL * a.width / a.fx)
    cam.GetVerticalApertureAttr().Set(FOCAL * a.height / a.fx)
    cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.05, 100.0))
    op = UsdGeom.Xformable(cam).AddTransformOp()
    rp = rep.create.render_product(path, (a.width, a.height))
    ann = {}
    for name in ("rgb", "distance_to_image_plane"):
        ann[name] = rep.AnnotatorRegistry.get_annotator(name)
        ann[name].attach([rp])
    return op, ann


left_op, left = camera("/KinetixCamera/left")
right_op, right = camera("/KinetixCamera/right") if not a.serve else (None, None)   # mono serve: left only


def quat_to_R(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def place(op, p_map, R_cv):
    """OpenCV optical camera (x right, y down, z forward) in map -> USD camera (x right, y up, looks along -z) in world."""
    R = R_cv @ np.diag([1.0, -1.0, -1.0])
    M = np.eye(4)
    M[:3, :3] = R.T                                       # USD matrices are row-vector: rows = the camera axes
    M[3, :3] = np.asarray(p_map, float) - w2m
    op.Set(Gf.Matrix4d(M.tolist()))


FIRST = [True]


def render(pose, with_right):
    """Left RGB (HxWx3 uint8), depth (HxW float32 m, NaN = no return) and right RGB (or None) at camera pose T_map_cam."""
    p, R = np.asarray(pose["p"], float), quat_to_R(pose["q"])
    place(left_op, p, R)
    if with_right:
        place(right_op, p + R[:, 0] * a.baseline, R)     # the right camera: +baseline along the optical x
    for _ in range(a.settle * (6 if FIRST[0] else 1)):   # the first view also streams the textures it sees: grainy and dark at 4
        app.update()
    FIRST[0] = False
    for _ in range(100):                                  # annotators return an empty array until their first frame lands
        rgb, depth = np.array(left["rgb"].get_data()), np.array(left["distance_to_image_plane"].get_data(), dtype=np.float32)
        if rgb.ndim == 3 and depth.ndim == 2 and (not with_right or np.array(right["rgb"].get_data()).ndim == 3):
            break
        app.update()
    else:
        raise RuntimeError("the annotators produced no image")
    rgb = rgb[..., :3].astype(np.uint8)
    depth[~np.isfinite(depth) | (depth <= 0)] = np.nan
    rgb_r = np.array(right["rgb"].get_data())[..., :3].astype(np.uint8) if with_right else None
    return rgb, depth, rgb_r


omni.timeline.get_timeline_interface().play()             # annotators only produce data while the timeline plays
for _ in range(30):                                       # first frames compile shaders and stream textures
    app.update()

if not a.serve:
    from PIL import Image
    os.umask(0)                                           # the pool is the host user's: writable, so the pool rig can hard-link it
    views = [json.loads(line) for line in open(a.views) if line.strip()]
    for sub in ("images", "right", "depth"):
        os.makedirs(os.path.join(a.out, sub), exist_ok=True)
    t1 = time.time()
    for i, v in enumerate(views):
        rgb, depth, rgb_r = render(v["pose"], True)
        Image.fromarray(rgb).save(os.path.join(a.out, "images", f"{v['id']}.png"))
        Image.fromarray(rgb_r).save(os.path.join(a.out, "right", f"{v['id']}.png"))
        np.save(os.path.join(a.out, "depth", f"{v['id']}.npy"), depth)
        if i % 20 == 0:
            print(f"[render_server] {i + 1}/{len(views)} views", flush=True)
    shutil.copy(a.views, os.path.join(a.out, "views.jsonl"))
    yaml.safe_dump({"scene": scene["name"], "generator": {"tool": "render_server", "views": os.path.basename(a.views)},
                    "camera": {"width": a.width, "height": a.height, "fx": a.fx, "fy": a.fx, "cx": a.width / 2, "cy": a.height / 2,
                               "mount_pitch_deg": 0, "baseline_m": a.baseline},
                    "renderer": {"isaac": "6.0", "mode": "RaytracedLighting", "settle_frames": a.settle},
                    "n_views": len(views)}, open(os.path.join(a.out, "pool.yaml"), "w"), sort_keys=False)
    print(f"[render_server] {len(views)} views in {time.time() - t1:.0f} s -> {a.out}", flush=True)
else:
    from isaacsim.core.experimental.utils.app import enable_extension
    enable_extension("isaacsim.ros2.bridge")              # Isaac's bundled Jazzy rclpy
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CameraInfo, Image as ImageMsg
    try:
        rclpy.init()
    except RuntimeError:
        pass
    node = rclpy.create_node("kinetix_render_server")
    qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
    pub_img = node.create_publisher(ImageMsg, "/kinetix/render/image", qos)
    pub_dep = node.create_publisher(ImageMsg, "/kinetix/render/depth", qos)
    pub_info = node.create_publisher(CameraInfo, "/kinetix/render/camera_info", qos)
    goals = []
    node.create_subscription(PoseStamped, "/kinetix/render/goal", goals.append, qos)

    def image(header, arr, enc, step):
        m = ImageMsg(header=header, height=arr.shape[0], width=arr.shape[1], encoding=enc, is_bigendian=0, step=step)
        m.data = arr.tobytes()
        return m

    print(f"[render_server] serving {scene['name']} on /kinetix/render/goal", flush=True)
    try:
        while app.is_running():
            rclpy.spin_once(node, timeout_sec=0.01)
            if not goals:
                app.update()
                continue
            g = goals.pop(0)
            q = g.pose.orientation
            rgb, depth, _ = render({"p": [g.pose.position.x, g.pose.position.y, g.pose.position.z], "q": [q.x, q.y, q.z, q.w]}, False)
            h = g.header
            h.frame_id = "kinetix_camera_optical"
            info = CameraInfo(header=h, height=a.height, width=a.width, distortion_model="plumb_bob")
            info.k = [a.fx, 0.0, a.width / 2, 0.0, a.fx, a.height / 2, 0.0, 0.0, 1.0]
            pub_info.publish(info)
            pub_dep.publish(image(h, depth, "32FC1", 4 * a.width))
            pub_img.publish(image(h, rgb, "rgb8", 3 * a.width))
    except KeyboardInterrupt:
        pass
app.close()
