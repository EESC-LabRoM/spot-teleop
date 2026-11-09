# Video of the 36 planned sweep poses in the simulation: front camera | top camera (both at the recorded-frame calibration fits) | a third-person view.
# KINEMATIC playback: the joints are SET directly (linear interpolation between consecutive poses, a short hold at each pose). No controller, no
# torques, no tracking error: this shows WHERE the arm goes, not HOW the real impedance controller would get it there.
#   ./run_isaac.sh -s calib_sim_video.py calib_sim_video [--steps 10] [--hold 4]
import argparse, json, math, os
HERE = os.path.dirname(os.path.abspath(__file__))
ap = argparse.ArgumentParser()
ap.add_argument("--out", required=True)
ap.add_argument("--stage", default=os.path.join(HERE, "franka_workcell.usda"))
ap.add_argument("--inputs", default=os.path.join(HERE, "calib_sim", "inputs.json"))
ap.add_argument("--set", default="sweep36")
ap.add_argument("--steps", type=int, default=10, help="interpolation frames between consecutive poses")
ap.add_argument("--hold", type=int, default=4, help="frames held at each pose")
a = ap.parse_args(); a.out = os.path.abspath(a.out); os.makedirs(os.path.join(a.out, "frames"), exist_ok=True)

from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "renderer": "RayTracedLighting", "width": 960, "height": 540, "multi_gpu": False})
import numpy as np, omni.usd, omni.timeline, omni.kit.app, warp as wp
from pxr import Gf, Usd, UsdGeom, UsdPhysics
from PIL import Image, ImageDraw
from isaacsim.core.utils.stage import is_stage_loading
from isaacsim.core.simulation_manager import SimulationManager, SimulationEvent

DT, CTRL_EVERY = 1.0 / 200.0, 4
omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate("isaacsim.core.rendering_manager", True)
from isaacsim.core.rendering_manager import RenderingManager
RenderingManager.set_dt(CTRL_EVERY * DT)
import omni.replicator.core as rep

inp = json.load(open(a.inputs)); W, H = inp["res"]
ctx = omni.usd.get_context(); ctx.open_stage(a.stage); stage = ctx.get_stage()
for i in range(3000):
    app.update()
    if i > 60 and not is_stage_loading(): break
scene = json.load(open(os.path.join(HERE, "runs", "workcell", "scene.json")))
for n in ["zed", "zed_stand", "zed2", "zed2_stand"] + list(scene["objects"]):
    p = stage.GetPrimAtPath(f"/World/{n}")
    if p.IsValid(): p.SetActive(False)
fr = stage.GetPrimAtPath("/World/franka")
art_root = next((str(p.GetPath()) for p in Usd.PrimRange(fr) if p.HasAPI(UsdPhysics.ArticulationRootAPI)), None)
S = {"art": None}
def on_pre_step(dt, context):
    if S["art"] is not None: return
    view = SimulationManager.get_physics_simulation_view()
    if view is None: return
    art = view.create_articulation_view(art_root); names = list(art.shared_metatype.dof_names)
    dev = art.get_dof_positions().device; idx = wp.array([0], dtype=wp.uint32, device=dev)
    q = art.get_dof_positions().numpy()[0].astype(np.float32).copy()
    pre = next(n for n in names if n.endswith("_joint1"))[:-len("joint1")]
    S.update(art=art, dev=dev, idx=idx, q=q, a2d=np.array([names.index(f"{pre}joint{j + 1}") for j in range(7)]),
             fi=[i_ for i_, n_ in enumerate(names) if "finger" in n_], root=art.get_root_transforms().numpy()[0].copy())
SimulationManager.setup_simulation(dt=DT, device="cpu")
cb = SimulationManager.register_callback(on_pre_step, event=SimulationEvent.PHYSICS_PRE_STEP)
tl = omni.timeline.get_timeline_interface(); tl.play()
for _ in range(300):
    app.update()
    if S["art"] is not None: break
art, dev, idx, a2d, fi = S["art"], S["dev"], S["idx"], S["a2d"], S["fi"]
root = S["root"]


def quat_xyzw_to_R(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w); x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


T_wb = np.eye(4); T_wb[:3, :3] = quat_xyzw_to_R(*root[3:7]); T_wb[:3, 3] = root[:3]
CV2USD = np.diag([1.0, -1.0, -1.0, 1.0]); f_mm = 24.0
rps = {}


def add_cam(name, T_wc_usd, fx, fy):
    path = f"/World/vid_{name}"; cam = UsdGeom.Camera.Define(stage, path)
    cam.CreateFocalLengthAttr(f_mm); cam.CreateHorizontalApertureAttr(f_mm * W / fx); cam.CreateVerticalApertureAttr(f_mm * H / fy)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 40.0))
    xf = UsdGeom.Xformable(cam); xf.ClearXformOpOrder(); xf.AddTransformOp().Set(Gf.Matrix4d(*T_wc_usd.T.flatten().tolist()))
    rp = rep.create.render_product(path, (W, H)); an = rep.AnnotatorRegistry.get_annotator("rgb"); an.attach([rp]); rps[name] = (rp, an)


for name in ("front", "top"):
    c = inp["cameras"][name]; add_cam(name, T_wb @ np.array(c["T_base_cam"]) @ CV2USD, c["K"]["fx"], c["K"]["fy"])
base = T_wb[:3, 3]; ctr = np.array([base[0] + 0.35, base[1], base[2] + 0.1]); eye = ctr + np.array([1.9, -1.2, 0.9])
f = ctr - eye; f /= np.linalg.norm(f); r = np.cross(f, [0, 0, 1.0]); r /= np.linalg.norm(r); u = np.cross(r, f)
M = np.eye(4); M[:3, 0], M[:3, 1], M[:3, 2], M[:3, 3] = r, u, -f, eye
add_cam("overview", M, 520.0, 520.0)
for _ in range(10): app.update()


def grab(an):
    d = an.get_data(); d = d.get("data") if isinstance(d, dict) else d; return np.asarray(d)[..., :3]


def set_pose(q7, grip):
    q = S["q"].copy(); q[a2d] = q7
    for i_ in fi: q[i_] = float(np.clip(grip / 2.0, 0.0, 0.04))
    z = np.zeros_like(q)
    art.set_dof_positions(wp.array(q[None], dtype=wp.float32, device=dev), idx)
    art.set_dof_velocities(wp.array(z[None], dtype=wp.float32, device=dev), idx)
    art.set_dof_position_targets(wp.array(q[None], dtype=wp.float32, device=dev), idx)


poses = inp["sets"][a.set]; seq = []
for k, p in enumerate(poses):
    seq += [(np.array(p["q"], np.float32), p["grip"], f"pose {k + 1}/{len(poses)}", True)] * a.hold
    if k + 1 < len(poses):
        q0, q1 = np.array(p["q"], np.float32), np.array(poses[k + 1]["q"], np.float32)
        for s in range(1, a.steps + 1): seq.append(((1 - s / a.steps) * q0 + (s / a.steps) * q1, p["grip"], f"pose {k + 1}/{len(poses)} -> {k + 2}", False))
print(f"[video] {len(seq)} frames", flush=True)
labels = {"front": "front camera (zed1 fit)", "top": "top camera (zed2 fit)", "overview": "third-person view"}
for n, (q7, grip, txt, held) in enumerate(seq):
    set_pose(q7, grip)
    for _ in range(2): app.update()
    rep.orchestrator.step(rt_subframes=2, pause_timeline=False)
    tiles = []
    for name in ("front", "top", "overview"):
        im = Image.fromarray(grab(rps[name][1]).copy()); d = ImageDraw.Draw(im); d.rectangle([0, 0, W, 16], fill=(0, 0, 0)); d.text((4, 2), labels[name], fill=(255, 255, 255)); tiles.append(np.array(im))
    row = Image.fromarray(np.hstack(tiles)); d = ImageDraw.Draw(row)
    d.rectangle([0, H - 18, 3 * W, H], fill=(0, 0, 0)); d.text((4, H - 15), f"{txt}   |   KINEMATIC playback: joints set directly, no controller, no tracking error", fill=(255, 220, 0))
    row.save(os.path.join(a.out, "frames", f"{n:05d}.jpg"), quality=88)
    if n % 50 == 0: print(f"[video] frame {n}/{len(seq)}", flush=True)
os.system(f"chown -R 1000:1000 {a.out}")
tl.stop(); import sys; sys.stdout.flush(); os._exit(0)
