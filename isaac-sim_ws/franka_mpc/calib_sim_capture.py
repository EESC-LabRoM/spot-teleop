# Camera-calibration test data in simulation: the Franka workcell (franka_workcell.usda) with two pinhole cameras placed EXACTLY at the
# recorded-frame EasyHeC fits of the real lab (front = zed1, top = zed2; T_base_cam in fr3_link0, calib_sim/inputs.json) and the arm driven
# through joint configurations (the 40 real recorded poses and the 36 planned sweep poses). Per pose and camera it saves the RGB image,
# the exact robot mask (instance segmentation, everything under /World/franka) and the joint positions READ BACK from PhysX, so a
# calibration can be scored against known ground truth. No physics task, no ROS, no ZED extension.
#   ./run_isaac.sh -s calib_sim_capture.py calib_sim1 [--sets recorded40,sweep36] [--max-poses N] [--keep-objects]
import argparse, json, math, os
HERE = os.path.dirname(os.path.abspath(__file__))
ap = argparse.ArgumentParser()
ap.add_argument("--out", required=True)
ap.add_argument("--stage", default=os.path.join(HERE, "franka_workcell.usda"))
ap.add_argument("--inputs", default=os.path.join(HERE, "calib_sim", "inputs.json"))
ap.add_argument("--sets", default="recorded40,sweep36")
ap.add_argument("--max-poses", type=int, default=0)
ap.add_argument("--keep-objects", action="store_true", help="leave the table objects (drill, mug, ...) in the scene; default removes them so a low pose cannot push one")
ap.add_argument("--settle", type=int, default=14, help="app updates after setting the joints (each = 4 physics steps)")
a = ap.parse_args(); a.out = os.path.abspath(a.out); os.makedirs(a.out, exist_ok=True)

from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "renderer": "RayTracedLighting", "width": 960, "height": 540, "multi_gpu": False})
import numpy as np, omni.usd, omni.timeline, omni.kit.app, warp as wp
from pxr import Gf, Usd, UsdGeom, UsdPhysics
from PIL import Image
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

# ---- scene: no ZED stands (our cameras are virtual), optionally no table objects --------------------------------------------
scene = json.load(open(os.path.join(HERE, "runs", "workcell", "scene.json")))
gone = []
for n in ["zed", "zed_stand", "zed2", "zed2_stand"] + ([] if a.keep_objects else list(scene["objects"])):
    p = stage.GetPrimAtPath(f"/World/{n}")
    if p.IsValid(): p.SetActive(False); gone.append(n)
print("[calib] deactivated", gone, flush=True)

fr = stage.GetPrimAtPath("/World/franka")
art_root = next((str(p.GetPath()) for p in Usd.PrimRange(fr) if p.HasAPI(UsdPhysics.ArticulationRootAPI)), None)
print("[calib] franka articulation", art_root, flush=True)
if art_root is None: os._exit(2)

S = {"art": None}
def on_pre_step(dt, context):
    if S["art"] is not None: return
    view = SimulationManager.get_physics_simulation_view()
    if view is None: return
    art = view.create_articulation_view(art_root); names = list(art.shared_metatype.dof_names)
    dev = art.get_dof_positions().device; idx = wp.array([0], dtype=wp.uint32, device=dev)
    q = art.get_dof_positions().numpy()[0].astype(np.float32).copy()
    pre = next(n for n in names if n.endswith("_joint1"))[:-len("joint1")]
    rt = art.get_root_transforms().numpy()[0]
    S.update(art=art, dev=dev, idx=idx, names=names, q=q, pre=pre, a2d=np.array([names.index(f"{pre}joint{j + 1}") for j in range(7)]),
             fi=[i_ for i_, n_ in enumerate(names) if "finger" in n_], root=rt.copy())
SimulationManager.setup_simulation(dt=DT, device="cpu")
cb = SimulationManager.register_callback(on_pre_step, event=SimulationEvent.PHYSICS_PRE_STEP)
tl = omni.timeline.get_timeline_interface(); tl.play()
for _ in range(300):
    app.update()
    if S["art"] is not None: break
if S["art"] is None: print("[calib] ERROR: articulation never initialised", flush=True); os._exit(3)
art, dev, idx, names, a2d, fi = S["art"], S["dev"], S["idx"], S["names"], S["a2d"], S["fi"]


def quat_xyzw_to_R(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w); x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


root = S["root"]; T_wb = np.eye(4); T_wb[:3, :3] = quat_xyzw_to_R(*root[3:7]); T_wb[:3, 3] = root[:3]
print("[calib] T_world_base (physics root):", T_wb.round(4).tolist(), flush=True)

# ---- the two calibrated cameras: pinhole, centred principal point (USD cameras), exact pose ---------------------------------
CV2USD = np.diag([1.0, -1.0, -1.0, 1.0])                 # OpenCV optical frame (x right, y down, z fwd) -> USD camera (x right, y up, -z fwd)
cams, meta_cams = {}, {}
for name, c in inp["cameras"].items():
    K = c["K"]; f_mm = 24.0
    path = f"/World/calib_{name}"; cam = UsdGeom.Camera.Define(stage, path)
    cam.CreateFocalLengthAttr(f_mm); cam.CreateHorizontalApertureAttr(f_mm * W / K["fx"]); cam.CreateVerticalApertureAttr(f_mm * H / K["fy"])
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 30.0))
    T_wc = T_wb @ np.array(c["T_base_cam"]) @ CV2USD
    xf = UsdGeom.Xformable(cam); xf.ClearXformOpOrder(); xf.AddTransformOp().Set(Gf.Matrix4d(*T_wc.T.flatten().tolist()))
    meta_cams[name] = dict(prim=path, T_base_cam=c["T_base_cam"], K_sim=dict(fx=K["fx"], fy=K["fy"], cx=W / 2, cy=H / 2), K_real_mapped=K,
                           T_world_camera_usd=T_wc.round(6).tolist(), source=c["source"])
    rp = rep.create.render_product(path, (W, H))
    an_rgb = rep.AnnotatorRegistry.get_annotator("rgb"); an_rgb.attach([rp])
    an_seg = rep.AnnotatorRegistry.get_annotator("instance_id_segmentation", init_params={"colorize": False}); an_seg.attach([rp])
    cams[name] = (rp, an_rgb, an_seg)
for _ in range(10): app.update()


def grab(an):
    d = an.get_data(); d = d.get("data") if isinstance(d, dict) else d; return np.asarray(d)


def set_pose(q7, grip):
    q = S["q"].copy(); q[a2d] = q7
    for i_ in fi: q[i_] = float(np.clip(grip / 2.0, 0.0, 0.04))
    z = np.zeros_like(q)
    art.set_dof_positions(wp.array(q[None], dtype=wp.float32, device=dev), idx)
    art.set_dof_velocities(wp.array(z[None], dtype=wp.float32, device=dev), idx)
    art.set_dof_position_targets(wp.array(q[None], dtype=wp.float32, device=dev), idx)


meta = dict(res=[W, H], T_world_base=T_wb.tolist(), cameras=meta_cams, deactivated=gone, joint_names=names, arm_dof_index=a2d.tolist(),
            note="mask = every pixel whose instance prim path is under /World/franka; q_read = PhysX DOF positions after settling")
json.dump(meta, open(os.path.join(a.out, "meta.json"), "w"), indent=1)

for sname in [s for s in a.sets.split(",") if s]:
    poses = inp["sets"][sname]; poses = poses[:a.max_poses] if a.max_poses else poses
    out = []
    for cname in cams: os.makedirs(os.path.join(a.out, sname, cname), exist_ok=True)
    for p in poses:
        set_pose(np.array(p["q"], np.float32), p["grip"])
        for _ in range(a.settle): app.update()
        qr = art.get_dof_positions().numpy()[0].astype(np.float64)
        for _ in range(2): rep.orchestrator.step(rt_subframes=3, pause_timeline=False)
        files = {}
        for cname, (rp, an_rgb, an_seg) in cams.items():
            rgb = grab(an_rgb)[..., :3]; seg = an_seg.get_data(); ids = np.asarray(seg["data"]); ids = ids[..., 0] if ids.ndim == 3 else ids
            lab = seg["info"]["idToLabels"]; mask = np.zeros(ids.shape, bool)
            for i_, path in lab.items():
                s_ = path if isinstance(path, str) else str(path)
                if s_.startswith("/World/franka"): mask |= (ids == int(i_))
            base = os.path.join(sname, cname, f"{p['i']:03d}")
            Image.fromarray(rgb).save(os.path.join(a.out, base + "_rgb.png")); Image.fromarray((mask * 255).astype(np.uint8)).save(os.path.join(a.out, base + "_mask.png"))
            files[cname] = dict(rgb=base + "_rgb.png", mask=base + "_mask.png", mask_px=int(mask.sum()))
        err = float(np.abs(qr[a2d] - np.array(p["q"])).max())
        out.append(dict(i=p["i"], q_cmd=p["q"], q_read=qr[a2d].tolist(), fingers_read=[float(qr[i_]) for i_ in fi], grip_cmd=p["grip"], max_joint_err_rad=err, files=files))
        print(f"[calib] {sname} {p['i']:02d}: joint err {err:.4f} rad, mask px " + ", ".join(f"{c} {files[c]['mask_px']}" for c in cams), flush=True)
    json.dump(out, open(os.path.join(a.out, sname, "poses.json"), "w"), indent=1)

os.system(f"chown -R 1000:1000 {a.out}")
tl.stop(); import sys; sys.stdout.flush(); os._exit(0)
