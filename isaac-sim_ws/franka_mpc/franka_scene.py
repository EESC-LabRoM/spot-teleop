# Franka workcell in the spot-sim warehouse (Isaac Sim 6.0): the Franka on the edge of the black work table, the table's
# manipulable objects (drill, mug, veja bottle, beaker, bin with two bottles) laid out within reach, the Spot-era boxes
# taken off the table, and an external ZED X on a stand across the table for nvblox mapping.
# Builds the stage, saves it as franka_workcell.usda (warehouse.usd is only sublayered, never edited), lets physics
# settle for 2 s, and writes previews: overview views, the ZED left RGB, its depth (.npy in m + 16-bit png in mm) and
# the intrinsics/pose a mapper needs (camera.json), plus scene.json (object poses relative to the Franka base).
#   ./run_isaac.sh -s franka_scene.py workcell
import argparse, json, math, os
HERE = os.path.dirname(os.path.abspath(__file__))
ap = argparse.ArgumentParser()
ap.add_argument("--out", required=True)
ap.add_argument("--warehouse", default="/workspace/spot-sim/assets/clutter/warehouse.usd")
ap.add_argument("--robot", default="fr3", choices=["fr3", "panda"], help="Isaac robot asset (fr3 = the lab arm)")
ap.add_argument("--zed-usd", default="/workspace/zed-isaac-sim/exts/sl.sensor.camera/data/usd/ZED_X.usdc")
ap.add_argument("--save", default=os.path.join(HERE, "franka_workcell.usda"))
a = ap.parse_args(); os.makedirs(a.out, exist_ok=True)

from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "renderer": "RayTracedLighting", "width": 1280, "height": 720, "multi_gpu": False})
import numpy as np, omni.usd, omni.timeline, warp as wp
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics
from isaacsim.core.utils.stage import is_stage_loading
from isaacsim.core.simulation_manager import SimulationManager, SimulationEvent
from isaacsim.storage.native import get_assets_root_path

TABLE = "/World/SM_TableWorkSecurity"                # black work table, top at z = 0.395 m (measured)
BASE = np.array([-1.15, -0.57])                      # Franka base on the table's -x long edge, facing +x
# object -> (forward, left) of the Franka base, m. Objects that ride along with another (bottles in the bin) follow it.
LAYOUT = {"drill": (0.45, -0.20), "SM_Mug_B1": (0.35, 0.22), "veja": (0.55, 0.02), "beaker_500ml": (0.30, -0.50),
          "small_KLT_visual_collision": (0.40, 0.48)}
RIDERS = {"small_KLT_visual_collision": ["SM_BottlePlasticC_01", "SM_BottlePlasticC_02"]}
ZED_EYE, ZED_AT = np.array([0.10, -0.30, 1.00]), np.array([-0.85, -0.57, 0.45])   # stand across the table, looking at the arm
RETRACT = [0.0, -1.3, 0.0, -2.5, 0.0, 1.0, 0.0]     # cuRobo franka.yml retract_config
W, H = 1280, 720

ctx = omni.usd.get_context(); ctx.new_stage(); stage = ctx.get_stage()
stage.GetRootLayer().subLayerPaths.append(a.warehouse)
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z); UsdGeom.SetStageMetersPerUnit(stage, 1.0)
for i in range(3000):
    app.update()
    if i > 60 and not is_stage_loading(): break

cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
def box(p):
    r = cache.ComputeWorldBound(p).ComputeAlignedRange(); return np.array(r.GetMin()), np.array(r.GetMax())
tmin, tmax = box(stage.GetPrimAtPath(TABLE)); TOP = float(tmax[2])
print("[scene] table", TABLE, "x", tmin[0].round(3), tmax[0].round(3), "y", tmin[1].round(3), tmax[1].round(3), "top", round(TOP, 3), flush=True)

# ---- clear the table except the objects we lay out ------------------------------------------------------------------
keep = set(LAYOUT) | {r for v in RIDERS.values() for r in v}
removed = []
for p in stage.GetPrimAtPath("/World").GetChildren():
    if p.GetPath().pathString == TABLE or not p.IsA(UsdGeom.Imageable): continue
    lo, hi = box(p)
    if not np.all(np.isfinite(lo)) or hi[2] - lo[2] > 5: continue
    c = (lo + hi) / 2
    on_top = tmin[0] <= c[0] <= tmax[0] and tmin[1] <= c[1] <= tmax[1] and lo[2] >= TOP - 0.03
    if on_top and p.GetName() not in keep:
        p.SetActive(False); removed.append(p.GetName())
print("[scene] taken off the table:", removed, flush=True)

def shift(p, d):                                     # world-space shift: /World children sit under an identity parent
    xf = UsdGeom.Xformable(p); ops = xf.GetOrderedXformOps()
    op = xf.AddTranslateOp(opSuffix="layout"); op.Set(Gf.Vec3d(*d.tolist())); xf.SetXformOpOrder([op] + ops)
for name, (fw, lf) in LAYOUT.items():
    p = stage.GetPrimAtPath(f"/World/{name}"); lo, hi = box(p); c = (lo + hi) / 2
    d = np.array([BASE[0] + fw - c[0], BASE[1] + lf - c[1], 0.0])
    for q in [name] + RIDERS.get(name, []): shift(stage.GetPrimAtPath(f"/World/{q}"), d)
cache.Clear()

# ---- Franka on the table edge ------------------------------------------------------------------------------------------
franka_usd = get_assets_root_path() + {"fr3": "/Isaac/Robots/FrankaRobotics/FrankaFR3/fr3.usd",   # the lab arm (FR3)
                                       "panda": "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"}[a.robot]
fr = stage.DefinePrim("/World/franka", "Xform"); fr.GetReferences().AddReference(franka_usd)
fx_ = UsdGeom.Xformable(fr); fx_.ClearXformOpOrder()            # franka.usd brings identity translate/orient/scale ops
fx_.AddTranslateOp(opSuffix="mount").Set(Gf.Vec3d(float(BASE[0]), float(BASE[1]), TOP))

# ---- ZED X on a stand --------------------------------------------------------------------------------------------------
def frame(fwd, eye):                                 # +X forward (the ZED looks along +X), +Z up
    f = fwd / np.linalg.norm(fwd); l = np.cross([0, 0, 1.0], f); l /= np.linalg.norm(l); u = np.cross(f, l)
    M = np.eye(4); M[:3, 0], M[:3, 1], M[:3, 2], M[:3, 3] = f, l, u, eye; return M
zed = stage.DefinePrim("/World/zed", "Xform"); zed.GetReferences().AddReference(a.zed_usd)
zx = UsdGeom.Xformable(zed); zx.ClearXformOpOrder()
zx.AddTransformOp().Set(Gf.Matrix4d(*frame(ZED_AT - ZED_EYE, ZED_EYE).T.flatten().tolist()))
for i in range(30): app.update()
zroot = stage.GetPrimAtPath("/World/zed")             # ZED_X.usdc's root is a free rigid body: pin it
if zroot.HasAPI(UsdPhysics.RigidBodyAPI): UsdPhysics.RigidBodyAPI(zroot).CreateKinematicEnabledAttr(True)
pole = UsdGeom.Cylinder.Define(stage, "/World/zed_stand/pole"); pole.CreateRadiusAttr(0.015); pole.CreateHeightAttr(float(ZED_EYE[2] - 0.03))
UsdGeom.Xformable(pole).AddTranslateOp().Set(Gf.Vec3d(float(ZED_EYE[0]), float(ZED_EYE[1]), float(ZED_EYE[2] - 0.03) / 2))
pole.CreateDisplayColorAttr([Gf.Vec3f(0.08, 0.08, 0.08)])
foot = UsdGeom.Cylinder.Define(stage, "/World/zed_stand/foot"); foot.CreateRadiusAttr(0.18); foot.CreateHeightAttr(0.02)
UsdGeom.Xformable(foot).AddTranslateOp().Set(Gf.Vec3d(float(ZED_EYE[0]), float(ZED_EYE[1]), 0.01))
foot.CreateDisplayColorAttr([Gf.Vec3f(0.08, 0.08, 0.08)])
left = "/World/zed/base_link/ZED_X/CameraLeft"
assert stage.GetPrimAtPath(left).IsValid(), "ZED left camera prim missing"

art_root = None
for i_ in range(1200):
    app.update()
    art_root = next((str(p.GetPath()) for p in Usd.PrimRange(fr) if p.HasAPI(UsdPhysics.ArticulationRootAPI)), None)
    if art_root and i_ >= 30: break
if art_root is None: print("[scene] ERROR: Franka did not load", flush=True); os._exit(2)

# ---- save the workcell (relative paths, so it opens from franka_mpc/ in the container or on the host) ---------------
stage.SetDefaultPrim(stage.GetPrimAtPath("/World")); stage.GetRootLayer().Export(a.save)
txt = open(a.save).read().replace("@/workspace/", "@../"); open(a.save, "w").write(txt)
print("[scene] saved", a.save, flush=True)

# ---- settle physics with the arm held at retract ---------------------------------------------------------------------
S = {"art": None}
def on_pre_step(dt, context):
    if S["art"] is not None: return
    view = SimulationManager.get_physics_simulation_view()
    if view is None: return
    art = view.create_articulation_view(art_root); names = list(art.shared_metatype.dof_names)
    dev = art.get_dof_positions().device; idx = wp.array([0], dtype=wp.uint32, device=dev)
    q = art.get_dof_positions().numpy()[0].astype(np.float32).copy()
    pre = next(n for n in names if n.endswith("_joint1"))[:-len("joint1")]   # "fr3_" or "panda_"
    for j, v in enumerate(RETRACT): q[names.index(f"{pre}joint{j + 1}")] = v
    for n in names:
        if "finger" in n: q[names.index(n)] = 0.04
    art.set_dof_positions(wp.array(q[None], dtype=wp.float32, device=dev), idx)
    art.set_dof_velocities(wp.array(np.zeros_like(q)[None], dtype=wp.float32, device=dev), idx)
    art.set_dof_position_targets(wp.array(q[None], dtype=wp.float32, device=dev), idx)
    S["art"] = art
SimulationManager.setup_simulation(dt=1.0 / 200.0, device="cpu")
cb = SimulationManager.register_callback(on_pre_step, event=SimulationEvent.PHYSICS_PRE_STEP)
tl = omni.timeline.get_timeline_interface(); tl.play()
def obj_pos():                                         # box centre and bottom of each kept object (physics writes USD)
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render]); out = {}
    for n in keep:
        r = bc.ComputeWorldBound(stage.GetPrimAtPath(f"/World/{n}")).ComputeAlignedRange()
        out[n] = (np.array(r.GetMin()) + np.array(r.GetMax())) / 2, float(r.GetMin()[2])
    return out
for _ in range(10): app.update()
p0 = obj_pos()
for _ in range(120): app.update()
p1 = obj_pos()

# ---- renders -----------------------------------------------------------------------------------------------------------
import omni.replicator.core as rep
from PIL import Image
ov = UsdGeom.Camera.Define(stage, "/World/overview_cam"); ov.CreateFocalLengthAttr(16.0); ov.CreateHorizontalApertureAttr(20.955)
ov.CreateVerticalApertureAttr(11.787); ov.CreateClippingRangeAttr(Gf.Vec2f(0.02, 200.0)); ov_op = ov.AddTransformOp()
rp_ov = rep.create.render_product("/World/overview_cam", (W, H)); a_ov = rep.AnnotatorRegistry.get_annotator("rgb"); a_ov.attach([rp_ov])
rp_z = rep.create.render_product(left, (W, H))
a_zrgb = rep.AnnotatorRegistry.get_annotator("rgb"); a_zrgb.attach([rp_z])
a_zd = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane"); a_zd.attach([rp_z])
def look(eye, at):                                    # USD camera: looks along -Z, +Y up
    eye, at = np.array(eye, float), np.array(at, float); f = at - eye; f /= np.linalg.norm(f)
    r = np.cross(f, [0, 0, 1.0]); r /= np.linalg.norm(r); u = np.cross(r, f)
    M = np.eye(4); M[:3, 0], M[:3, 1], M[:3, 2], M[:3, 3] = r, u, -f, eye; ov_op.Set(Gf.Matrix4d(*M.T.flatten().tolist()))
def grab(an):
    d = an.get_data(); d = d.get("data") if isinstance(d, dict) else d; return np.asarray(d)
ctr = np.array([BASE[0] + 0.35, BASE[1], TOP + 0.1])
for name, eye in {"overview_front": ctr + [1.9, -1.2, 0.9], "overview_back": ctr + [-1.6, 1.3, 1.1],
                  "overview_side": ctr + [0.2, -2.2, 0.7], "overview_wide": ctr + [3.2, -3.0, 2.2]}.items():
    look(eye, ctr)
    for _ in range(14): app.update()
    Image.fromarray(grab(a_ov)[..., :3]).save(os.path.join(a.out, f"{name}.jpg"), quality=90)
for _ in range(4): app.update()
Image.fromarray(grab(a_zrgb)[..., :3]).save(os.path.join(a.out, "zed_left_rgb.jpg"), quality=92)
depth = grab(a_zd).astype(np.float32); np.save(os.path.join(a.out, "zed_left_depth_m.npy"), depth)
Image.fromarray(np.clip(np.nan_to_num(depth, posinf=0) * 1000, 0, 65535).astype(np.uint16)).save(os.path.join(a.out, "zed_left_depth_mm.png"))

# ---- camera + scene facts ----------------------------------------------------------------------------------------------
cam = UsdGeom.Camera(stage.GetPrimAtPath(left)); f_mm = cam.GetFocalLengthAttr().Get()
hap, vap = cam.GetHorizontalApertureAttr().Get(), cam.GetVerticalApertureAttr().Get()
T = np.array(UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(left))).T   # column-vector convention
json.dump({"prim": left, "resolution": [W, H], "fx": f_mm / hap * W, "fy": f_mm / vap * H, "cx": W / 2, "cy": H / 2,
           "hfov_deg": math.degrees(2 * math.atan(hap / 2 / f_mm)), "vfov_deg": math.degrees(2 * math.atan(vap / 2 / f_mm)),
           "baseline_m": 0.12, "T_world_camera_usd": T.round(5).tolist(),
           "note": "USD camera axes (looks along -Z, +Y up); depth = distance to image plane, m"},
          open(os.path.join(a.out, "camera.json"), "w"), indent=1)
valid = np.isfinite(depth) & (depth > 0) & (depth < 20)
res = {"table": TABLE, "table_top_z": round(TOP, 3), "franka_base": [*BASE.round(3).tolist(), round(TOP, 3)],
       "taken_off_table": removed, "objects": {}, "zed_eye": ZED_EYE.tolist(), "zed_looks_at": ZED_AT.tolist(),
       "depth_valid_frac": round(float(valid.mean()), 3),
       "depth_m": {"min": round(float(depth[valid].min()), 3), "median": round(float(np.median(depth[valid])), 3)} if valid.any() else None}
for n in keep:
    c1, bottom = p1[n]; rel = c1[:2] - BASE
    res["objects"][n] = {"centre_world": c1.round(3).tolist(), "bottom_z": round(bottom, 3), "forward_left_of_base_m": rel.round(3).tolist(),
                         "dist_from_base_m": round(float(np.linalg.norm(rel)), 3),
                         "moved_while_settling_cm": round(float(np.linalg.norm(c1 - p0[n][0]) * 100), 2), "on_table": bool(bottom > TOP - 0.02)}
json.dump(res, open(os.path.join(a.out, "scene.json"), "w"), indent=1)
print("[scene] RESULT " + json.dumps(res), flush=True)
os.system(f"chown -R 1000:1000 {a.out} {a.save}")
tl.stop(); import sys; sys.stdout.flush(); os._exit(0)
