# Adds the HiveBoard (honeycomb panel + ball-valve attachment, MIT, EESC/USP) to the Franka workcell: franka_workcell.usda -> franka_workcell_hive.usda.
# Placement = the isaaclab-hiveboard FR3 ball-valve task: valve root 0.70 m ahead, 0.06 m left, 0.554 m above the Franka base, yawed 180 deg
# (valve +x faces the robot), honeycomb at (-0.04,0,0) in the valve frame, scaled 0.001 (its USD is in mm). The valve's FixedJoint pins it to the
# WORLD (body0 empty), so its localPos0/localRot0 are set to the placed pose. Plastic board has no physics material in the HiveBoard USDs.
# The table is 0.8 m long (x -1.27..-0.47, base at -1.15), so the board sits at 0.60 m ahead (the task's 0.70 m would hang past the edge) and the veja bottle is taken off (--drop).
# Also: 1.5 s of physics (arm at its default pose) to check that the valve stays put, then overview + close-up renders.
#   ./run_isaac.sh -s hiveboard_scene.py hive_scene1
import argparse, json, math, os
HERE = os.path.dirname(os.path.abspath(__file__))
ap = argparse.ArgumentParser(); ap.add_argument("--out", required=True)
ap.add_argument("--src", default=os.path.join(HERE, "franka_workcell.usda")); ap.add_argument("--save", default=os.path.join(HERE, "franka_workcell_hive.usda"))
ap.add_argument("--fwd", type=float, default=0.60); ap.add_argument("--left", type=float, default=0.06); ap.add_argument("--up", type=float, default=0.554)
ap.add_argument("--drop", default="veja", help="comma list of /World prims taken off the table (they sit where the board goes)")
a = ap.parse_args(); os.makedirs(a.out, exist_ok=True)
from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "renderer": "RayTracedLighting", "width": 1280, "height": 720, "multi_gpu": False})
import numpy as np, omni.usd, omni.timeline
from pxr import Gf, Usd, UsdGeom, UsdPhysics
from isaacsim.core.utils.stage import is_stage_loading
ctx = omni.usd.get_context(); ctx.open_stage(a.src); stage = ctx.get_stage()
for i in range(3000):
    app.update()
    if i > 60 and not is_stage_loading(): break
TABLE = "/World/SM_TableWorkSecurity"; BASE = np.array([-1.15, -0.57])
bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
def box(p):
    r = bc.ComputeWorldBound(p).ComputeAlignedRange(); return np.array(r.GetMin()), np.array(r.GetMax())
tmin, tmax = box(stage.GetPrimAtPath(TABLE)); TOP = float(tmax[2])
print("[hive] table x", tmin[0].round(3), tmax[0].round(3), "y", tmin[1].round(3), tmax[1].round(3), "top", round(TOP, 3), flush=True)
for n in [x for x in a.drop.split(",") if x]: stage.GetPrimAtPath(f"/World/{n}").SetActive(False); print("[hive] taken off the table:", n, flush=True)
pos = np.array([BASE[0] + a.fwd, BASE[1] + a.left, TOP + a.up])
# ---- board frame = valve frame ---------------------------------------------------------------------------------------
board = UsdGeom.Xform.Define(stage, "/World/hiveboard"); bx = board.AddTranslateOp(); bx.Set(Gf.Vec3d(*pos.tolist())); board.AddRotateZOp().Set(180.0)
valve = stage.DefinePrim("/World/hiveboard/valve", "Xform"); valve.GetReferences().AddReference("assets/hiveboard/Valves/Lever Valve/Ball Valve/Ball_Valve.usd")
hc = stage.DefinePrim("/World/hiveboard/honeycomb", "Xform"); hc.GetReferences().AddReference("assets/hiveboard/Honeycomb/Honeycomb_Panel.usd")
hx = UsdGeom.Xformable(hc); hx.AddTranslateOp().Set(Gf.Vec3d(-0.04, 0, 0)); hx.AddScaleOp().Set(Gf.Vec3f(0.001, 0.001, 0.001))
for i in range(60): app.update()
# ---- pin the valve to the placed frame (its FixedJoint has body0 = world) ----------------------------------------------
fj = next(p for p in Usd.PrimRange(valve) if p.GetTypeName() == "PhysicsFixedJoint")
yaw = Gf.Rotation(Gf.Vec3d(0, 0, 1), 180.0).GetQuat()
fj.GetAttribute("physics:localPos0").Set(Gf.Vec3f(*pos.tolist()))
fj.GetAttribute("physics:localRot0").Set(Gf.Quatf(float(yaw.GetReal()), *[float(x) for x in yaw.GetImaginary()]))
# ---- a thin post from the table to the panel (my addition: the real board hangs on the lab's frame) -------------------
post = UsdGeom.Cube.Define(stage, "/World/hiveboard_post"); post.CreateSizeAttr(1.0)
hp = (pos[2] - 0.1375 - TOP); UsdGeom.Xformable(post).AddTranslateOp().Set(Gf.Vec3d(pos[0] + 0.05, pos[1], TOP + hp / 2))
UsdGeom.Xformable(post).AddScaleOp().Set(Gf.Vec3f(0.04, 0.04, float(hp))); post.CreateDisplayColorAttr([Gf.Vec3f(0.1, 0.1, 0.1)])
UsdPhysics.CollisionAPI.Apply(post.GetPrim())
bc.Clear()
def lever_angle():                                    # lever rotation about the valve x axis relative to the base [deg]
    xc = UsdGeom.XformCache(); L = next(p for p in Usd.PrimRange(valve) if p.GetName() == "alavanca_pivot"); B = next(p for p in Usd.PrimRange(valve) if p.GetName() == "base_pivot")
    R = np.array(xc.GetLocalToWorldTransform(B)).T[:3, :3].T @ np.array(xc.GetLocalToWorldTransform(L)).T[:3, :3]     # base^-1 * lever
    return float(math.degrees(math.atan2(R[2, 1], R[1, 1]))), np.array(xc.GetLocalToWorldTransform(B).ExtractTranslation())
ang0, base0 = lever_angle()
# ---- 1.5 s of physics: does the valve stay where it was put? ---------------------------------------------------------
tl = omni.timeline.get_timeline_interface(); tl.play()
for _ in range(120): app.update()
ang1, base1 = lever_angle()
res = {"valve_world_pos": pos.round(4).tolist(), "table_top": round(TOP, 3), "franka_base": [*BASE.tolist(), round(TOP, 3)],
       "base_pivot_before": base0.round(4).tolist(), "base_pivot_after": base1.round(4).tolist(), "base_moved_mm": round(float(np.linalg.norm(base1 - base0) * 1000), 3),
       "lever_deg_before": round(ang0, 2), "lever_deg_after": round(ang1, 2)}
print("[hive] RESULT " + json.dumps(res), flush=True)
tl.stop()
for _ in range(10): app.update()
stage.SetDefaultPrim(stage.GetPrimAtPath("/World")); stage.GetRootLayer().Export(a.save); print("[hive] saved", a.save, flush=True)
tl.play()                                             # the franka_scene.py renders ran with the timeline playing
# ---- renders -------------------------------------------------------------------------------------------------------------
import omni.replicator.core as rep
from PIL import Image
W, H = 1280, 720
ov = UsdGeom.Camera.Define(stage, "/World/overview_cam"); ov.CreateFocalLengthAttr(16.0); ov.CreateHorizontalApertureAttr(20.955)
ov.CreateVerticalApertureAttr(11.787); ov.CreateClippingRangeAttr(Gf.Vec2f(0.02, 200.0)); ov_op = ov.AddTransformOp()
rp = rep.create.render_product("/World/overview_cam", (W, H)); an = rep.AnnotatorRegistry.get_annotator("rgb"); an.attach([rp])
def look(eye, at):
    eye, at = np.array(eye, float), np.array(at, float); f = at - eye; f /= np.linalg.norm(f)
    r = np.cross(f, [0, 0, 1.0]); r /= np.linalg.norm(r); u = np.cross(r, f)
    M = np.eye(4); M[:3, 0], M[:3, 1], M[:3, 2], M[:3, 3] = r, u, -f, eye; ov_op.Set(Gf.Matrix4d(*M.T.flatten().tolist()))
for name, (eye, at) in {"hive_overview": (pos + [-2.2, -1.9, 0.9], pos + [-0.8, 0.0, -0.35]), "hive_side": (pos + [-0.3, -2.0, 0.1], pos + [-0.4, 0.0, -0.2]),
                        "hive_closeup": (pos + [-0.55, -0.35, 0.12], pos + [0.0, 0.0, 0.0]), "hive_top": (pos + [-0.9, 0.0, 1.6], pos + [-0.5, 0.0, -0.4])}.items():
    look(eye, at)
    for _ in range(16): app.update()
    for _ in range(60):
        d = an.get_data(); d = d.get("data") if isinstance(d, dict) else d
        if d is not None and np.asarray(d).size > 0: break
        app.update()
    Image.fromarray(np.asarray(d)[..., :3]).save(os.path.join(a.out, f"{name}.jpg"), quality=90)
json.dump(res, open(os.path.join(a.out, "hive_scene.json"), "w"), indent=1)
os.system(f"chown -R 1000:1000 {a.out} {a.save}")
import sys; sys.stdout.flush(); os._exit(0)
