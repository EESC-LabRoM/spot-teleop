# Load the spot-sim warehouse (simple_warehouse + tables + manipulable clutter) in Isaac Sim 6.0, list the tables and the
# rigid objects with their world boxes after 2 s of physics (so we see where they rest), and render a few views of the
# table area. Output: <out>/inventory.json, <out>/view_*.jpg.
#   ./run_isaac.sh <name> is for franka_mpc_isaac.py; run this one with:  ./run_isaac.sh -s scene_inventory.py <name>
import argparse, json, math, os
ap = argparse.ArgumentParser()
ap.add_argument("--out", required=True)
ap.add_argument("--warehouse", default="/workspace/spot-sim/assets/clutter/warehouse.usd")
a = ap.parse_args(); os.makedirs(a.out, exist_ok=True)

from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "renderer": "RayTracedLighting", "width": 1280, "height": 720, "multi_gpu": False})
import numpy as np, omni.usd, omni.timeline
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics
from isaacsim.core.utils.stage import is_stage_loading

ctx = omni.usd.get_context(); ctx.new_stage(); stage = ctx.get_stage()
stage.GetRootLayer().subLayerPaths.append(a.warehouse)
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z); UsdGeom.SetStageMetersPerUnit(stage, 1.0)
for i in range(3000):                                  # assets stream from the S3 asset server
    app.update()
    if i > 60 and not is_stage_loading(): break
print("[scene] loaded after", i, "updates", flush=True)

def boxes():
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    out = {}
    world = stage.GetPrimAtPath("/World")
    for p in world.GetChildren():
        name = p.GetName(); sub = list(Usd.PrimRange(p))
        rb = [q for q in sub if q.HasAPI(UsdPhysics.RigidBodyAPI)]
        if not ("table" in name.lower() or rb or name in ("drill", "veja")): continue
        r = cache.ComputeWorldBound(p).ComputeAlignedRange()
        if r.IsEmpty(): continue
        mass = None
        for q in sub:
            if q.HasAPI(UsdPhysics.MassAPI):
                v = UsdPhysics.MassAPI(q).GetMassAttr().Get()
                if v: mass = round(float(v), 3)
        out[name] = {"min": [round(v, 3) for v in r.GetMin()], "max": [round(v, 3) for v in r.GetMax()],
                     "rigid": bool(rb), "mass_kg": mass}
    return out

before = boxes()
tl = omni.timeline.get_timeline_interface(); tl.play()
for _ in range(120): app.update()                       # ~2 s at 60 Hz: objects settle (or fall, which we want to know)
after = boxes()
moved = {k: round(float(np.linalg.norm(np.array(after[k]["min"]) - np.array(before[k]["min"]))), 3) for k in after if k in before}
for k in after: after[k]["moved_m"] = moved.get(k)
json.dump(after, open(os.path.join(a.out, "inventory.json"), "w"), indent=1)
for k, v in sorted(after.items(), key=lambda kv: kv[1]["min"][0]):
    print(f"[scene] {k:34s} min {v['min']} max {v['max']} rigid {v['rigid']} mass {v['mass_kg']} moved {v['moved_m']}", flush=True)

# views of the area around the drill's table
import omni.replicator.core as rep
from PIL import Image
d = after.get("drill"); c = (np.array(d["min"]) + np.array(d["max"])) / 2 if d else np.array([-1.0, -0.7, 0.7])
cam = UsdGeom.Camera.Define(stage, "/World/inv_cam"); cam.CreateFocalLengthAttr(14.0); cam.CreateHorizontalApertureAttr(20.955)
cam.CreateVerticalApertureAttr(11.787); cam.CreateClippingRangeAttr(Gf.Vec2f(0.02, 200.0)); op = cam.AddTransformOp()
rp = rep.create.render_product("/World/inv_cam", (1280, 720)); annot = rep.AnnotatorRegistry.get_annotator("rgb"); annot.attach([rp])
def look(eye, at):
    f = np.array(at, float) - np.array(eye, float); f /= np.linalg.norm(f); r = np.cross(f, [0, 0, 1.0]); r /= np.linalg.norm(r)
    u = np.cross(r, f); M = np.eye(4); M[:3, 0], M[:3, 1], M[:3, 2], M[:3, 3] = r, u, -f, eye
    op.Set(Gf.Matrix4d(*M.T.flatten().tolist()))
views = {"from_minus_x": c + [-2.2, 0.0, 1.0], "from_plus_x": c + [2.2, 0.3, 1.0], "from_minus_y": c + [0.2, -2.2, 1.0],
         "from_plus_y": c + [0.2, 2.2, 1.0], "top": c + [0.01, 0.0, 3.5], "wide": c + [-4.5, 3.5, 3.0]}
for name, eye in views.items():
    look(eye, c)
    for _ in range(12): app.update()
    img = annot.get_data()
    if isinstance(img, dict): img = img.get("data")
    if img is not None and getattr(img, "size", 0):
        Image.fromarray(np.asarray(img)[..., :3]).save(os.path.join(a.out, f"view_{name}.jpg"), quality=88)
print("[scene] drill centre", c.round(3).tolist(), flush=True)
os.system(f"chown -R 1000:1000 {a.out}")
tl.stop(); import sys; sys.stdout.flush(); os._exit(0)
