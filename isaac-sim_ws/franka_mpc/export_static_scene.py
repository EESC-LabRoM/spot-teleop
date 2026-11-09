# Export the workcell's static geometry around the table (world-frame triangles of every visible mesh: the warehouse
# props, the table, shelves, floor) for the offline ZED placement search (zed_placement.py). The tabletop objects, the
# Franka, the ZEDs and our markers are left out: the objects come from a run's objects_start.npz, the arm from cuRobo.
# The warehouse references NVIDIA's S3 assets, which only Isaac's resolver can open, so this runs in Isaac:
#   ./run_isaac.sh -s export_static_scene.py static_scene        -> runs/static_scene/static_scene.npz
import argparse, json, os, time
HERE = os.path.dirname(os.path.abspath(__file__))
ap = argparse.ArgumentParser()
ap.add_argument("--out", required=True)
ap.add_argument("--stage", default=os.path.join(HERE, "franka_workcell.usda"))
ap.add_argument("--box", type=float, nargs=6, default=[-4.5, -4.0, -0.1, 2.5, 3.0, 3.0], help="world xmin ymin zmin xmax ymax zmax")
a = ap.parse_args(); os.makedirs(a.out, exist_ok=True)
from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "width": 320, "height": 240, "multi_gpu": False})
import numpy as np
import omni.usd
from pxr import Usd, UsdGeom
ctx = omni.usd.get_context(); ctx.open_stage(a.stage)
t0 = time.time()
while ctx.get_stage_loading_status()[2] > 0 or time.time() - t0 < 5: app.update()
stage = ctx.get_stage()
OBJ = list(json.load(open(os.path.join(HERE, "runs", "workcell", "scene.json")))["objects"])
skip = tuple(f"/World/{n}" for n in OBJ + ["franka", "zed", "mpc_"]) + ("/Render", "/OmniverseKit")   # prefixes ("zed" covers zed2, stands)
lo, hi = np.array(a.box[:3]), np.array(a.box[3:])
bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
out, stats, n_inst = {}, [], 0
for p in Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
    path = str(p.GetPath())
    if path.startswith(skip): continue
    if p.IsA(UsdGeom.PointInstancer): n_inst += 1; continue
    if not p.IsA(UsdGeom.Mesh) or UsdGeom.Imageable(p).ComputeVisibility() == UsdGeom.Tokens.invisible: continue
    if UsdGeom.Imageable(p).ComputePurpose() in (UsdGeom.Tokens.guide, UsdGeom.Tokens.proxy): continue
    r = bc.ComputeWorldBound(p).ComputeAlignedRange()
    if r.IsEmpty() or np.any(np.array(r.GetMax()) < lo) or np.any(np.array(r.GetMin()) > hi): continue
    m = UsdGeom.Mesh(p); P = np.array(m.GetPointsAttr().Get() or [], float)
    cnt = np.array(m.GetFaceVertexCountsAttr().Get() or [], int); idx = np.array(m.GetFaceVertexIndicesAttr().Get() or [], int)
    if not len(P) or not len(cnt): continue
    W = np.array(UsdGeom.Xformable(p).ComputeLocalToWorldTransform(Usd.TimeCode.Default())).T
    Pw = (W[:3, :3] @ P.T).T + W[:3, 3]
    off = np.concatenate([[0], np.cumsum(cnt)[:-1]]); nt = np.maximum(cnt - 2, 0)      # fan-triangulate, vectorised
    f = np.repeat(np.arange(len(cnt)), nt); k = np.arange(nt.sum()) - np.repeat(np.cumsum(nt) - nt, nt) + 1
    T = Pw[idx[np.stack([off[f], off[f] + k, off[f] + k + 1], 1)]].astype(np.float32)
    top = "/".join(path.split("/")[:3]); out.setdefault(top, []).append(T)
    stats.append((path, len(T), [round(v, 2) for v in r.GetMin()], [round(v, 2) for v in r.GetMax()]))
arr = {k.replace("/", "|"): np.concatenate(v) for k, v in out.items()}
np.savez_compressed(os.path.join(a.out, "static_scene.npz"), **arr)
json.dump({"box": a.box, "point_instancers_skipped": n_inst, "groups": {k: int(len(v)) for k, v in arr.items()},
           "meshes": stats}, open(os.path.join(a.out, "static_scene.json"), "w"), indent=0)
print(f"[export] {len(stats)} meshes, {sum(len(v) for v in arr.values())} triangles in {len(arr)} groups; "
      f"point instancers skipped: {n_inst}", flush=True)
for k, v in sorted(arr.items(), key=lambda kv: -len(kv[1]))[:25]:
    print(f"   {k:45s} {len(v):8d} tris  x {v[..., 0].min():6.2f}..{v[..., 0].max():6.2f}  y {v[..., 1].min():6.2f}..{v[..., 1].max():6.2f}"
          f"  z {v[..., 2].min():5.2f}..{v[..., 2].max():5.2f}", flush=True)
app.close()
