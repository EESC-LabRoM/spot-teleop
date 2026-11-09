"""Export a run's 3D data for the map page (web viewer, three.js): the recorded nvblox mesh(es) as GLB (vertex colors,
triangles ordered by when their block was first meshed, so a slider can replay the reconstruction), the true object
surfaces as a GLB point cloud, and a JSON with the ZED camera poses, the fingertip paths and the reconstruction timeline.
Coordinates: the robot base frame (fr3_link0, Z up) turned to glTF's Y up as (x, z, -y).
   /usr/bin/python3 glb_export.py runs/<run> <out_dir> [--crop xmin ymin zmin xmax ymax zmax]
"""
import argparse, json, os, re, struct
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("run"); ap.add_argument("out")
ap.add_argument("--crop", type=float, nargs=6, default=[-0.70, -1.25, -0.30, 1.45, 1.10, 1.40])
a = ap.parse_args(); os.makedirs(a.out, exist_ok=True)
lo, hi = np.array(a.crop[:3]), np.array(a.crop[3:])
yup = lambda P: np.stack([P[:, 0], P[:, 2], -P[:, 1]], 1).astype(np.float32)


def glb(path, P, C=None, tri=None, extras=None):
    """Minimal glTF 2.0 binary: one mesh, POSITION (+ COLOR_0 uint8) (+ uint32 indices; else points)."""
    bufs, views, accs = [], [], []
    def add(arr, target, comp, typ, norm=False, minmax=False):
        b = arr.tobytes(); off = sum(len(x) for x in bufs); pad = (-len(b)) % 4
        bufs.append(b + b"\0" * pad); views.append({"buffer": 0, "byteOffset": off, "byteLength": len(b), "target": target})
        acc = {"bufferView": len(views) - 1, "componentType": comp, "count": int(len(arr)), "type": typ}
        if norm: acc["normalized"] = True
        if minmax: acc["min"] = arr.min(0).tolist(); acc["max"] = arr.max(0).tolist()
        accs.append(acc); return len(accs) - 1
    attr = {"POSITION": add(P, 34962, 5126, "VEC3", minmax=True)}
    if C is not None: attr["COLOR_0"] = add(C.astype(np.uint8), 34962, 5121, "VEC3", norm=True)
    prim = {"attributes": attr, "mode": 4 if tri is not None else 0}
    if tri is not None: prim["indices"] = add(tri.astype(np.uint32).reshape(-1), 34963, 5125, "SCALAR")
    bin_ = b"".join(bufs)
    js = {"asset": {"version": "2.0", "generator": "franka_mpc/glb_export.py"}, "scene": 0, "scenes": [{"nodes": [0]}],
          "nodes": [{"mesh": 0}], "meshes": [{"primitives": [prim]}], "buffers": [{"byteLength": len(bin_)}],
          "bufferViews": views, "accessors": accs}
    if extras: js["scenes"][0]["extras"] = extras
    j = json.dumps(js, separators=(",", ":")).encode(); j += b" " * ((-len(j)) % 4)
    with open(path, "wb") as f:
        f.write(struct.pack("<III", 0x46546C67, 2, 12 + 8 + len(j) + 8 + len(bin_)))
        f.write(struct.pack("<II", len(j), 0x4E4F534A)); f.write(j)
        f.write(struct.pack("<II", len(bin_), 0x004E4942)); f.write(bin_)
    return os.path.getsize(path)


meta = {"run": os.path.basename(os.path.abspath(a.run)), "meshes": {}}
t_ref = None
for tag in ("main", "extra"):
    p = os.path.join(a.run, f"mesh_{tag}.npz")
    if not os.path.exists(p): continue
    M = np.load(p, allow_pickle=True); V, C, T, F = M["vertices"], M["colors"], M["triangles"], M["tri_first_seen"]
    cen = V[T].mean(1); keep = np.all((cen >= lo) & (cen <= hi), axis=1); T, F = T[keep], F[keep]
    used = np.unique(T); remap = -np.ones(len(V), np.int64); remap[used] = np.arange(len(used))
    V, C, T = V[used], C[used], remap[T]
    o = np.argsort(F, kind="stable"); T, F = T[o], F[o]
    t_ref = F.min() if t_ref is None else min(t_ref, F.min())
    meta["meshes"][tag] = {"file": f"map_{tag}.glb", "triangles": int(len(T)), "vertices": int(len(V)), "t_first": float(F.min()),
                           "t_last": float(F.max()), "_F": F}
    vox = re.search(r"voxel[_ ]?([0-9.]+)", str(M.get("frame", ""))); meta["meshes"][tag]["src"] = p
    size = glb(os.path.join(a.out, f"map_{tag}.glb"), yup(V), C, T)
    print(f"{tag}: {len(T)} triangles, {len(V)} vertices -> map_{tag}.glb {size / 1e6:.1f} MB")
for tag, m in meta["meshes"].items():                         # timeline: triangles drawn by t seconds after the first block
    F = m.pop("_F") - t_ref; ts = np.linspace(0, F.max(), 120)
    m["timeline"] = {"t": ts.round(2).tolist(), "count": np.searchsorted(F, ts, side="right").tolist()}
    m["t_first"] -= t_ref; m["t_last"] -= t_ref; m.pop("src")

root = np.array(json.load(open(os.path.join(a.run, "RESULT_isaac.json")))["root"]) if os.path.exists(os.path.join(a.run, "RESULT_isaac.json")) \
    else np.array(json.load(open(os.path.join(os.path.dirname(__file__), "runs", "workcell", "scene.json")))["franka_base"])
if os.path.exists(os.path.join(a.run, "objects_start.npz")):
    O = np.load(os.path.join(a.run, "objects_start.npz")); rng = np.random.default_rng(0); P, Cc = [], []
    pal = {"veja": (40, 80, 200)}
    for k in O.files:
        tri = O[k].astype(np.float64) - root; cr = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]); ar = 0.5 * np.linalg.norm(cr, axis=1)
        n = int(np.clip(ar.sum() / 4e-6, 3000, 15000)); f = rng.choice(len(tri), n, p=ar / ar.sum()); u, w = rng.random(n), rng.random(n)
        s = u + w > 1; u[s], w[s] = 1 - u[s], 1 - w[s]
        P.append(tri[f, 0] + u[:, None] * (tri[f, 1] - tri[f, 0]) + w[:, None] * (tri[f, 2] - tri[f, 0]))
        Cc.append(np.tile(pal.get(k, (230, 120, 40)), (n, 1)))
    size = glb(os.path.join(a.out, "true_objects.glb"), yup(np.concatenate(P)), np.concatenate(Cc)); meta["true_objects"] = "true_objects.glb"
    print(f"true objects: {sum(len(p) for p in P)} points -> {size / 1e6:.1f} MB")
cams = {}
for f in sorted(os.listdir(a.run)):
    m_ = re.match(r"camera_(\w+)\.json", f)
    if m_:
        Tw = np.array(json.load(open(os.path.join(a.run, f)))["T_world_camera_usd"]); Tb = Tw.copy(); Tb[:3, 3] -= root
        cams[m_.group(1)] = {"pos": yup(Tb[None, :3, 3])[0].tolist(), "fwd": yup(-Tb[None, :3, 2])[0].tolist(), "up": yup(Tb[None, :3, 1])[0].tolist()}
meta["cameras"] = cams
if os.path.exists(os.path.join(a.run, "trace.npz")):
    T = np.load(os.path.join(a.run, "trace.npz")); TCP = 0.1034
    x, y, z, w = T["target_quat"].T
    ax = np.stack([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)], 1)
    meta["paths"] = {"commanded": yup(T["target"] + TCP * ax)[::2].round(4).tolist(), "executed": yup(T["tcp"])[::2].round(4).tolist()}
json.dump(meta, open(os.path.join(a.out, "scene.json"), "w"))
print("scene.json:", {k: (list(v) if isinstance(v, dict) else v) for k, v in meta.items() if k != "paths"})
