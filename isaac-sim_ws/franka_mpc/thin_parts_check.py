"""Are the objects' thin tops (bottle caps, tops of tall objects) in the map? For every object except the target: sample the true mesh (objects_start.npz,
world frame -> base frame) in the top TOPCM cm of the object, look up the saved ESDF (map_check esdf.npz) at each sample and report the share that is
  mapped   |E| <= 1.5 voxel   (the surface is in the map)
  carved   E  >  1.5 voxel     (observed FREE where an object is: the thin part was lost, the arm/MPC would go through it)
  unobs    E <= -999            (never observed: cuRobo treats it as an obstacle)
   python3 thin_parts_check.py runs/<run> [runs/<run2> ...] [--map pre] [--target drill] [--topcm 6]      (any python with numpy + scipy)"""
import argparse, json, os
import numpy as np
ap = argparse.ArgumentParser(); ap.add_argument("runs", nargs="+"); ap.add_argument("--map", default="pre"); ap.add_argument("--target", default="drill")
ap.add_argument("--topcm", type=float, default=6.0); ap.add_argument("--json", default=""); a = ap.parse_args()
HERE = os.path.dirname(os.path.abspath(__file__))
root = np.array(json.load(open(os.path.join(HERE, "runs", "workcell", "scene.json")))["franka_base"]); out = {}
for r in a.runs:
    f = os.path.join(r, f"map_{a.map}", "esdf.npz")
    if not os.path.exists(f): print(r, "no esdf.npz"); continue
    Zm = np.load(f); E, org, vs = Zm["esdf"], Zm["origin"], float(Zm["voxel"]); O = np.load(os.path.join(r, "objects_start.npz"))
    print(f"{os.path.basename(r.rstrip('/'))}  voxel {vs:.3f} m, top {a.topcm} cm of each object")
    for k in O.files:
        if k == a.target: continue
        T = O[k].astype(np.float64).reshape(-1, 3, 3) - root                      # triangles in the base frame
        w = np.random.default_rng(0).dirichlet([1, 1, 1], size=(len(T), 6)); P = np.einsum("tsk,tkd->tsd", w, T).reshape(-1, 3)   # 6 samples per triangle
        P = P[P[:, 2] > P[:, 2].max() - a.topcm / 100]
        ij = np.floor((P - org) / vs).astype(int); ok = np.all((ij >= 0) & (ij < np.array(E.shape)), 1)
        if ok.sum() < 20: print(f"   {k:28s} outside the dumped region ({int(ok.sum())} samples inside)"); continue
        v = E[tuple(ij[ok].T)]; n = len(v)
        d = {"samples": n, "mapped": round(float((np.abs(v) <= 1.5 * vs).sum()) / n, 3), "carved": round(float(((v > 1.5 * vs)).sum()) / n, 3),
             "unobs": round(float((v <= -999).sum()) / n, 3), "top_z_base": round(float(P[:, 2].max()), 3)}
        out[f"{os.path.basename(r.rstrip('/'))}/{k}"] = d; print(f"   {k:28s} {d}")
if a.json: json.dump(out, open(a.json, "w"), indent=1)
