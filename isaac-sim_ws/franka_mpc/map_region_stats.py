"""What does a map dump (map_check.py esdf.npz, robot base frame) contain in the grasp region? Fractions of observed-free / occupied / UNOBSERVED
(-1000 = the service's value for unobserved) in (a) the whole dumped region, (b) the working volume above the table, and (c) the target's true bbox.
   python3 map_region_stats.py runs/<run> [runs/<run2> ...] [--maps pre post] [--target drill] [--json out.json]      (any python with numpy)"""
import argparse, json, os
import numpy as np
ap = argparse.ArgumentParser(); ap.add_argument("runs", nargs="+"); ap.add_argument("--maps", nargs="+", default=["pre"]); ap.add_argument("--target", default="drill")
ap.add_argument("--json", default=""); a = ap.parse_args()
HERE = os.path.dirname(os.path.abspath(__file__))
root = np.array(json.load(open(os.path.join(HERE, "runs", "workcell", "scene.json")))["franka_base"])
res = {}
for r in a.runs:
    tri = np.load(os.path.join(r, "objects_start.npz"))[a.target].astype(np.float64).reshape(-1, 3) - root
    lo, hi = tri.min(0), tri.max(0)
    for m in a.maps:
        f = os.path.join(r, f"map_{m}", "esdf.npz")
        if not os.path.exists(f): print(r, m, "no esdf.npz"); continue
        Z = np.load(f); E, org, vs = Z["esdf"], Z["origin"], float(Z["voxel"])
        P = org + (np.stack(np.meshgrid(*[np.arange(n) for n in E.shape], indexing="ij"), -1) + 0.5) * vs
        unobs, occ = E <= -999.0, (E > -999.0) & (E <= 0.0); free = E > 0.0
        work = P[..., 2] > 0.02                                    # above the table top (z = 0 in the base frame)
        box = np.all((P > lo + 0.01) & (P < hi - 0.01), -1)
        d = {"voxel_m": vs, "shape": list(E.shape)}
        for name, sel in (("region", np.ones_like(work)), ("above_table", work), ("target_bbox", box)):
            n = max(int(sel.sum()), 1)
            d[name] = {"voxels": int(sel.sum()), "free": round(float((free & sel).sum()) / n, 4), "occupied": round(float((occ & sel).sum()) / n, 4),
                       "unobserved": round(float((unobs & sel).sum()) / n, 4)}
        res[f"{os.path.basename(r.rstrip('/'))}/{m}"] = d
        print(f"{os.path.basename(r.rstrip('/'))}/{m}: voxel {vs} shape {E.shape}")
        for k in ("region", "above_table", "target_bbox"): print(f"   {k:12s} {d[k]}")
if a.json: json.dump(res, open(a.json, "w"), indent=1)
