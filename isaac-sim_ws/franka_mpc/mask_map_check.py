"""Is the target out of nvblox's static map? Occupied ESDF voxels (distance <= 0) of the grasp region (map_check.py output, robot base frame) inside the
target's true bounding box, per map dump, plus the ESDF value along the target's centre column.
   /usr/bin/python3 mask_map_check.py runs/<run> [--target drill] [--maps pre post]   (run with the graspgenx venv python or any numpy)"""
import argparse, json, os
import numpy as np
ap = argparse.ArgumentParser(); ap.add_argument("run"); ap.add_argument("--target", default="drill"); ap.add_argument("--maps", nargs="+", default=["pre", "post"]); a = ap.parse_args()
HERE = os.path.dirname(os.path.abspath(__file__))
root = np.array(json.load(open(os.path.join(HERE, "runs", "workcell", "scene.json")))["franka_base"])
tri = np.load(os.path.join(a.run, "objects_start.npz"))[a.target].astype(np.float64).reshape(-1, 3) - root
lo, hi = tri.min(0), tri.max(0); print(f"{a.target} true bbox (base frame) {np.round(lo, 3).tolist()} .. {np.round(hi, 3).tolist()}")
out = {}
for m in a.maps:
    f = os.path.join(a.run, f"map_{m}", "esdf.npz")
    if not os.path.exists(f): continue
    Z = np.load(f); E, org, vs = Z["esdf"], Z["origin"], float(Z["voxel"]); idx = np.stack(np.meshgrid(*[np.arange(n) for n in E.shape], indexing="ij"), -1)
    P = org + (idx + 0.5) * vs; inb = np.all((P > lo + 0.01) & (P < hi - 0.01), -1)          # 1 cm inside the true box (skip the surface layer)
    # -1000 is the service's value for unobserved voxels
    occ = (E > -999.0) & (E <= 0.0);  tot = int(inb.sum()); n_occ = int((occ & inb).sum())
    # surfaces inside the box = mapped surface voxels near zero distance: the table top sits at z ~ 0, the drill's body above it
    surf = (E > -999.0) & (np.abs(E) < vs) & inb & (P[..., 2] > 0.02)
    out[m] = {"voxels_in_box": tot, "occupied_in_box": n_occ, "occupied_frac": round(n_occ / max(tot, 1), 4), "surface_voxels_above_table": int(surf.sum()),
              "unobserved_in_box": int(((E <= -999.0) & inb).sum())}
    print(m, out[m])
json.dump(out, open(os.path.join(a.run, "mask_map_check.json"), "w"), indent=1)
