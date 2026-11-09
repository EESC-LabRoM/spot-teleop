"""Batch input: the K best VALID grasps of grasp_filter.py's pool as separate grasp_goal.py jsons (same fields as <obj>_selected.json),
ranked by grasp_filter's score (confidence + 0.15 * approach-down). run_batch.sh runs one trial per json.
   python3 grasp_variants.py runs/<run with grasps/> --object drill --k 8 --out runs/<prefix>_grasps
"""
import argparse, json, os
import numpy as np
from scipy.spatial.transform import Rotation

ap = argparse.ArgumentParser(); ap.add_argument("run"); ap.add_argument("--object", default="drill"); ap.add_argument("--k", type=int, default=8)
ap.add_argument("--out", required=True); a = ap.parse_args()
sel = json.load(open(os.path.join(a.run, "grasps", f"{a.object}_selected.json")))
S = np.load(os.path.join(a.run, "grasps", f"{a.object}_status.npz")); G, conf = S["grasps"], S["conf"]
pool = np.array(sel["valid_indices"]); down = -G[pool][:, 2, 2]; score = conf[pool] + 0.15 * np.clip(down, 0, 1)
order = pool[np.argsort(-score)][:a.k]; os.makedirs(a.out, exist_ok=True); pre = sel["pregrasp_m"]
def pose7(p, R): x, y, z, w = Rotation.from_matrix(R).as_quat(); return [float(v) for v in (*p, x, y, z, w)]
for k, i in enumerate(order):
    R, t = G[i][:3, :3], G[i][:3, 3]
    d = {key: sel[key] for key in ("object", "target_center", "target_half", "pregrasp_m")}
    d.update(index=int(S["src"][i]), flipped=bool(S["flip"][i]), confidence=float(conf[i]), approach_down=float(-R[2, 2]), finger_gap_m=float(S["width"][i]),
             grasp=pose7(t, R), pregrasp=pose7(t - pre * R[:, 2], R), tcp_grasp=(t + 0.1034 * R[:, 2]).tolist(), rank=k)
    json.dump(d, open(os.path.join(a.out, f"g{k:02d}.json"), "w"), indent=1)
    print(f"g{k:02d}: GraspGen-X #{d['index']}{' (flipped)' if d['flipped'] else ''} conf {d['confidence']:.3f} down {d['approach_down']:.2f}")
