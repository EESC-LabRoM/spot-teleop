"""Check the segmentation clouds against the sim's exact meshes: distance of each cloud point to the true surface (base frame)."""
import json, os, sys
import numpy as np
from scipy.spatial import cKDTree
run = sys.argv[1]; HERE = os.path.dirname(os.path.abspath(__file__))
root = np.array(json.load(open(os.path.join(HERE, "runs", "workcell", "scene.json")))["franka_base"])
O = dict(np.load(os.path.join(run, "objects_start.npz"))); rng = np.random.default_rng(0); out = {}
for n, tri in O.items():
    tri = tri.astype(np.float64) - root; Z = np.load(os.path.join(run, "clouds", f"{n}.npz"))
    cr = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]); ar = 0.5 * np.linalg.norm(cr, axis=1); ok = ar > 1e-12
    f = rng.choice(np.nonzero(ok)[0], 200000, p=ar[ok] / ar[ok].sum()); w1, w2 = rng.random(len(f)), rng.random(len(f)); s = w1 + w2 > 1
    w1[s], w2[s] = 1 - w1[s], 1 - w2[s]; S = tri[f, 0] + w1[:, None] * (tri[f, 1] - tri[f, 0]) + w2[:, None] * (tri[f, 2] - tri[f, 0])
    t = cKDTree(S); r = {}
    for k in [x for x in Z.files if x.startswith("pc_")]:
        P = Z[k]
        if len(P) == 0: r[k[3:]] = None; continue
        d, _ = t.query(P); r[k[3:]] = {"points": int(len(P)), "median_mm": round(float(np.median(d)) * 1e3, 1), "p95_mm": round(float(np.percentile(d, 95)) * 1e3, 1)}
    out[n] = r; print(n, r)
json.dump(out, open(os.path.join(run, "clouds", "check.json"), "w"), indent=1)
