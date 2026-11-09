"""How well the nvblox map matches each object: the map's ESDF (m, + outside) read at dense samples of the true surface
(Isaac's meshes, objects_start.npz). 0 = the mapped surface is where the object really is; + = the map thinks that
spot of the real surface is free space (the mapped object is smaller there: what can make the arm touch it); - = the
map is fatter. Split into the half facing the front ZED (+x) and the half facing the robot (-x, which a single front
camera cannot see). Unobserved voxels count as free (as the MPC node treats them).
   curobo-isaac6/.venv/bin/python -I map_coverage.py runs/A [runs/B ...]      -> table, <run>/map_error.json
"""
import json, os, sys

import numpy as np

for run in sys.argv[1:]:
    M = dict(np.load(os.path.join(run, "objects_start.npz"))); rj = os.path.join(run, "RESULT_isaac.json")   # MAP_ONLY runs have none
    root = np.array(json.load(open(rj))["root"] if os.path.exists(rj) else json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "runs", "workcell", "scene.json")))["franka_base"])
    d = np.load(os.path.join(run, "map_check", "esdf.npz")); E, o, v = d["esdf"], d["origin"], float(d["voxel"])
    E = np.where(E < -999, 2.0, E)

    def esdf_at(P):                                            # trilinear on voxel centres
        g = (P - o) / v - 0.5; i0 = np.floor(g).astype(int); f = g - i0; out = np.zeros(len(P))
        for dx in (0, 1):
            for dy in (0, 1):
                for dz in (0, 1):
                    i = np.clip(i0 + [dx, dy, dz], 0, np.array(E.shape) - 1); w = np.prod(np.where([dx, dy, dz], f, 1 - f), axis=1)
                    out += w * E[i[:, 0], i[:, 1], i[:, 2]]
        return out
    rng = np.random.default_rng(0); out = {}
    for name, tri in M.items():
        tri = tri.astype(np.float64) - root
        cr = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]); area = 0.5 * np.linalg.norm(cr, axis=1)
        n = 20000; f = rng.choice(len(tri), n, p=area / area.sum()); u, w = rng.random(n), rng.random(n); s = u + w > 1
        u[s], w[s] = 1 - u[s], 1 - w[s]
        P = tri[f, 0] + u[:, None] * (tri[f, 1] - tri[f, 0]) + w[:, None] * (tri[f, 2] - tri[f, 0])
        P = P[P[:, 2] > 0.02]                                  # skip the rim on the table
        e = esdf_at(P); cx = 0.5 * (P[:, 0].min() + P[:, 0].max()); fr = P[:, 0] >= cx
        r = lambda m: {"median_cm": round(float(np.median(e[m])) * 100, 1), "p90_cm": round(float(np.percentile(e[m], 90)) * 100, 1),
                       "missed_over_2cm_pct": round(float((e[m] > 0.02).mean()) * 100, 1)}
        out[name] = {"front_half": r(fr), "robot_side_half": r(~fr)}
    json.dump({"objects": out}, open(os.path.join(run, "map_error.json"), "w"), indent=1)
    print(f"\n{os.path.basename(run)}: map ESDF at the true surface (cm; + = map thinks it is free)   front half | robot side")
    for k, r_ in out.items():
        a, b = r_["front_half"], r_["robot_side_half"]
        print(f"  {k:28s} median {a['median_cm']:+5.1f} p90 {a['p90_cm']:+5.1f} missed {a['missed_over_2cm_pct']:5.1f}% | "
              f"median {b['median_cm']:+5.1f} p90 {b['p90_cm']:+5.1f} missed {b['missed_over_2cm_pct']:5.1f}%")
