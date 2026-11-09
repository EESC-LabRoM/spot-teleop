"""Where does the bottle's cap get lost? (1) in the ZED depth: back-project one depth frame per camera (raw and robot-
filtered) into the base frame and count the points that land on the real cap (within 1 cm of Isaac's mesh above z 0.25);
(2) in the map: read each saved ESDF (map_check*/esdf.npz, e.g. 2 cm and 1 cm voxels) on the cap's true surface.
   curobo-isaac6/.venv/bin/python -I cap_check.py runs/mapcheck_2zed      -> prints, <run>/cap_check.json
"""
import glob, json, os, sys

import numpy as np
from scipy.spatial import cKDTree

run = os.path.abspath(sys.argv[1]); Z = "/home/nexus/spot-teleop/zed_lite/nvblox/out"
root = np.array(json.load(open(os.path.join(run, "RESULT_isaac.json")))["root"]) if os.path.exists(os.path.join(run, "RESULT_isaac.json")) \
    else np.array(json.load(open("/home/nexus/spot-teleop/isaac-sim_ws/franka_mpc/runs/workcell/scene.json"))["franka_base"])
tri = np.load(os.path.join(run, "objects_start.npz"))["veja"].astype(np.float64) - root
rng = np.random.default_rng(0); cr = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]); area = 0.5 * np.linalg.norm(cr, axis=1)
n = 200000; f = rng.choice(len(tri), n, p=area / area.sum()); u, w = rng.random(n), rng.random(n); s = u + w > 1; u[s], w[s] = 1 - u[s], 1 - w[s]
P = tri[f, 0] + u[:, None] * (tri[f, 1] - tri[f, 0]) + w[:, None] * (tri[f, 2] - tri[f, 0])
ztop = P[:, 2].max(); CAP = P[P[:, 2] > ztop - 0.04]; BODY = P[(P[:, 2] > 0.10) & (P[:, 2] < 0.20)]
print(f"bottle: top at z {ztop:.3f}; cap region = top 4 cm, {np.ptp(CAP[:, 0])*100:.1f} x {np.ptp(CAP[:, 1])*100:.1f} cm across")
res = {"top_z": round(float(ztop), 3), "depth": {}, "map": {}}
tc, tb = cKDTree(CAP), cKDTree(BODY)

D = np.load(os.path.join(run, "depth.npz"), allow_pickle=True)
for key in sorted(k for k in D.files if k.endswith("_depth")):
    name = key[:-6]; cam = name.split("_")[0]; dep = D[key]; K = D.get(name + "_K")
    ext = json.load(open(os.path.join(Z, f"extrinsics_{cam}.json"))); T = np.array(ext["base_T_left_optical"])
    if K is None: continue
    sx = dep.shape[1] / (2 * K[0, 2]) if K[0, 2] > 0 else 1.0    # K may be for another resolution
    v, u_ = np.nonzero(np.isfinite(dep) & (dep > 0.05)); z = dep[v, u_]
    x = (u_ - K[0, 2] * sx) / (K[0, 0] * sx) * z; y = (v - K[1, 2] * sx) / (K[1, 1] * sx) * z
    Pb = (T[:3, :3] @ np.stack([x, y, z])).T + T[:3, 3]
    dc, _ = tc.query(Pb, distance_upper_bound=0.01); db, _ = tb.query(Pb, distance_upper_bound=0.01)
    near_cap = Pb[np.isfinite(dc)]
    # depth bias on the body: signed along the camera ray is hard; use distance of near points to the true surface
    res["depth"][name] = {"points_on_cap_1cm": int(np.isfinite(dc).sum()), "points_on_body_band_1cm": int(np.isfinite(db).sum()),
                          "image": list(dep.shape)}
    print(f"depth {name:13s}: {np.isfinite(dc).sum():5d} points within 1 cm of the cap, {np.isfinite(db).sum():6d} on the body band (z 0.10-0.20)")

for mdir in sorted(glob.glob(os.path.join(run, "map_check*"))):
    if not os.path.exists(os.path.join(mdir, "esdf.npz")): continue
    d = np.load(os.path.join(mdir, "esdf.npz")); E, o, vx = d["esdf"], d["origin"], float(d["voxel"]); E = np.where(E < -999, 2.0, E)
    def at(Q):
        g = (Q - o) / vx - 0.5; i0 = np.floor(g).astype(int); fr = g - i0; out = np.zeros(len(Q))
        for dx in (0, 1):
            for dy in (0, 1):
                for dz in (0, 1):
                    i = np.clip(i0 + [dx, dy, dz], 0, np.array(E.shape) - 1); ww = np.prod(np.where([dx, dy, dz], fr, 1 - fr), axis=1)
                    out += ww * E[i[:, 0], i[:, 1], i[:, 2]]
        return out
    ec, eb = at(CAP), at(BODY)
    occ = o + (np.argwhere(E < 0) + 0.5) * vx; near = occ[(np.abs(occ[:, 0] - 0.55) < 0.06) & (np.abs(occ[:, 1] - 0.02) < 0.06)]
    res["map"][os.path.basename(mdir)] = {"voxel_m": round(vx, 3), "cap_esdf_median_cm": round(float(np.median(ec)) * 100, 1),
        "cap_missed_over_2cm_pct": round(float((ec > 0.02).mean()) * 100, 1), "body_esdf_median_cm": round(float(np.median(eb)) * 100, 1),
        "mapped_top_z": round(float(near[:, 2].max()), 3) if len(near) else None}
    print(f"map {os.path.basename(mdir):15s} voxel {vx*100:.0f} cm: ESDF on the cap median {np.median(ec)*100:+.1f} cm, "
          f"missed >2 cm {(ec > 0.02).mean()*100:.0f}% | body median {np.median(eb)*100:+.1f} cm | mapped top z "
          f"{near[:, 2].max() if len(near) else float('nan'):.3f} (true {ztop:.3f})")
json.dump(res, open(os.path.join(run, "cap_check.json"), "w"), indent=1)
