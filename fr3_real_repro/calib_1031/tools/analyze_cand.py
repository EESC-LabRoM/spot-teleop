import numpy as np
R = np.load("/s/objects_raw.npz"); names = {"o1": "bottle", "o2": "drill", "o3": "box", "o4": "wall panel"}
MAPS = [("LIVE MPC map (5 cm, weights 0.5)", "/s/esdf_live.npz"), ("A: 5 cm, min weights 0.1", "/s/esdf_cand_a.npz"), ("B: 3 cm, min weights 0.5", "/s/esdf_cand_b.npz"), ("C: 3 cm, min weights 0.1", "/s/esdf_cand_c.npz")]
def load(p): Z = np.load(p); return Z["esdf"], Z["origin"].astype(float), float(Z["voxel"])
def at(E, O, V, Q):
    i = np.floor((Q - O) / V).astype(int); ok = ((i >= 0) & (i < np.array(E.shape))).all(1); v = np.full(len(Q), np.nan); v[ok] = E[tuple(i[ok].T)]; return v
print("belief error at the object's TRUE surface = what the MPC thinks the clearance is where the real clearance is 0:  E - 0.5*voxel  (cm; unobserved counts as free = +inf)")
print(f"{'map':36s} {'object':10s} {'median':>7s} {'p90':>7s} {'hidden(>1cm)':>13s} {'unobs':>6s}   occupied-voxel top vs raw top (cm)")
for lab, p in MAPS:
    E, O, V = load(p)
    for k in ("o1", "o2", "o3", "o4"):
        P = R[k]; v = at(E, O, V, P); unobs = (v < -999); known = np.isfinite(v)
        d = np.where(unobs, 1000.0, v - 0.5 * V)[known]
        # top of the occupied voxels over the object's footprint vs the raw top
        c = np.median(P[:, :2], 0); half = np.maximum(np.percentile(P[:, :2], 99, 0) - np.percentile(P[:, :2], 1, 0), 0.06) / 2 + 0.03
        ii = np.argwhere((E > -999) & (E <= 0)); C = O + (ii + 0.5) * V; m = (abs(C[:, 0] - c[0]) < half[0]) & (abs(C[:, 1] - c[1]) < half[1]) & (C[:, 2] > 0.08)
        top = (C[m, 2].max() + V / 2) if m.any() else np.nan; raw_top = np.percentile(P[:, 2], 99)
        print(f"{lab:36s} {names[k]:10s} {100*np.median(d):7.1f} {100*np.percentile(d,90):7.1f} {100*np.mean(d>0.01):12.0f}% {100*np.mean(unobs[known]):5.0f}%   {100*top:5.1f} vs {100*raw_top:5.1f}  ({m.sum()} voxels above 8 cm)")
    print()
