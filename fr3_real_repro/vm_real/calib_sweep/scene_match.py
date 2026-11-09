"""Gate for the calibration sweep (objects stay on the table): the sweep was planned against a depth snapshot (calib_sweep/world/segcloud.npz). Refuse if the
scene now has NEW stuff: live robot-free depth points of either camera, inside the volume the arm sweeps, that are > 6 cm from every snapshot point
(snapshot = both cameras, so the small camera disagreement does not trigger it).   scene_match.py <live_segcloud.npz> <planning_segcloud.npz>"""
import sys, numpy as np
from scipy.spatial import cKDTree
S0 = np.load(sys.argv[2]); ref = np.vstack([S0[k] for k in S0.files]); T = cKDTree(ref[np.unique(np.floor(ref / 0.01).astype(int), axis=0, return_index=True)[1]])
S = np.load(sys.argv[1]); bad = False
def region(P): return P[(P[:, 0] > 0.10) & (P[:, 0] < 0.90) & (np.abs(P[:, 1]) < 0.70) & (P[:, 2] > 0.09) & (P[:, 2] < 0.95)]
for c in ("zed1", "zed2"):
    if c not in S.files: print(f"  {c}: no cloud"); bad = True; continue
    P = region(S[c]); d, _ = T.query(P, distance_upper_bound=0.5); new = d > 0.06
    print(f"  {c}: {len(P)} points in the sweep volume, {int(new.sum())} of them > 6 cm from anything in the planning snapshot" + (f" (centre {np.round(P[new].mean(0), 2).tolist()})" if new.any() else ""))
    if new.sum() > 300: bad = True
print("SCENE_MATCHES_PLAN" if not bad else "SCENE_CHANGED"); sys.exit(1 if bad else 0)
