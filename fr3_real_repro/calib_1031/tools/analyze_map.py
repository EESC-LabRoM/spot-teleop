import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree
Z = np.load("/s/esdf_live.npz"); E, O, V = Z["esdf"], Z["origin"], float(Z["voxel"]); S = np.load("/s/segcloud.npz")
print(f"ESDF {E.shape} voxel {V} m origin {O.round(2).tolist()}  occupied {(E>-999).__and__(E<=0).sum()}  free {(E>0).sum()}  unobserved {(E<=-999).sum()}")
idx = np.argwhere((E > -999) & (E <= 0)); C = O + (idx + 0.5) * V
# table plane in the map: peak of occupied voxel z under the table region
reg = (C[:, 0] > 0.15) & (C[:, 0] < 0.95) & (abs(C[:, 1]) < 0.6)
h, e = np.histogram(C[reg, 2], bins=np.arange(-0.2, 0.6, V)); print("occupied-voxel z histogram peak (table):", round(float(0.5 * (e[np.argmax(h)] + e[np.argmax(h) + 1])), 3), "m")
ztab = 0.5 * (e[np.argmax(h)] + e[np.argmax(h) + 1])
# map clusters above the table
sel = reg & (C[:, 2] > ztab + 0.07); lab_grid = np.zeros(E.shape, int); lab_grid[tuple(idx[sel].T)] = 1
lab, n = ndimage.label(lab_grid, structure=np.ones((3, 3, 3))); print(f"\nMAP clusters above the table (z > {ztab+0.07:.2f}): {n}")
MC = []
for k in range(1, n + 1):
    ii = np.argwhere(lab == k); c = O + (ii + 0.5) * V
    if len(ii) < 2: continue
    MC.append(c); print(f"  map {k}: {len(ii):3d} voxels, centre ({c[:,0].mean():.2f},{c[:,1].mean():.2f}), x {c[:,0].min():.2f}..{c[:,0].max():.2f} y {c[:,1].min():.2f}..{c[:,1].max():.2f} z {c[:,2].min():.2f}..{c[:,2].max():.2f}")
# raw robot-free cloud clusters above the table plane (use the median table plane of the cloud itself)
P = np.vstack([S["zed1"], S["zed2"]]); r = (P[:, 0] > 0.15) & (P[:, 0] < 0.95) & (abs(P[:, 1]) < 0.6) & (P[:, 2] > -0.1) & (P[:, 2] < 0.6); P = P[r]
hh, ee = np.histogram(P[:, 2], bins=np.arange(-0.1, 0.15, 0.004)); zt = 0.5 * (ee[np.argmax(hh)] + ee[np.argmax(hh) + 1]); print(f"\nraw cloud table plane z = {zt*1000:.0f} mm; points in the workspace {len(P)}")
obj = P[P[:, 2] > zt + 0.04]; g = np.floor(obj / 0.02).astype(int); u, inv = np.unique(g, axis=0, return_inverse=True)
grid = np.zeros(u.max(0) - u.min(0) + 1, bool); grid[tuple((u - u.min(0)).T)] = True; l2, n2 = ndimage.label(ndimage.binary_closing(grid, iterations=2), structure=np.ones((3, 3, 3)))
RAW = []
print(f"RAW clusters above the table (> {zt+0.04:.2f}): ")
for k in range(1, n2 + 1):
    cells = np.argwhere(l2 == k) + u.min(0); pts = obj[np.isin(inv, [i for i in range(len(u)) if l2[tuple(u[i] - u.min(0))] == k])]
    if len(pts) < 300: continue
    RAW.append(pts); print(f"  raw {len(RAW)}: {len(pts):6d} pts, centre ({pts[:,0].mean():.2f},{pts[:,1].mean():.2f}), x {np.percentile(pts[:,0],1):.2f}..{np.percentile(pts[:,0],99):.2f} y {np.percentile(pts[:,1],1):.2f}..{np.percentile(pts[:,1],99):.2f} top z {np.percentile(pts[:,2],99):.2f}")
# how does the MPC map represent each raw object: ESDF value at the raw points (0 = on the surface, <0 inside), occupied-voxel coverage
def esdf_at(Q):
    i = np.floor((Q - O) / V).astype(int); ok = ((i >= 0) & (i < np.array(E.shape))).all(1); v = np.full(len(Q), np.nan); v[ok] = E[tuple(i[ok].T)]; return v
print("\nper raw object: ESDF at its raw surface points (m): share occupied(<=0) / median / p90 ; nearest map-occupied voxel centre distance median (cm)")
occT = cKDTree(C)
for k, pts in enumerate(RAW, 1):
    v = esdf_at(pts); known = v > -999; d, _ = occT.query(pts)
    print(f"  raw {k}: occupied {np.mean(v[known]<=0):.2f}, ESDF median {np.nanmedian(v[known]):+.3f}, p90 {np.nanpercentile(v[known],90):+.3f}, unobserved share {np.mean(~known):.2f}; nearest occupied voxel median {np.median(d)*100:.1f} cm, p90 {np.percentile(d,90)*100:.1f} cm")
np.savez("/s/objects_raw.npz", **{f"o{k}": p for k, p in enumerate(RAW, 1)})
