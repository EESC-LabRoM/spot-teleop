"""A SYNTHETIC esdf_full.npz for pipeline tests only (not a measured map): the table as a slab (ESDF = signed distance to the plane z = 0 of fr3_link0), free elsewhere, with the
REAL dumped grasp region (a map_check.py esdf.npz, e.g. runs/rs2/ep00/map_pre/esdf.npz) pasted in. Same format as esdf_capture.py.
   python3 make_test_esdf.py runs/rs2/ep00/map_pre/esdf.npz runs/newton_test_esdf.npz"""
import sys
import numpy as np
src, out = sys.argv[1], sys.argv[2]
Z = np.load(src); E, org, vs = Z["esdf"], Z["origin"].astype(np.float64), float(Z["voxel"])
mn, size = np.array([-0.75, -1.2, -0.45]), np.array([2.4, 2.4, 1.8])
lo = np.floor(mn / vs + 1e-6).astype(int); hi = np.floor((mn + size) / vs + 1e-6).astype(int); n = hi - lo + 1; O = lo * vs
zc = O[2] + (np.arange(n[2]) + 0.5) * vs
F = np.broadcast_to(zc[None, None, :].astype(np.float32), tuple(n)).copy()                       # signed distance to the table plane (z = 0): + above, - inside
F = np.where(F > 0, np.minimum(F, 1.0), np.maximum(F, -0.2)).astype(np.float32)
i0 = np.round((org - O) / vs).astype(int); sl = tuple(slice(i, i + s) for i, s in zip(i0, E.shape)); F[sl] = E
np.savez(out, esdf=F, origin=O, voxel=vs, frame="fr3_link0", req_min=mn, req_size=size, synthetic=True)
print("synthetic ESDF", F.shape, "origin", O.round(3).tolist(), "region pasted at", i0.tolist(), "->", out)
