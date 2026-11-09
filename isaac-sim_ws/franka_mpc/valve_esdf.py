"""A SYNTHETIC esdf_full.npz for the HiveBoard valve (pipeline test, NOT a measured/nvblox map): the table as a slab (signed distance to z = 0 of fr3_link0) united with the
valve body, the lever (at theta = 0) and the honeycomb panel from their meshes (valve_geometry.py), voxelized at 5 mm, signed distance by EDT, sampled on the nvblox lattice
(voxel centres at (i + 0.5) * voxel). Same format as esdf_capture.py / make_test_esdf.py. The lever is NOT in the map by default (--with-lever keeps it: then the MPC stalls ~6 cm short, the clear box does not remove the lever's distance field outside its box, runs/valve_mpc1).
   ../graspgenx/.venv/bin/python -I valve_esdf.py runs/valve_esdf.npz [--voxel 0.02]"""
import argparse, os, sys
import numpy as np, trimesh
from scipy.ndimage import distance_transform_edt
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__))); import valve_geometry as vg
ap = argparse.ArgumentParser(); ap.add_argument("out"); ap.add_argument("--voxel", type=float, default=0.02); ap.add_argument("--pitch", type=float, default=0.005)
ap.add_argument("--with-lever", action="store_true", help="keep the lever in the map (default: left out, as nvblox with the target mask never integrates the target; a clear box only blanks the target's own voxels, the distance field around it would still be the lever's)")
a = ap.parse_args()
vs = a.voxel
parts = []
for name, (V, F) in {**({"lever": vg.link_mesh("Mesh_1.obj", vg.LEVER_VIS)} if a.with_lever else {}), "body": vg.link_mesh("Mesh.obj", vg.BODY_VIS), "panel": vg.panel_mesh()}.items():
    parts.append(trimesh.Trimesh(vertices=vg.to_base(V), faces=F, process=False))
lo_o = np.min([p.bounds[0] for p in parts], 0) - 0.30; hi_o = np.max([p.bounds[1] for p in parts], 0) + 0.30
shape = np.ceil((hi_o - lo_o) / a.pitch).astype(int) + 1; occ = np.zeros(shape, bool)
for p in parts:
    vox = p.voxelized(a.pitch); pts = vox.points; idx = np.round((pts - lo_o) / a.pitch).astype(int)
    ok = np.all((idx >= 0) & (idx < shape), 1); occ[tuple(idx[ok].T)] = True
d_out = distance_transform_edt(~occ) * a.pitch; d_in = distance_transform_edt(occ) * a.pitch
sdf = np.where(occ, -d_in, d_out).astype(np.float32)                                    # + free, - inside, ~distance to the surface (a half pitch off)
mn, size = np.array([-0.75, -1.2, -0.45]), np.array([2.4, 2.4, 1.8])
lo = np.floor(mn / vs + 1e-6).astype(int); hi = np.floor((mn + size) / vs + 1e-6).astype(int); n = hi - lo + 1; O = lo * vs
zc = O[2] + (np.arange(n[2]) + 0.5) * vs
F = np.broadcast_to(zc[None, None, :].astype(np.float32), tuple(n)).copy()
F = np.where(F > 0, np.minimum(F, 1.0), np.maximum(F, -0.2)).astype(np.float32)           # table slab
gx = [O[i] + (np.arange(n[i]) + 0.5) * vs for i in range(3)]
G = np.stack(np.meshgrid(*gx, indexing="ij"), -1)                                          # lattice centres
inside = np.all((G >= lo_o) & (G <= lo_o + (shape - 1) * a.pitch), -1)
fi = np.clip(np.round((G[inside] - lo_o) / a.pitch).astype(int), 0, shape - 1)
vals = sdf[tuple(fi.T)]; Fo = F[inside]; F[inside] = np.minimum(Fo, np.where(vals < 0, vals, np.minimum(vals, 1.0)))
np.savez(a.out, esdf=F, origin=O, voxel=vs, frame="fr3_link0", req_min=mn, req_size=size, synthetic=True)
occ_n = int((F[inside] < 0.0).sum()); print(f"synthetic valve ESDF {F.shape} voxel {vs} origin {O.round(3).tolist()}; {occ_n} object voxels (<0), {int((F < 0.02).sum())} voxels < 2 cm -> {a.out}")
