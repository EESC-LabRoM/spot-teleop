"""GraspGen-X input for the HiveBoard valve: runs/valve_ggx/clouds/{lever,valve_body,panel}.npz (key 'pc', base frame fr3_link0).
NOT a rendered depth cloud: points are sampled on the lever / body / panel meshes (ground-truth geometry) and the lever's are kept only where the surface
faces a virtual camera on the robot side (n . (cam - p) > 0), a partial single-view cloud like a real segmentation cloud. Run in the graspgenx venv:
   ../graspgenx/.venv/bin/python -I valve_clouds.py runs/valve_ggx"""
import os, sys
import numpy as np, trimesh
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__))); import valve_geometry as vg
out = sys.argv[1]; os.makedirs(os.path.join(out, "clouds"), exist_ok=True); rng = np.random.default_rng(0)
CAM = np.array([-0.15, 0.10, 0.80])                         # virtual camera in the base frame: robot side, a bit above the lever
def sample(V, F, n, cam=None):
    m = trimesh.Trimesh(vertices=V, faces=F, process=False); P, fi = trimesh.sample.sample_surface(m, n * (3 if cam is not None else 1), seed=0)
    if cam is not None:
        N = m.face_normals[fi]; keep = np.einsum("ij,ij->i", N, cam - P) > 0; P = P[keep]
    return P[:n] if len(P) > n else P
Vl, Fl = vg.link_mesh("Mesh_1.obj", vg.LEVER_VIS); Vb, Fb = vg.link_mesh("Mesh.obj", vg.BODY_VIS); Vp, Fp = vg.panel_mesh()
sets = {"lever": sample(vg.to_base(Vl), Fl, 4000, CAM), "valve_body": sample(vg.to_base(Vb), Fb, 4000, CAM), "panel": sample(vg.to_base(Vp), Fp, 4000, CAM)}
for k, P in sets.items():
    np.savez_compressed(os.path.join(out, "clouds", f"{k}.npz"), pc=P.astype(np.float32))
    print(f"[valve_clouds] {k}: {len(P)} pts, bbox {P.min(0).round(3).tolist()} .. {P.max(0).round(3).tolist()}", flush=True)
