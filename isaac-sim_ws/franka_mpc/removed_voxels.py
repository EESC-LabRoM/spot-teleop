"""Which voxels of the MPC grid disappear during the approach, which scene part they belong to (valve body, honeycomb panel, post, lever, other) and how far they were from
the robot's collision spheres when they went (the self-filter removes depth within threshold 0.15 m of the sphere surfaces).  Uses runs/<run>/epNN/esdf_dump (node_dump).
   ../curobo-isaac6/.venv/bin/python -I removed_voxels.py runs/<run> --ep 0 1 2"""
import argparse, glob, json, os, sys
import numpy as np, torch, trimesh
from scipy.spatial import cKDTree
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); import valve_geometry as vg
from curobo.types.base import TensorDeviceType
from curobo.types.robot import RobotConfig
from curobo.util_file import load_yaml
from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel
ap = argparse.ArgumentParser(); ap.add_argument("run"); ap.add_argument("--ep", type=int, nargs="+", default=[0]); ap.add_argument("--tol", type=float, default=0.04); a = ap.parse_args()
ta = TensorDeviceType(); rc = RobotConfig.from_dict(load_yaml(os.path.join(HERE, "node_config", "fr3.yml"))["robot_cfg"]); kin = CudaRobotModel(rc.kinematics)
def surf(V, F, n): return trimesh.sample.sample_surface(trimesh.Trimesh(vertices=V, faces=F, process=False), n, seed=0)[0]
Vb, Fb = vg.link_mesh("Mesh.obj", vg.BODY_VIS); Vp, Fp = vg.panel_mesh(); Vl, Fl = vg.link_mesh("Mesh_1.obj", vg.LEVER_VIS)
parts = {"valve body": cKDTree(surf(vg.to_base(Vb), Fb, 40000)), "honeycomb panel": cKDTree(surf(vg.to_base(Vp), Fp, 80000)), "lever (target)": cKDTree(surf(vg.to_base(Vl), Fl, 20000)),
         "post": cKDTree(np.array([[0.65 + dx, 0.06 + dy, z] for dx in np.linspace(-0.02, 0.02, 5) for dy in np.linspace(-0.02, 0.02, 5) for z in np.arange(0.0, 0.4165, 0.01)]))}
def label(P):
    d = np.stack([t.query(P)[0] for t in parts.values()], 1); k = d.argmin(1); lab = np.array(list(parts))[k]; lab[d.min(1) > a.tol] = "unexplained"; return lab
for ep in a.ep:
    fs = sorted(glob.glob(os.path.join(a.run, f"ep{ep:02d}", "esdf_dump", "e_*.npz"))); prev = None; print(f"== ep{ep:02d}: {len(fs)} grids")
    for f in fs:
        Z = np.load(f); E, org, vs = Z["esdf"], Z["origin"], float(Z["voxel"]); names = list(Z["names"])
        if "fr3_joint1" not in names: continue
        q = np.array([Z["q"][names.index(f"fr3_joint{j}")] for j in range(1, 8)]); occ = (E > -999) & (E <= 0)
        if prev is not None:
            gone = np.argwhere(prev[0] & ~occ); new = np.argwhere(~prev[0] & occ)
            if len(gone) + len(new):
                S = kin.get_state(torch.tensor(q[None].astype(np.float32), device=ta.device)).link_spheres_tensor.cpu().numpy()[0]; act = S[:, 3] > 0; C, Rr = S[act, :3].astype(np.float64), S[act, 3].astype(np.float64)
                def info(ix):
                    P = org + (ix + 0.5) * vs; dist = (np.linalg.norm(P[:, None] - C[None], axis=2) - Rr[None]).min(1); lab = label(P); out = {}
                    for l in sorted(set(lab)): m = lab == l; out[l] = f"{int(m.sum())} vox, dist to robot {dist[m].min()*1e3:.0f}-{dist[m].max()*1e3:.0f} mm"
                    return out
                print(f"  t={float(Z['t_sim']) - float(np.load(fs[0])['t_sim']):5.1f}s  REMOVED {len(gone)}: {info(gone) if len(gone) else '-'}   ADDED {len(new)}: {info(new) if len(new) else '-'}")
        prev = (occ,)
