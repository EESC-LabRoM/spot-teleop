"""Video of the MPC's OWN voxel grid (node_dump esdf_dump) during one episode: occupied voxels coloured by the true scene part they belong to
(valve body / honeycomb panel / post / lever / unexplained) + the arm's collision spheres at the joint state of each grid. Side view (x forward, z up) and top view.
   ../curobo-isaac6/.venv/bin/python -I map_video.py runs/<run> --ep 1 --out out.mp4"""
import argparse, glob, os, subprocess, sys, tempfile
import numpy as np, torch, trimesh
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); import valve_geometry as vg
from curobo.types.base import TensorDeviceType
from curobo.types.robot import RobotConfig
from curobo.util_file import load_yaml
from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel
ap = argparse.ArgumentParser(); ap.add_argument("run"); ap.add_argument("--ep", type=int, default=0); ap.add_argument("--out", required=True); ap.add_argument("--tol", type=float, default=0.04); a = ap.parse_args()
ta = TensorDeviceType(); rc = RobotConfig.from_dict(load_yaml(os.path.join(HERE, "node_config", "fr3.yml"))["robot_cfg"]); kin = CudaRobotModel(rc.kinematics)
def surf(V, F, n): return trimesh.sample.sample_surface(trimesh.Trimesh(vertices=V, faces=F, process=False), n, seed=0)[0]
Vb, Fb = vg.link_mesh("Mesh.obj", vg.BODY_VIS); Vp, Fp = vg.panel_mesh(); Vl, Fl = vg.link_mesh("Mesh_1.obj", vg.LEVER_VIS)
PARTS = {"valve body": (cKDTree(surf(vg.to_base(Vb), Fb, 40000)), "#2ca02c"), "honeycomb panel": (cKDTree(surf(vg.to_base(Vp), Fp, 80000)), "#1f77b4"),
         "lever": (cKDTree(surf(vg.to_base(Vl), Fl, 20000)), "#ff7f0e"), "post": (cKDTree(np.array([[0.65 + dx, 0.06 + dy, z] for dx in np.linspace(-0.02, 0.02, 5) for dy in np.linspace(-0.02, 0.02, 5) for z in np.arange(0.0, 0.4165, 0.01)])), "#8c564b")}
fs = sorted(glob.glob(os.path.join(a.run, f"ep{a.ep:02d}", "esdf_dump", "e_*.npz"))); t0 = float(np.load(fs[0])["t_sim"]); tmp = tempfile.mkdtemp(dir="/tmp/claude-1000/-home-nexus/fdbb2ee7-f147-4e1d-845e-ddf76aeb2c0f/scratchpad"); n = 0
for f in fs:
    Z = np.load(f); names = list(Z["names"])
    if "fr3_joint1" not in names: continue
    E, org, vs = Z["esdf"], Z["origin"], float(Z["voxel"]); q = np.array([Z["q"][names.index(f"fr3_joint{j}")] for j in range(1, 8)])
    S = kin.get_state(torch.tensor(q[None].astype(np.float32), device=ta.device)).link_spheres_tensor.cpu().numpy()[0]; S = S[S[:, 3] > 0]
    idx = np.argwhere((E > -999) & (E <= 0)); P = org + (idx + 0.5) * vs
    d = np.stack([p[0].query(P)[0] for p in PARTS.values()], 1) if len(P) else np.zeros((0, 4)); k = d.argmin(1) if len(P) else np.zeros(0, int); ok = (d.min(1) <= a.tol) if len(P) else np.zeros(0, bool)
    fig, ax = plt.subplots(1, 2, figsize=(11, 4.6))
    for j, (u, v, nm) in enumerate(((0, 2, "side view: x forward / z up"), (0, 1, "top view: x forward / y"))):
        for i, (name, (_, col)) in enumerate(PARTS.items()):
            m = ok & (k == i); ax[j].scatter(P[m, u], P[m, v], s=26, c=col, marker="s", label=f"{name} ({int(m.sum())})" if j == 0 else None)
        m = ~ok; ax[j].scatter(P[m, u], P[m, v], s=26, c="#bbbbbb", marker="s", label=f"other ({int(m.sum())})" if j == 0 else None)
        for c in S: ax[j].add_patch(plt.Circle((c[u], c[v]), c[3], fill=False, ec="k", lw=0.6, alpha=0.7))
        ax[j].set_xlim(0.0, 0.85); ax[j].set_ylim((0.1, 0.75) if v == 2 else (-0.45, 0.2)); ax[j].set_aspect("equal"); ax[j].grid(alpha=.3); ax[j].set_title(nm, fontsize=9)
    ax[0].legend(fontsize=7, loc="upper left"); fig.suptitle(f"ep{a.ep:02d}  MPC grid at sim t = {float(Z['t_sim']) - t0:5.1f} s  (black circles = the arm's collision spheres; squares = occupied 2 cm voxels)", fontsize=10)
    fig.tight_layout(); fig.savefig(os.path.join(tmp, f"{n:04d}.png"), dpi=90); plt.close(fig); n += 1
subprocess.run(["ffmpeg", "-loglevel", "error", "-y", "-framerate", "3", "-i", os.path.join(tmp, "%04d.png"), "-vf", "scale=trunc(iw/2)*2:trunc(ih/2)*2", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "22", a.out], check=True)
print(n, "frames ->", a.out)
