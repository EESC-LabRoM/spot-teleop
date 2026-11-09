"""Did the hand/fingers touch the valve housing (valvula_esfera base) or the honeycomb panel? Per control step of a trace.npz: exact distance from (a) the Franka hand model
(palm + 2 finger boxes of grasp_filter.py at the logged finger opening, margin 0) and (b) the cuRobo collision spheres (conservative envelope) to the true meshes.
Distance <= 0 = interpenetration, small positive = within the box model's tolerance. Video time = trace t - t[0].
   ../curobo-isaac6/.venv/bin/python -I body_contact.py runs/valve_isaac_video --ep 0 1"""
import argparse, os, sys
import numpy as np, torch, trimesh
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as R
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); import valve_geometry as vg
from curobo.types.base import TensorDeviceType
from curobo.types.robot import RobotConfig
from curobo.util_file import load_yaml
from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel
ap = argparse.ArgumentParser(); ap.add_argument("run"); ap.add_argument("--ep", type=int, nargs="+", default=[0, 1]); a = ap.parse_args()
ta = TensorDeviceType(); rc = RobotConfig.from_dict(load_yaml(os.path.join(HERE, "node_config", "fr3.yml"))["robot_cfg"]); kin = CudaRobotModel(rc.kinematics)
def surf(V, F, n): return trimesh.sample.sample_surface(trimesh.Trimesh(vertices=V, faces=F, process=False), n, seed=0)[0]
Vb, Fb = vg.link_mesh("Mesh.obj", vg.BODY_VIS); Vp, Fp = vg.panel_mesh()
BODY = surf(vg.to_base(Vb), Fb, 60000); PANEL = surf(vg.to_base(Vp), Fp, 120000); tb, tp = cKDTree(BODY), cKDTree(PANEL)
def box_sd(pl, c, h): q = np.abs(pl - np.array(c)) - np.array(h); return np.linalg.norm(np.maximum(q, 0), axis=1) + np.minimum(q.max(1), 0)
def hand_dist(P, Rm, t, qf):
    pl = (P - t) @ Rm; out = []
    for c, h in ((( 0, 0, 0.0292), (0.035, 0.100, 0.0292)), ((0, qf[0] + 0.010, 0.0809), (0.0105, 0.010, 0.0225)), ((0, -(qf[1] + 0.010), 0.0809), (0.0105, 0.010, 0.0225))):
        out.append(box_sd(pl, c, h).min())
    return min(out), int(np.argmin(out))
for ep in a.ep:
    T = np.load(os.path.join(a.run, f"ep{ep:02d}", "trace.npz")); t = T["t"] - T["t"][0]; n = len(t); res = []
    Q = torch.tensor(T["q"].astype(np.float32), device=ta.device); SP = kin.get_state(Q).link_spheres_tensor.cpu().numpy()
    for i in range(0, n, 2):
        Rm = R.from_quat(T["hand_quat"][i]).as_matrix(); h = T["hand"][i]; qf = T["q_fingers"][i]
        db, kb = hand_dist(BODY, Rm, h, qf); dp, kp = hand_dist(PANEL, Rm, h, qf); S = SP[i][SP[i][:, 3] > 0]
        sb = (tb.query(S[:, :3])[0] - S[:, 3]).min(); sp = (tp.query(S[:, :3])[0] - S[:, 3]).min()
        res.append((t[i], db, dp, sb, sp, kb))
    r = np.array(res); ib = r[:, 1].argmin(); ip = r[:, 2].argmin(); isb = r[:, 3].argmin()
    pn = ["palm", "left finger", "right finger"]
    print(f"ep{ep:02d}: hand model -> valve body: min {r[ib,1]*1e3:+.1f} mm at video t={r[ib,0]:.2f} s ({pn[int(r[ib,5])]}); -> panel: min {r[ip,2]*1e3:+.1f} mm at t={r[ip,0]:.2f} s | "
          f"collision spheres -> valve body: min {r[isb,3]*1e3:+.1f} mm at t={r[isb,0]:.2f} s; -> panel min {r[:,4].min()*1e3:+.1f} mm | steps with hand-model distance < 1 mm to body: {int((r[:,1] < 0.001).sum())}")
