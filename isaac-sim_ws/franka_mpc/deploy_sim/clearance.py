"""Avoidance result of one run_deploy_sim_obstacle.sh run: distance between the arm's collision spheres (cuRobo's, buffer 0, posed by FK on the SIM arm's measured joints)
and the REAL objects' robot-free depth points (objects_raw.npz: o1 bottle, o2 drill, o3 box, o4 wall panel), not the map. Negative = the sphere surface is inside the object (contact).
  clearance.py <run dir> <objects_raw.npz> [table_z]"""
import sys, json, numpy as np, torch
from scipy.spatial import cKDTree
from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel, CudaRobotModelConfig
from curobo.types.base import TensorDeviceType
from curobo.util_file import load_yaml
run, objp = sys.argv[1], sys.argv[2]; table_z = float(sys.argv[3]) if len(sys.argv) > 3 else 0.042
Z = np.load(f"{run}/trace.npz", allow_pickle=True); t = np.asarray(Z["t"], float); t -= t[0]; q = np.asarray(Z["q"], float)[:, :7]; hand = np.asarray(Z["hand"], float); tgt = np.asarray(Z["target"], float)
R = np.load(objp); names = {"o1": "bottle", "o2": "drill", "o3": "box", "o4": "wall panel"}
kin = load_yaml("/workspace/franka_mpc/node_config/fr3_vcap1.2_acc10.yml")["robot_cfg"]["kinematics"]; kin["collision_sphere_buffer"] = 0.0
tdev = TensorDeviceType(); m = CudaRobotModel(CudaRobotModelConfig.from_data_dict(kin, tdev))
S = m.get_state(torch.tensor(q, device=tdev.device, dtype=tdev.dtype)).link_spheres_tensor.cpu().numpy()      # (n, S, 4): x y z r
S = S[:, S[0, :, 3] > 0]; C, r = S[..., :3], S[..., 3]
print(f"{len(t)} steps over {t[-1]:.1f} sim s, {S.shape[1]} spheres; hand target error: median {np.median(np.linalg.norm(hand-tgt,axis=1))*1000:.0f} mm, max {np.linalg.norm(hand-tgt,axis=1).max()*1000:.0f} mm")
res = {}
for k in ("o1", "o2", "o3", "o4"):
    tree = cKDTree(R[k]); d, _ = tree.query(C.reshape(-1, 3)); d = d.reshape(C.shape[:2]) - r          # surface-to-point clearance per step and sphere
    mn = d.min(1); i = int(mn.argmin()); res[names[k]] = float(mn.min())
    first = np.nonzero(mn < 0)[0]; print(f"  {names[k]:10s} min clearance {mn.min()*100:6.1f} cm at t={t[i]:5.1f} s (sphere {int(d[i].argmin())}); contact steps (<0): {len(first)}" + (f", first at t={t[first[0]]:.1f} s" if len(first) else ""))
tab = (C[..., 2] - r).min(1) - table_z; res["table"] = float(tab.min()); print(f"  {'table':10s} min clearance {tab.min()*100:6.1f} cm at t={t[int(tab.argmin())]:5.1f} s")
print("  hand path (flange) at sim seconds:  " + "  ".join(f"t={s:4.1f}:({hand[np.argmin(abs(t-s))][0]:.2f},{hand[np.argmin(abs(t-s))][1]:.2f},{hand[np.argmin(abs(t-s))][2]:.2f})/goal({tgt[np.argmin(abs(t-s))][0]:.2f},{tgt[np.argmin(abs(t-s))][1]:.2f},{tgt[np.argmin(abs(t-s))][2]:.2f})" for s in (8, 14, 16, 22, 25, 34)))
try: json.dump(res, open(f"{run}/clearance.json", "w"), indent=1)
except OSError: pass
