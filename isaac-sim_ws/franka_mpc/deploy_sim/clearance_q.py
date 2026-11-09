"""Clearance of a joint-space path (CSV with columns t,q0..q6; preview.csv of vm_goal_path or the contact guard's log) against the real object clouds; same sphere model as clearance.py.
  clearance_q.py <csv> <objects_raw.npz> [table_z] [label]"""
import sys, csv, numpy as np, torch
from scipy.spatial import cKDTree
from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel, CudaRobotModelConfig
from curobo.types.base import TensorDeviceType
from curobo.util_file import load_yaml
p, objp = sys.argv[1], sys.argv[2]; table_z = float(sys.argv[3]) if len(sys.argv) > 3 else 0.042; label = sys.argv[4] if len(sys.argv) > 4 else p
rows = list(csv.DictReader(open(p))); t = np.array([float(r["t"]) for r in rows]); q = np.array([[float(r[f"q{i}"]) for i in range(7)] for r in rows]); t -= t[0]
R = np.load(objp); names = {"o1": "bottle", "o2": "drill", "o3": "box", "o4": "wall panel"}
kin = load_yaml("/workspace/franka_mpc/node_config/fr3_vcap1.2_acc10.yml")["robot_cfg"]["kinematics"]; kin["collision_sphere_buffer"] = 0.0
tdev = TensorDeviceType(); m = CudaRobotModel(CudaRobotModelConfig.from_data_dict(kin, tdev))
st = m.get_state(torch.tensor(q, device=tdev.device, dtype=tdev.dtype)); S = st.link_spheres_tensor.cpu().numpy(); S = S[:, S[0, :, 3] > 0]; C, r = S[..., :3], S[..., 3]; hand = st.ee_position.cpu().numpy()
print(f"== {label}: {len(t)} samples over {t[-1]:.1f} s, {S.shape[1]} spheres; hand x {hand[:,0].min():.2f}..{hand[:,0].max():.2f} y {hand[:,1].min():.2f}..{hand[:,1].max():.2f} z {hand[:,2].min():.2f}..{hand[:,2].max():.2f}")
for k in ("o1", "o2", "o3", "o4"):
    d, _ = cKDTree(R[k]).query(C.reshape(-1, 3)); d = (d.reshape(C.shape[:2]) - r).min(1); i = int(d.argmin()); n = int((d < 0).sum())
    print(f"  {names[k]:10s} min clearance {d.min()*100:6.1f} cm at t={t[i]:5.1f} s; contact samples {n}")
tab = (C[..., 2] - r).min(1) - table_z; print(f"  {'table':10s} min clearance {tab.min()*100:6.1f} cm")
