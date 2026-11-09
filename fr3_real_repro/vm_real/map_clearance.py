"""Clearance of a simulated/real joint path against the MAP the planner saw, and floaters on the path.
  map_clearance.py <esdf.npz> <trace.npz (sim, key q) | csv with q0..q6> <objects_raw.npz>
Prints (a) min sphere-surface clearance to the ESDF (buffer 0) and how many samples overlap an occupied voxel, (b) occupied voxels above the table that are NOT near
any real object cloud (> 8 cm) but within 15 cm of the arm's spheres somewhere on the path = floaters sitting on the route."""
import sys, csv, numpy as np, torch
from scipy.ndimage import map_coordinates
from scipy.spatial import cKDTree
from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel, CudaRobotModelConfig
from curobo.types.base import TensorDeviceType
from curobo.util_file import load_yaml
ep, tp, op = sys.argv[1:4]
if tp.endswith(".npz"): q = np.asarray(np.load(tp, allow_pickle=True)["q"], float)[:, :7]
else: rows = list(csv.DictReader(open(tp))); q = np.array([[float(r[f"q{i}"]) for i in range(7)] for r in rows])
q = q[::max(1, len(q) // 3000)]
Z = np.load(ep); E = Z["esdf"].astype(float); o = Z["origin"]; v = float(Z["voxel"]); R = np.load(op)
kin = load_yaml("/workspace/franka_mpc/node_config/fr3_vcap1.2_acc10.yml")["robot_cfg"]["kinematics"]; kin["collision_sphere_buffer"] = 0.0
tdev = TensorDeviceType(); m = CudaRobotModel(CudaRobotModelConfig.from_data_dict(kin, tdev))
S = m.get_state(torch.tensor(q, device=tdev.device, dtype=tdev.dtype)).link_spheres_tensor.cpu().numpy(); S = S[:, S[0, :, 3] > 0]; C, r = S[..., :3], S[..., 3]
unk = E <= -999; idx = (C.reshape(-1, 3) - o) / v - 0.5
d = map_coordinates(np.where(unk, 1.0, E), idx.T, order=1, mode="nearest").reshape(C.shape[:2]) - r
inb = np.all((idx >= 0) & (idx <= np.array(E.shape) - 1), axis=1).reshape(C.shape[:2]); mn = np.where(inb, d, 9).min(1)
print(f"MAP clearance on the path: min {mn.min()*100:.1f} cm; samples overlapping an occupied voxel: {(mn < 0).sum()} of {len(mn)}")
occ = np.argwhere((E > -999) & (E <= 0)); P = o + (occ + 0.5) * v
allc = np.vstack([R[k] for k in R.files]); dobj, _ = cKDTree(allc).query(P)
cand = (P[:, 2] > 0.10) & (dobj > 0.08)
near = cKDTree(C.reshape(-1, 3)).query(P[cand])[0] - r.max() < 0.15
F = P[cand][near]
print(f"floaters on the route (occupied, above table, >8 cm from any real object, within 15 cm of the arm's path): {len(F)}" + ("" if not len(F) else "  " + "; ".join(str(np.round(f, 3).tolist()) for f in F[:8])))
print(f"all occupied above the table and >8 cm from any real object (anywhere in the box): {int(cand.sum())}")
