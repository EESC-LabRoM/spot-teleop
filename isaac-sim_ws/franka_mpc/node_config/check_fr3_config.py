"""Check node_config/fr3.yml against the Panda config it was made from (make_fr3_config.py):
FK of the hand and of every collision sphere at random joint angles (same kinematics -> should match), the joint limits
cuRobo loaded vs franka_description's fr3 joint_limits.yaml, retract inside the limits, and how much of each FR3 / Panda
collision mesh the (shared) spheres cover.
   curobo-isaac6/.venv/bin/python -I node_config/check_fr3_config.py
"""
import os
import numpy as np, torch, trimesh, yaml
from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel, CudaRobotModelConfig
from curobo.types.base import TensorDeviceType
from curobo.util_file import get_assets_path, get_robot_configs_path, join_path, load_yaml

HERE = os.path.dirname(os.path.abspath(__file__))
FD = os.path.normpath(os.path.join(HERE, "..", "..", "franka_description"))
ta = TensorDeviceType()
def model(p):
    c = load_yaml(p)["robot_cfg"]; c["kinematics"]["collision_sphere_buffer"] = 0.0
    return CudaRobotModel(CudaRobotModelConfig.from_data_dict(c["kinematics"], tensor_args=ta)), c
pm, pc = model(os.path.join(HERE, "franka.yml")); fm, fc = model(os.path.join(HERE, "fr3.yml"))
lo_p, hi_p = pm.get_joint_limits().position.cpu().numpy(); lo_f, hi_f = fm.get_joint_limits().position.cpu().numpy()
print("joint names:", fm.joint_names)
ref = np.array([[-2.7437, 2.7437], [-1.7837, 1.7837], [-2.9007, 2.9007], [-3.0421, -0.1518], [-2.8065, 2.8065], [0.5445, 4.5169], [-3.0159, 3.0159]])
print("FR3 limits loaded vs the FCI's FR3 q_min/q_max, max |diff| rad:", np.abs(np.stack([lo_f[:7], hi_f[:7]], 1) - ref).max().round(5))
print("velocity limits loaded (rad/s):", fm.get_joint_limits().velocity[1].cpu().numpy()[:7].round(2))
r = np.array(fc["kinematics"]["cspace"]["retract_config"][:7]); print("retract inside FR3 limits:", bool(np.all((r > lo_f[:7]) & (r < hi_f[:7]))), r.tolist())
rng = np.random.default_rng(0); lo, hi = np.maximum(lo_p, lo_f), np.minimum(hi_p, hi_f)
q = torch.tensor(lo + rng.random((256, len(lo))) * (hi - lo), device="cuda", dtype=torch.float32)
sp, sf = pm.get_state(q), fm.get_state(q)
dp = (sp.ee_position - sf.ee_position).norm(dim=-1).max().item(); dq = (sp.ee_quaternion - sf.ee_quaternion).abs().max().item()
ds = (sp.link_spheres_tensor[..., :3] - sf.link_spheres_tensor[..., :3]).norm(dim=-1).max().item()
print(f"FK Panda cfg vs FR3 cfg at 256 random q: hand {dp * 1e3:.4f} mm, quat {dq:.2e}, spheres {ds * 1e3:.4f} mm")
# coverage: share of each link's collision mesh surface inside its spheres (+1 cm), FR3 meshes vs Panda meshes
S = load_yaml(join_path(get_robot_configs_path(), "spheres/fr3_mesh.yml"))["collision_spheres"]
def cover(mesh_path, sph, pad=0.01):
    m = trimesh.load(mesh_path, force="mesh"); np.random.seed(0); P = m.sample(4000)
    C = np.array([s["center"] for s in sph]); R = np.array([s["radius"] for s in sph])
    d = np.linalg.norm(P[:, None] - C[None], axis=-1) - R[None]; return float((d.min(1) <= pad).mean())
fr3m = os.path.join(get_assets_path(), "robot", "fr3_description", "meshes", "robots", "fr3", "collision")
pm_ = os.path.join(get_assets_path(), "robot", "franka_description", "meshes", "collision")
print("link   spheres  covered(+1cm): FR3 mesh | Panda mesh")
for i in range(1, 8):
    k = f"fr3_link{i}"; f = os.path.join(fr3m, f"link{i}.stl"); p = next((os.path.join(pm_, x) for x in (f"link{i}.obj", f"link{i}.stl") if os.path.exists(os.path.join(pm_, x))), None)
    print(f"{k:10s} {len(S.get(k, [])):3d}   {cover(f, S[k]):.3f} | {cover(p, S[k]) if p else float('nan'):.3f}")
