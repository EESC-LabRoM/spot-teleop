"""Export per-step link poses (pos+quat) for the visual links, subsampled, for the web viewer."""
import sys
import numpy as np
import torch
from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel, CudaRobotModelConfig
from curobo.types.base import TensorDeviceType
from curobo.types.state import JointState
from curobo.util_file import get_robot_configs_path, join_path, load_yaml

run = sys.argv[1]
T = np.load(f"{run}/trace.npz")
cfg = load_yaml(join_path(get_robot_configs_path(), "fr3.yml"))["robot_cfg"]
cfg["kinematics"]["collision_sphere_buffer"] = 0.0
cfg["kinematics"]["link_names"] = ["fr3_link0","fr3_link1","fr3_link2","fr3_link3","fr3_link4","fr3_link5","fr3_link6","fr3_link7","fr3_hand","fr3_leftfinger","fr3_rightfinger"]
kin = CudaRobotModel(CudaRobotModelConfig.from_data_dict(cfg["kinematics"], tensor_args=TensorDeviceType()))
q = np.concatenate([T["q"], np.full((len(T["q"]), 2), 0.04)], 1)
step = 5
qq = torch.tensor(q[::step], device="cuda", dtype=torch.float32)
state = JointState(position=qq, joint_names=kin.joint_names)
out = kin.compute_kinematics_from_joint_state(state)
names = list(out.link_names)
print("links:", names)
want = ["fr3_link0","fr3_link1","fr3_link2","fr3_link3","fr3_link4","fr3_link5","fr3_link6","fr3_link7","fr3_hand","fr3_leftfinger","fr3_rightfinger"]
idx = []
for w in want:
    hit = [i for i, n in enumerate(names) if n == w or n.endswith("_" + w) or n.endswith(w)]
    idx.append(hit[0] if hit else -1)
print("map:", list(zip(want, idx)))
P = torch.cat([out.links_position, out.links_quaternion], dim=-1).cpu().numpy()
keep = [i for i in idx if i >= 0]
np.savez(f"{run}/poses.npz", step=np.int64(step), t=T["t"][::step],
         links=np.array([want[i] for i, k in enumerate(idx) if k >= 0]),
         poses=P[:, keep, :])
print("wrote", f"{run}/poses.npz", P[:, keep, :].shape)
