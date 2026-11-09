"""Hand pose (fr3_hand in fr3_link0) at the lab ready pose, from cuRobo's own kinematics (the frame the node's /wrist_pose goal uses)."""
import math, sys, torch
from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel, CudaRobotModelConfig
from curobo.types.base import TensorDeviceType
from curobo.util_file import load_yaml
cfg = sys.argv[1] if len(sys.argv) > 1 else "/workspace/franka_mpc/node_config/fr3_vcap1.2_acc10.yml"
tdev = TensorDeviceType(); kin = load_yaml(cfg)["robot_cfg"]["kinematics"]
m = CudaRobotModel(CudaRobotModelConfig.from_data_dict(kin, tdev))
q = torch.tensor([[0.0, -math.pi / 4, 0.0, -3 * math.pi / 4, 0.0, math.pi / 2, math.pi / 4, 0.04, 0.04]], device=tdev.device, dtype=tdev.dtype)
s = m.get_state(q[:, :len(m.joint_names)])
p = s.ee_position[0].tolist(); w = s.ee_quaternion[0].tolist()          # cuRobo quaternion order is w, x, y, z
print("JOINTS", m.joint_names)
print("P0 %.5f %.5f %.5f" % tuple(p)); print("Q0_XYZW %.5f %.5f %.5f %.5f" % (w[1], w[2], w[3], w[0]))
