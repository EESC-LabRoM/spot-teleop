"""fr3_hand pose (flange, fr3_link0) for given arm joints, from cuRobo's own FK (the frame vm_goal_path.py and the node use). usage: fk_q.py q1,...,q7"""
import sys, torch
from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel, CudaRobotModelConfig
from curobo.types.base import TensorDeviceType
from curobo.util_file import load_yaml
q = [float(x) for x in sys.argv[1].split(",")]; tdev = TensorDeviceType(); kin = load_yaml("/workspace/franka_mpc/node_config/fr3_vcap1.2_acc10.yml")["robot_cfg"]["kinematics"]
m = CudaRobotModel(CudaRobotModelConfig.from_data_dict(kin, tdev)); s = m.get_state(torch.tensor([q], device=tdev.device, dtype=tdev.dtype))
p = s.ee_position[0].tolist(); w = s.ee_quaternion[0].tolist()
print("P0 %.5f,%.5f,%.5f" % tuple(p)); print("QUAT_XYZW %.5f,%.5f,%.5f,%.5f" % (w[1], w[2], w[3], w[0]))
