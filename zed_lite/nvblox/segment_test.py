"""Offline test of cuRobo's RobotSegmenter on a real frame from the pipeline: the ZED stereo depth recorded by
zed_lite/check_topics.py (Franka held at retract), the camera pose from zed_extrinsics_tf.py (base -> left optical) and
cuRobo's fr3.yml spheres (the FR3; same as franka.yml's). Also checks that Isaac 6.0's bundled ROS 2 Humble rclpy (Python 3.12) imports next to our
existing cuRobo build. Runs with Isaac's python in the Isaac image (no Kit app):
    see the docker command in zed_lite/README.md (segmenter test)
"""
import json, os, sys, time
import numpy as np
import torch

out = sys.argv[1]
import rclpy                                                   # Isaac's bundled Humble rclpy (py3.12)
rclpy.init(); n = rclpy.create_node("segmenter_import_test"); print("[seg] rclpy", rclpy.__file__.split("/exts/")[-1], "node", n.get_name())
from sensor_msgs.msg import Image, CameraInfo, JointState as JS   # noqa: F401 (just proving the message types import)
from curobo.types.base import TensorDeviceType
from curobo.types.camera import CameraObservation
from curobo.types.math import Pose
from curobo.types.state import JointState
from curobo.wrap.model.robot_segmenter import RobotSegmenter

D = "/zl"
depth_m = np.load(f"{D}/check/depth_m.npy"); info = json.load(open(f"{D}/check/topics.json"))["info"]
K = np.array(info["K"], dtype=np.float32).reshape(3, 3)
T = np.array(json.load(open(f"{D}/nvblox/out/extrinsics.json"))["base_T_left_optical"])   # pose of the camera in the base
def mat2quat(R):                                                # 3x3 rotation -> (w, x, y, z)
    R = np.asarray(R, float); t = np.trace(R)
    if t > 0:
        s = 2 * np.sqrt(t + 1); return np.array([s / 4, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s])
    i = int(np.argmax(np.diag(R))); j, k = (i + 1) % 3, (i + 2) % 3
    s = 2 * np.sqrt(1 + R[i, i] - R[j, j] - R[k, k]); q = np.zeros(4); q[0] = (R[k, j] - R[j, k]) / s
    q[1 + i] = s / 4; q[1 + j] = (R[j, i] + R[i, j]) / s; q[1 + k] = (R[k, i] + R[i, k]) / s; return q


q = mat2quat(T[:3, :3]).tolist()                                # (w, x, y, z)
ta = TensorDeviceType()
res = {"depth_size": list(depth_m.shape), "K_fx": float(K[0, 0])}
for thr in (0.02, 0.05, 0.10, 0.20):
    seg = RobotSegmenter.from_robot_file("fr3.yml", collision_sphere_buffer=0.0, distance_threshold=thr, use_cuda_graph=False)
    depth_mm = ta.to_device(np.nan_to_num(depth_m, nan=0, posinf=0, neginf=0).astype(np.float32) * 1000.0).unsqueeze(0)
    cam = CameraObservation(depth_image=depth_mm, intrinsics=ta.to_device(K).unsqueeze(0),
                            pose=Pose(position=ta.to_device(T[:3, 3].tolist()).unsqueeze(0), quaternion=ta.to_device(q).unsqueeze(0)))
    seg.update_camera_projection(cam)
    js = JointState.from_position(ta.to_device([[0.0, -1.3, 0.0, -2.5, 0.0, 1.0, 0.0]]),          # retract, as held in the sim
                                  joint_names=[f"fr3_joint{i}" for i in range(1, 8)])
    torch.cuda.synchronize(); t0 = time.time()
    mask, filtered = seg.get_robot_mask(cam, js)
    torch.cuda.synchronize(); ms = (time.time() - t0) * 1000
    m = mask[0].cpu().numpy()
    res[f"thr_{thr}"] = {"masked_px": int(m.sum()), "masked_frac": round(float(m.mean()), 4), "first_call_ms": round(ms, 1)}
    np.save(os.path.join(out, f"robot_mask_thr{int(thr * 100):02d}.npy"), m)
    if thr == 0.05:
        np.save(os.path.join(out, "world_depth_thr05_m.npy"), filtered[0].cpu().numpy() / 1000.0)
        sph = seg.robot_world.get_kinematics(js.position).link_spheres_tensor.view(-1, 4).cpu().numpy()
        res["spheres"] = int((sph[:, 3] > 0).sum())
        np.save(os.path.join(out, "spheres_retract.npy"), sph)
print("[seg] RESULT " + json.dumps(res)); json.dump(res, open(os.path.join(out, "segment_test.json"), "w"), indent=1)
os.system(f"chown -R 1000:1000 {out}")
