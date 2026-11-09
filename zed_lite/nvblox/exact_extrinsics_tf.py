"""Static TF for the exact-depth (no ZED SDK) runs: world -> fr3_link0 and
fr3_link0 -> <optical> directly from Isaac's camera_<name>.json poses.
Same CLI as zed_extrinsics_tf.py (minus the wrapper lookup); --cam-link is
accepted and ignored. The depth/rgb topics carry frame_id <optical>, so the
self-filter and nvblox resolve poses through this chain alone.
"""
import argparse, json, os, sys, time
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster


def quat2mat(q):
    w, x, y, z = np.asarray(q, float)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def mat2quat(R):
    R = np.asarray(R, float); t = np.trace(R)
    if t > 0:
        s = 2 * np.sqrt(t + 1); return np.array([s / 4, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s])
    i = int(np.argmax(np.diag(R))); j, k = (i + 1) % 3, (i + 2) % 3
    s = 2 * np.sqrt(1 + R[i, i] - R[j, j] - R[k, k]); q = np.zeros(4); q[0] = (R[k, j] - R[j, k]) / s
    q[1 + i] = s / 4; q[1 + j] = (R[j, i] + R[i, j]) / s; q[1 + k] = (R[k, i] + R[i, k]) / s; return q


ap = argparse.ArgumentParser()
ap.add_argument("--workcell", required=True)
ap.add_argument("--camera-json", default="")
ap.add_argument("--world-frame", default="world")
ap.add_argument("--base-frame", default="fr3_link0")
ap.add_argument("--cam-link", default="zed_camera_link")
ap.add_argument("--optical", default="zed_left_camera_frame_optical")
ap.add_argument("--out", default="")
a = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

cam = json.load(open(a.camera_json or os.path.join(a.workcell, "camera.json"))); scene = json.load(open(os.path.join(a.workcell, "scene.json")))
T_w_usd = np.array(cam["T_world_camera_usd"])
T_w_opt = T_w_usd @ np.diag([1.0, -1.0, -1.0, 1.0])
T_w_base = np.eye(4); T_w_base[:3, 3] = scene["franka_base"]
T_base_opt = np.linalg.inv(T_w_base) @ T_w_opt


def msg(parent, child, T, stamp):
    m = TransformStamped(); m.header.stamp = stamp; m.header.frame_id = parent; m.child_frame_id = child
    m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = (float(v) for v in T[:3, 3])
    w, x, y, z = mat2quat(T[:3, :3]); m.transform.rotation.x, m.transform.rotation.y = float(x), float(y)
    m.transform.rotation.z, m.transform.rotation.w = float(z), float(w); return m


rclpy.init(); node = Node("exact_extrinsics_tf_" + a.optical.replace("_left_camera_frame_optical", ""))
br = StaticTransformBroadcaster(node)
now = node.get_clock().now().to_msg()
br.sendTransform([msg(a.world_frame, a.base_frame, T_w_base, now), msg(a.base_frame, a.optical, T_base_opt, now)])
node.get_logger().info("static TF published: " + json.dumps({
    "camera_position_in_base_m": T_base_opt[:3, 3].round(4).tolist(),
    "camera_forward_in_base": T_base_opt[:3, 2].round(4).tolist()}))
if a.out:
    json.dump({"world_T_base": T_w_base.round(5).tolist(), "base_T_left_optical": T_base_opt.round(5).tolist(),
               "camera_position_in_base_m": T_base_opt[:3, 3].round(4).tolist()}, open(a.out, "w"), indent=1)
rclpy.spin(node)
