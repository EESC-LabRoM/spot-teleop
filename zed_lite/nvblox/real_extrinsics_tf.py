"""Static TF for the REAL lab: fr3_link0 (FR3 base) -> <cam>_camera_link, from a calibrated T_base_optical (EasyHeC fit JSON).
Same idea as zed_extrinsics_tf.py (sim): the ZED wrapper (publish_tf:=false) still publishes its own static chain
camera_link -> ... -> left_camera_optical_frame, so T_base_link = T_base_optical * T_optical_link is chosen such that the
wrapper's optical frame lands exactly on the calibrated pose.   Domain 8 (ZED / nvblox), run inside zed-build:
    python3 real_extrinsics_tf.py --calib /work/results/front_camera_extrinsic.json --cam zed1
"""
import argparse, json, sys, time
import numpy as np, rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener

def quat2mat(q):
    w, x, y, z = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)], [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)], [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]])
def mat2quat(R):
    R = np.asarray(R, float); t = np.trace(R)
    if t > 0:
        s = 2*np.sqrt(t + 1); return np.array([s/4, (R[2,1]-R[1,2])/s, (R[0,2]-R[2,0])/s, (R[1,0]-R[0,1])/s])
    i = int(np.argmax(np.diag(R))); j, k = (i+1) % 3, (i+2) % 3
    s = 2*np.sqrt(1 + R[i,i] - R[j,j] - R[k,k]); q = np.zeros(4); q[0] = (R[k,j]-R[j,k])/s
    q[1+i] = s/4; q[1+j] = (R[j,i]+R[i,j])/s; q[1+k] = (R[k,i]+R[i,k])/s; return q

ap = argparse.ArgumentParser()
ap.add_argument("--calib", required=True, help="JSON with T_base_cam (4x4, camera optical frame in the base frame)")
ap.add_argument("--cam", required=True, help="wrapper camera name, e.g. zed1")
ap.add_argument("--base-frame", default="fr3_link0")
a = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
link, opt = f"{a.cam}_camera_link", f"{a.cam}_left_camera_optical_frame"
T_base_opt = np.array(json.load(open(a.calib))["T_base_cam"])
rclpy.init(); node = Node(f"real_extrinsics_tf_{a.cam}"); buf = Buffer(); TransformListener(buf, node)
t0 = time.time(); T_opt_link = None
while time.time() - t0 < 60 and T_opt_link is None:
    rclpy.spin_once(node, timeout_sec=0.2)
    try:
        tr = buf.lookup_transform(opt, link, rclpy.time.Time()).transform
        T_opt_link = np.eye(4); q = tr.rotation
        T_opt_link[:3, :3] = quat2mat([q.w, q.x, q.y, q.z]); T_opt_link[:3, 3] = [tr.translation.x, tr.translation.y, tr.translation.z]
    except Exception:
        pass
if T_opt_link is None:
    node.get_logger().error(f"no wrapper TF {opt} -> {link} after 60 s (is the {a.cam} wrapper up?)"); sys.exit(1)
T_base_link = T_base_opt @ T_opt_link
m = TransformStamped(); m.header.stamp = node.get_clock().now().to_msg(); m.header.frame_id = a.base_frame; m.child_frame_id = link
m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = (float(v) for v in T_base_link[:3, 3])
w, x, y, z = mat2quat(T_base_link[:3, :3]); m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z, m.transform.rotation.w = float(x), float(y), float(z), float(w)
br = StaticTransformBroadcaster(node); br.sendTransform([m])
node.get_logger().info(f"static TF published: {a.base_frame} -> {link}; optical frame position in base {np.round(T_base_opt[:3,3],3).tolist()} (calibration {a.calib})")
rclpy.spin(node)
