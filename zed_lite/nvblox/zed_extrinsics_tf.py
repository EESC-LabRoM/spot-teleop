"""Static TF for the workcell: world -> fr3_link0 (Franka FR3 base) and fr3_link0 -> zed_camera_link (the ZED on its stand).
The camera pose comes from the Isaac workcell (franka_scene.py camera.json: world pose of the ZED X CameraLeft prim, the
camera that renders the streamed left image) and the robot pose from scene.json. zed_camera_link is the root of the
ZED wrapper's URDF, so the link transform is chosen such that the wrapper's zed_left_camera_frame_optical lands exactly
on Isaac's CameraLeft:  T_base_link = T_base_optical * T_optical_link  (T_optical_link read from the wrapper's TF).
    python3 zed_extrinsics_tf.py --workcell /workcell [--base-frame fr3_link0]
    second camera: --camera-json <run>/camera_zed2.json --cam-link zed2_camera_link --optical zed2_left_camera_frame_optical
"""
import argparse, json, os, sys, time
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener


def quat2mat(q):                                                # (w, x, y, z) -> 3x3 rotation
    w, x, y, z = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def mat2quat(R):                                                # 3x3 rotation -> (w, x, y, z)
    R = np.asarray(R, float); t = np.trace(R)
    if t > 0:
        s = 2 * np.sqrt(t + 1); return np.array([s / 4, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s])
    i = int(np.argmax(np.diag(R))); j, k = (i + 1) % 3, (i + 2) % 3
    s = 2 * np.sqrt(1 + R[i, i] - R[j, j] - R[k, k]); q = np.zeros(4); q[0] = (R[k, j] - R[j, k]) / s
    q[1 + i] = s / 4; q[1 + j] = (R[j, i] + R[i, j]) / s; q[1 + k] = (R[k, i] + R[i, k]) / s; return q


ap = argparse.ArgumentParser()
ap.add_argument("--workcell", required=True, help="franka_scene.py run dir with camera.json and scene.json")
ap.add_argument("--camera-json", default="", help="camera pose file (default <workcell>/camera.json; franka_ros_arm.py writes camera_<name>.json)")
ap.add_argument("--world-frame", default="world")
ap.add_argument("--base-frame", default="fr3_link0")
ap.add_argument("--cam-link", default="zed_camera_link")
ap.add_argument("--optical", default="zed_left_camera_frame_optical")
ap.add_argument("--out", default="")
a = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

cam = json.load(open(a.camera_json or os.path.join(a.workcell, "camera.json"))); scene = json.load(open(os.path.join(a.workcell, "scene.json")))
T_w_usd = np.array(cam["T_world_camera_usd"])                 # USD camera: +X right, +Y up, looks along -Z
T_w_opt = T_w_usd @ np.diag([1.0, -1.0, -1.0, 1.0])           # ROS optical: +X right, +Y down, +Z forward
T_w_base = np.eye(4); T_w_base[:3, 3] = scene["franka_base"]  # the Franka prim is translated only (franka_scene.py)
T_base_opt = np.linalg.inv(T_w_base) @ T_w_opt


def msg(parent, child, T, stamp):
    m = TransformStamped(); m.header.stamp = stamp; m.header.frame_id = parent; m.child_frame_id = child
    m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = (float(v) for v in T[:3, 3])
    w, x, y, z = mat2quat(T[:3, :3]); m.transform.rotation.x, m.transform.rotation.y = float(x), float(y)
    m.transform.rotation.z, m.transform.rotation.w = float(z), float(w); return m


rclpy.init(); node = Node("zed_extrinsics_tf_" + a.cam_link.replace("_camera_link", "")); buf = Buffer(); TransformListener(buf, node)
t0 = time.time(); T_opt_link = None
while time.time() - t0 < 60 and T_opt_link is None:            # the wrapper's robot_state_publisher sends these as static TF
    rclpy.spin_once(node, timeout_sec=0.2)
    try:
        tr = buf.lookup_transform(a.optical, a.cam_link, rclpy.time.Time()).transform
        T_opt_link = np.eye(4); q = tr.rotation
        T_opt_link[:3, :3] = quat2mat([q.w, q.x, q.y, q.z]); T_opt_link[:3, 3] = [tr.translation.x, tr.translation.y, tr.translation.z]
    except Exception:
        pass
if T_opt_link is None:
    node.get_logger().error(f"no TF {a.optical} -> {a.cam_link} from the ZED wrapper after 60 s"); sys.exit(1)
br = StaticTransformBroadcaster(node); S = {"T_opt_link": T_opt_link}


def publish(T_opt_link):
    T_base_link = T_base_opt @ T_opt_link; now = node.get_clock().now().to_msg()
    br.sendTransform([msg(a.world_frame, a.base_frame, T_w_base, now), msg(a.base_frame, a.cam_link, T_base_link, now)])
    info = {"world_T_base": T_w_base.round(5).tolist(), "base_T_zed_camera_link": T_base_link.round(5).tolist(),
            "base_T_left_optical": T_base_opt.round(5).tolist(), "left_optical_T_zed_camera_link_from_wrapper": T_opt_link.round(5).tolist(),
            "camera_position_in_base_m": T_base_opt[:3, 3].round(4).tolist(), "camera_forward_in_base": T_base_opt[:3, 2].round(4).tolist()}
    node.get_logger().info("static TF published: " + json.dumps({k: info[k] for k in ("camera_position_in_base_m", "camera_forward_in_base")}))
    if a.out:
        json.dump(info, open(a.out, "w"), indent=1)


def recheck():                                                 # the wrapper refines its own lens transform after start-up
    try:
        tr = buf.lookup_transform(a.optical, a.cam_link, rclpy.time.Time()).transform
    except Exception:
        return
    T = np.eye(4); q = tr.rotation; T[:3, :3] = quat2mat([q.w, q.x, q.y, q.z]); T[:3, 3] = [tr.translation.x, tr.translation.y, tr.translation.z]
    # the lookup returns our own composed chain too, so compare only the wrapper's part (optical -> zed_camera_link)
    if np.abs(T - S["T_opt_link"]).max() > 1e-4:
        node.get_logger().info(f"wrapper lens transform changed by {np.linalg.norm(T[:3, 3] - S['T_opt_link'][:3, 3]) * 1000:.1f} mm: re-publishing")
        S["T_opt_link"] = T; publish(T)


publish(T_opt_link)
node.create_timer(1.0, recheck)
rclpy.spin(node)                                               # keep the static broadcaster alive
