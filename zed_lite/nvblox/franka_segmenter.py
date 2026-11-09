"""Robot self-filter in front of nvblox, with cuRobo's RobotSegmenter (the class Isaac ROS cuMotion's robot_segmenter
node wraps). Runs in Isaac's Python 3.12 with our curobo-isaac6 build and Isaac 6.0's bundled ROS 2 Humble rclpy.
Per depth frame: take the joint state with the closest stamp (within --slop), pose cuRobo's Franka collision spheres
(fr3.yml, the FR3; franka.yml = the Panda) by forward kinematics, back-project the depth with the camera intrinsics into the robot base frame (camera
pose from TF: the static base -> camera chain), and zero every pixel within --threshold of a sphere surface.
Publishes the robot-free depth (same header, 32FC1 m, 0 = removed) for nvblox and the mask (mono8, 255 = robot).
    zed_lite/nvblox/run_segmenter.sh [--threshold 0.05 --buffer 0.01]
"""
import argparse, bisect, collections, json, sys, time
import numpy as np
import rclpy
import torch
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState as JointStateMsg
from tf2_ros import Buffer, TransformListener

from curobo.types.base import TensorDeviceType
from curobo.types.camera import CameraObservation
from curobo.types.math import Pose
from curobo.types.state import JointState
from curobo.wrap.model.robot_segmenter import RobotSegmenter

ap = argparse.ArgumentParser()
ap.add_argument("--robot", default="fr3.yml", help="cuRobo robot config (collision spheres); fr3.yml = the lab FR3")
ap.add_argument("--depth", default="/zed/zed_node/depth/depth_registered")
ap.add_argument("--info", default="/zed/zed_node/depth/camera_info")
ap.add_argument("--joint-states", default="/joint_states")
ap.add_argument("--out-depth", default="/franka_seg/depth")
ap.add_argument("--out-mask", default="/franka_seg/robot_mask")
ap.add_argument("--threshold", type=float, default=0.05, help="margin beyond the sphere surfaces (m)")
ap.add_argument("--buffer", type=float, default=0.01, help="cuRobo collision_sphere_buffer: inflates every sphere (m)")
ap.add_argument("--slop", type=float, default=0.1, help="max |depth stamp - joint state stamp| (s)")
ap.add_argument("--qos", choices=["reliable", "best_effort"], default="reliable",
                help="depth in/out: 2 MB frames between Isaac's bundled ROS libs and the containers drop a lot as best effort")
a = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])


def mat2quat(R):                                                # 3x3 rotation -> (w, x, y, z)
    R = np.asarray(R, float); t = np.trace(R)
    if t > 0:
        s = 2 * np.sqrt(t + 1); return np.array([s / 4, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s])
    i = int(np.argmax(np.diag(R))); j, k = (i + 1) % 3, (i + 2) % 3
    s = 2 * np.sqrt(1 + R[i, i] - R[j, j] - R[k, k]); q = np.zeros(4); q[0] = (R[k, j] - R[j, k]) / s
    q[1 + i] = s / 4; q[1 + j] = (R[j, i] + R[i, j]) / s; q[1 + k] = (R[k, i] + R[i, k]) / s; return q


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


class FrankaSegmenter(Node):
    def __init__(self):
        super().__init__("franka_segmenter")
        self.ta = TensorDeviceType()
        self.seg = RobotSegmenter.from_robot_file(a.robot, collision_sphere_buffer=a.buffer, distance_threshold=a.threshold,
                                                  use_cuda_graph=False)
        self.base = self.seg.base_link
        self.joints = list(self.seg.robot_world.kinematics.joint_names)
        self.tf = Buffer(); TransformListener(self.tf, self)
        self.js_t, self.js_q = collections.deque(maxlen=2000), collections.deque(maxlen=2000)
        self.K = None; self.proj_shape = None
        img_qos = QoSProfile(depth=2, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE if a.qos == "reliable"
                             else ReliabilityPolicy.BEST_EFFORT)
        self.pub_d = self.create_publisher(Image, a.out_depth, img_qos)
        self.pub_m = self.create_publisher(Image, a.out_mask, img_qos)
        self.create_subscription(JointStateMsg, a.joint_states, self.on_js, 50)
        self.create_subscription(CameraInfo, a.info, self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, a.depth, self.on_depth, img_qos)
        self.st = collections.Counter(); self.lat = []; self.dt = []; self.frac = []; self.size = None
        self.create_timer(5.0, self.log_stats)
        self.get_logger().info(f"cuRobo segmenter: {a.robot} base {self.base}, joints {self.joints}, "
                               f"threshold {a.threshold} m, sphere buffer {a.buffer} m; {a.depth} -> {a.out_depth}")

    def on_js(self, m):
        idx = {n: i for i, n in enumerate(m.name)}
        if not all(j in idx for j in self.joints): return
        t = stamp(m.header)
        if self.js_t and t < self.js_t[-1]:                     # sim restarted: drop the old history
            self.js_t.clear(); self.js_q.clear()
        self.js_t.append(t); self.js_q.append([m.position[idx[j]] for j in self.joints])

    def on_info(self, m):
        self.K = np.array(m.k, dtype=np.float32).reshape(3, 3)

    def camera_pose(self, frame):
        tr = self.tf.lookup_transform(self.base, frame, rclpy.time.Time()).transform   # static chain: base -> camera
        q = tr.rotation; w, x, y, z = q.w, q.x, q.y, q.z
        R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                      [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                      [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        return Pose(position=self.ta.to_device([[tr.translation.x, tr.translation.y, tr.translation.z]]),
                    quaternion=self.ta.to_device([mat2quat(R).tolist()]))

    def on_depth(self, m):
        t_in = time.time(); self.st["depth_in"] += 1
        if self.K is None or not self.js_t:
            self.st["skip_no_info_or_js"] += 1; return
        td = stamp(m.header); ts = list(self.js_t); i = bisect.bisect_left(ts, td)
        cand = [k for k in (i - 1, i) if 0 <= k < len(ts)]; k = min(cand, key=lambda k: abs(ts[k] - td))
        if abs(ts[k] - td) > a.slop:
            self.st["skip_no_js_within_slop"] += 1; return
        try:
            pose = self.camera_pose(m.header.frame_id)
        except Exception:
            self.st["skip_no_tf"] += 1; return
        if m.encoding == "32FC1":
            d = np.frombuffer(m.data, np.float32).reshape(m.height, m.width)
        elif m.encoding == "16UC1":
            d = np.frombuffer(m.data, np.uint16).reshape(m.height, m.width).astype(np.float32) / 1000.0
        else:
            self.st[f"skip_encoding_{m.encoding}"] += 1; return
        d_mm = self.ta.to_device(np.nan_to_num(d, nan=0.0, posinf=0.0, neginf=0.0) * 1000.0).unsqueeze(0)
        cam = CameraObservation(depth_image=d_mm, intrinsics=self.ta.to_device(self.K).unsqueeze(0), pose=pose)
        if self.proj_shape != d.shape:
            self.seg.update_camera_projection(cam); self.proj_shape = d.shape
        js = JointState.from_position(self.ta.to_device([self.js_q[k]]), joint_names=self.joints)
        mask, filt = self.seg.get_robot_mask(cam, js)
        mask = mask[0].cpu().numpy(); out = (filt[0] / 1000.0).cpu().numpy().astype(np.float32)
        od = Image(header=m.header, height=m.height, width=m.width, encoding="32FC1", is_bigendian=0, step=m.width * 4,
                   data=out.tobytes())
        om = Image(header=m.header, height=m.height, width=m.width, encoding="mono8", is_bigendian=0, step=m.width,
                   data=(mask.astype(np.uint8) * 255).tobytes())
        self.pub_d.publish(od); self.pub_m.publish(om)
        self.st["depth_out"] += 1; self.lat.append((time.time() - t_in) * 1000); self.dt.append(ts[k] - td); self.frac.append(float(mask.mean()))
        self.size = [m.width, m.height]

    def log_stats(self):                                        # every 5 s, whether or not frames got through
        extra = {"js_buffered": len(self.js_t), "have_info": self.K is not None}
        if self.lat:
            extra.update(proc_ms_median=round(float(np.median(self.lat)), 1), js_minus_depth_stamp_s_median=round(float(np.median(self.dt)), 3),
                         masked_frac_median=round(float(np.median(self.frac)), 4), size=self.size)
        self.get_logger().info("stats " + json.dumps({**self.st, **extra})); self.lat, self.dt, self.frac = [], [], []


rclpy.init(); node = FrankaSegmenter()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
