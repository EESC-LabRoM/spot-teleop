"""Save one depth frame per topic (32FC1 m) with its camera_info, to check what the ZED depth contains (e.g. a thin part
of an object) before nvblox fuses it.
    python3 depth_dump.py --out /ws/nvblox/out/<run>/depth.npz --pairs NAME:DEPTH_TOPIC:INFO_TOPIC [...]
"""
import argparse, sys, time
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

ap = argparse.ArgumentParser()
ap.add_argument("--out", required=True)
ap.add_argument("--pairs", nargs="+", required=True)
ap.add_argument("--timeout", type=float, default=60.0)
a = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
rclpy.init(); node = rclpy.create_node("depth_dump"); got = {}
for pr in a.pairs:
    name, dt, it = pr.split(":")
    def on_d(m, name=name):
        if name + "_depth" not in got:
            got[name + "_depth"] = np.frombuffer(bytes(m.data), np.float32).reshape(m.height, m.width).copy()
            got[name + "_frame"] = m.header.frame_id; got[name + "_stamp"] = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    def on_i(m, name=name):
        got.setdefault(name + "_K", np.array(m.k).reshape(3, 3))
    for rel in (ReliabilityPolicy.RELIABLE, ReliabilityPolicy.BEST_EFFORT):
        node.create_subscription(Image, dt, on_d, QoSProfile(depth=2, reliability=rel))
    node.create_subscription(CameraInfo, it, on_i, qos_profile_sensor_data)
t0 = time.time(); need = 3 * len(a.pairs) + len(a.pairs)
while time.time() - t0 < a.timeout and len(got) < need:
    rclpy.spin_once(node, timeout_sec=0.2)
np.savez_compressed(a.out, **got); print("saved", a.out, sorted(got))
