"""Compare header stamps of the ZED depth and /joint_states (and wall time) for a few seconds."""
import time, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
rclpy.init(); n = Node("stamp_check"); last = {}
st = lambda m: m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
n.create_subscription(Image, "/zed/zed_node/depth/depth_registered", lambda m: last.__setitem__("depth", (st(m), time.time(), m.header.frame_id)), qos_profile_sensor_data)
n.create_subscription(JointState, "/joint_states", lambda m: last.__setitem__("js", (st(m), time.time(), len(m.name))), 10)
t0 = time.time()
while time.time() - t0 < 8:
    rclpy.spin_once(n, timeout_sec=0.05)
for k, v in last.items():
    print(f"{k:6s} stamp {v[0]:.3f}  received at wall {v[1]:.3f}  ({v[2]})")
if "depth" in last and "js" in last:
    print(f"depth stamp - joint_states stamp = {last['depth'][0] - last['js'][0]:+.3f} s  (both read at ~ the same wall time)")
