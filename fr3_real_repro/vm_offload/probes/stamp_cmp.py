import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

rclpy.init()
node = Node('stamp_cmp')
got = {}
for rel in (ReliabilityPolicy.RELIABLE, ReliabilityPolicy.BEST_EFFORT):
    qos = QoSProfile(depth=10, reliability=rel, history=HistoryPolicy.KEEP_LAST)
    for t in ('/zed1/target_mask', '/franka_seg_zed1/depth'):
        node.create_subscription(
            Image, t,
            lambda m, tt=t: got.setdefault(tt, m.header.stamp.sec + m.header.stamp.nanosec * 1e-9),
            qos)
t0 = time.time()
while len(got) < 2 and time.time() - t0 < 20:
    rclpy.spin_once(node, timeout_sec=0.2)
for k, v in got.items():
    print(f'{k}: {v:.3f}')
if len(got) == 2:
    a = list(got.values())
    print(f'skew: {abs(a[0] - a[1]) * 1000:.0f} ms')
else:
    print('missing topics:', [t for t in ('/zed1/target_mask', '/franka_seg_zed1/depth') if t not in got])
node.destroy_node()
rclpy.shutdown()
