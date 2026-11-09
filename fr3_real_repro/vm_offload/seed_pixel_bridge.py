"""Minimal static-camera stand-in for tf_projection: seed_3d -> seed_pixel.

ZED1 never moves, so reprojection is just pinhole projection with the live
camera_info. Publishes at 10 Hz for --dur seconds, then exits.
"""
import sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo

QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                 history=HistoryPolicy.KEEP_LAST)
dur = float(sys.argv[1]) if len(sys.argv) > 1 else 40.0
seed3_topic = sys.argv[2] if len(sys.argv) > 2 else '/vm_shadow/seed_3d'
seedpx_topic = sys.argv[3] if len(sys.argv) > 3 else '/vm_shadow/seed_pixel'

rclpy.init()
node = Node('vm_seed_pixel_bridge')
info = {}
node.create_subscription(CameraInfo, '/zed1/zed_node/depth/camera_info',
                         lambda m: info.setdefault('k', (m.k, m.header.frame_id)),
                         QOS)
t0 = time.time()
while 'k' not in info and time.time() - t0 < 15:
    rclpy.spin_once(node, timeout_sec=0.2)
if 'k' not in info:
    print('NO CAMERA_INFO')
    raise SystemExit(1)
k, frame = info['k']
fx, fy, cx, cy = k[0], k[4], k[2], k[5]
print(f'intrinsics fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f} frame={frame}', flush=True)

latest = {}
node.create_subscription(PointStamped, seed3_topic,
                         lambda m: latest.__setitem__(
                             'p', (m.point.x, m.point.y, m.point.z)), QOS)
pub = node.create_publisher(PointStamped, seedpx_topic, 10)
import numpy as _np
t0 = time.time()
n = 0
last_sent, burst_until = None, 0.0
while time.time() - t0 < dur:
    rclpy.spin_once(node, timeout_sec=0.1)
    if 'p' not in latest:
        continue
    # burst 15 s per NEW seed_3d, then idle: continuous republishing
    # forces endless video re-inits and stalls mask output
    if last_sent is None or _np.linalg.norm(
            _np.array(latest['p']) - _np.array(last_sent)) > 0.005:
        burst_until = time.time() + 15
        last_sent = latest['p']
    if time.time() > burst_until:
        continue
    x, y, z = latest['p']
    if z > 0:
        m = PointStamped()
        m.header.frame_id = frame
        m.header.stamp = node.get_clock().now().to_msg()
        m.point.x = fx * (x / z) + cx
        m.point.y = fy * (y / z) + cy
        m.point.z = z
        pub.publish(m)
        n += 1
print(f'published {n} seed_pixel msgs', flush=True)
node.destroy_node()
rclpy.shutdown()
