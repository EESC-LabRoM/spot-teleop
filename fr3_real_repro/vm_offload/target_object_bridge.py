"""Minimal stand-in for the missing tf_projection: broadcast target_object TF.

Subscribes the tracker's /tracking_3d_point (zed1 optical frame), transforms
each point to fr3_link0 at the message stamp, and broadcasts a world-fixed
fr3_link0 -> target_object frame at 20 Hz (latched: keeps publishing the last
known point so secondaries keep a stable reproject target while the hand
re-acquires). Secondaries, the lost-guard disc, and the clear-box consumer all
key off this frame; without it the ZED2 secondary can never arm.

Same container pattern as run_vm_seed_bridge.sh (nvblox-lite + humble setup).
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.duration import Duration
from geometry_msgs.msg import PointStamped, TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster

QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                 history=HistoryPolicy.KEEP_LAST)

rclpy.init()
node = Node('vm_target_object_bridge')
buf = Buffer()
TransformListener(buf, node)
br = TransformBroadcaster(node)
latest = {}


def xform_point(t, px, py, pz):
    q = t.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
         [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
         [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    o = t.transform.translation
    return (R[0][0] * px + R[0][1] * py + R[0][2] * pz + o.x,
            R[1][0] * px + R[1][1] * py + R[1][2] * pz + o.y,
            R[2][0] * px + R[2][1] * py + R[2][2] * pz + o.z)


def on_pt(m):
    try:
        t = buf.lookup_transform(
            'fr3_link0', m.header.frame_id,
            rclpy.time.Time.from_msg(m.header.stamp),
            timeout=Duration(seconds=0.3))
    except Exception as e:
        node.get_logger().warn(f'target bridge TF fail: {e}', throttle_duration_sec=5.0)
        return
    latest['p'] = xform_point(t, m.point.x, m.point.y, m.point.z)


node.create_subscription(PointStamped, '/tracking_3d_point', on_pt, QOS)
node.get_logger().info('vm_target_object_bridge: /tracking_3d_point -> TF target_object')
n = 0
t0 = time.time()
while True:
    rclpy.spin_once(node, timeout_sec=0.05)
    if 'p' not in latest:
        continue
    x, y, z = latest['p']
    m = TransformStamped()
    m.header.frame_id = 'fr3_link0'
    m.header.stamp = node.get_clock().now().to_msg()
    m.child_frame_id = 'target_object'
    m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = x, y, z
    m.transform.rotation.w = 1.0
    br.sendTransform(m)
    n += 1
    if n % 400 == 0:
        node.get_logger().info(
            f'target_object @ ({x:.3f},{y:.3f},{z:.3f}), age {time.time() - t0:.0f}s')
