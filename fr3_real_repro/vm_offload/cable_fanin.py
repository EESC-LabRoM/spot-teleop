#!/usr/bin/env python3
"""Single-reader cable fan-in for heavy ZED topics (VM103 side).

WHY: the lab publishers use default 212 KB FastDDS send buffers. One VM
bulk stream gets full rate; two raw streams (rgb 0.9 MB + depth 0.9 MB)
oscillate 27 Hz -> 0 Hz as the sender buffer overruns. So the rgb leg
rides the wrapper's image_transport JPEG (~73 KB, 12x smaller); depth
stays raw (geometry must not be quantized). EXACTLY ONE process on the
VM may subscribe to the cable topics below. Everyone else (tracker,
nvblox, grasp_loop, probes) subscribes to the /vm/cable/* republishes,
which stay VM-local over SHM.

Subscribes (cable, BEST_EFFORT):
  /zed1/zed_node/left/image_rect_color/compressed  (~73 KB @ 30 Hz)
  /franka_seg_zed1/depth                           (640x360 32FC1 ~0.9 MB)
Publishes (local SHM, RELIABLE -> satisfies RELIABLE + BE subs):
  /vm/cable/rgb    (sensor_msgs/Image rgb8, decoded from JPEG)
  /vm/cable/depth  (republished untouched)

Camera infos are tiny and stay direct-subscribed by each consumer.
DO NOT run hz probes against the raw cable topics while this runs --
probe /vm/cable/* instead, or stop this first.
"""
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage, Image

RGB_SRC = '/zed1/zed_node/left/image_rect_color/compressed'
DEPTH_SRC = '/franka_seg_zed1/depth'
RGB_DST = '/vm/cable/rgb'
DEPTH_DST = '/vm/cable/depth'

# 2026-11-05: per-camera depth scale from the robot-fit calibration (7DOF:
# pose + scale). The deployed TF pose is only right WITH this scale applied.
DEPTH_SCALE = float(__import__('os').environ.get('DEPTH_SCALE', '1.0'))


def scale_depth(m):
    """Return m with 32FC1 depth multiplied by DEPTH_SCALE (no-op at 1.0)."""
    if DEPTH_SCALE == 1.0 or m.encoding != '32FC1':
        return m
    d = np.frombuffer(m.data, np.float32) * np.float32(DEPTH_SCALE)
    out = Image()
    out.header, out.height, out.width = m.header, m.height, m.width
    out.encoding, out.is_bigendian, out.step = m.encoding, m.is_bigendian, m.step
    # array('B') skips rclpy's per-byte Python range check (bytes made the
    # fan-in CPU-bound at ~11 Hz, 2026-11-05)
    out.data = __import__('array').array('B', d.astype(np.float32).tobytes())
    return out


def main():
    rclpy.init()
    node = Node('vm_cable_fanin')
    # BEST_EFFORT: with default (multicast) transports, RELIABLE readers
    # trigger NACK/retransmit storms on the lab sender (0 Hz); BE sails
    # through at full rate on the 8 MB kernel buffers. Pub side stays
    # RELIABLE (the tracker requires it; VM-local SHM/unicast is lossless).
    sub_q = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST)
    # RELIABLE: the tracker subscribes RELIABLE; BE subs (nvblox etc.)
    # still match a RELIABLE publisher, so this satisfies everyone.
    pub_q = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST)
    rgb_pub = node.create_publisher(Image, RGB_DST, pub_q)
    depth_pub = node.create_publisher(Image, DEPTH_DST, pub_q)
    counts = {RGB_DST: 0, DEPTH_DST: 0}

    def on_rgb(m: CompressedImage):
        try:
            img = cv2.imdecode(np.frombuffer(m.data, np.uint8), cv2.IMREAD_COLOR)
        except Exception:
            return
        if img is None:
            return
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)  # nvblox requires rgb8; tracker converts via cv_bridge
        out = Image()
        out.header = m.header
        out.height, out.width = img.shape[:2]
        out.encoding = 'rgb8'
        out.is_bigendian = 0
        out.step = out.width * 3
        out.data = img.tobytes()
        rgb_pub.publish(out)
        counts[RGB_DST] += 1

    def on_depth(m: Image):
        depth_pub.publish(scale_depth(m))
        counts[DEPTH_DST] += 1

    node.create_subscription(CompressedImage, RGB_SRC, on_rgb, sub_q)
    node.create_subscription(Image, DEPTH_SRC, on_depth, sub_q)
    node.get_logger().info(f'fan-in live: cable(compressed rgb + depth) -> /vm/cable/*, depth scale {DEPTH_SCALE}')
    t0 = time.time()
    last = dict(counts)
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.5)
        if time.time() - t0 >= 10:
            dt = time.time() - t0
            rates = ' '.join(f'{d.split("/")[-1]}={(counts[d] - last[d]) / dt:.1f}Hz'
                             for d in counts)
            node.get_logger().info(f'rates: {rates}')
            last = dict(counts)
            t0 = time.time()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
