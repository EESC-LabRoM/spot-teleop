#!/usr/bin/env python3
"""ZED2 cable fan-in for dual-camera grasp (VM103 side).

Same WHY as cable_fanin.py: the lab sender has tiny FastDDS buffers, so the
VM takes ZED2 over small image_transport topics only — compressed color
(~73 KB) + compressedDepth (~100 KB). NO raw bulk subscriptions: a second
0.9 MB raw reader would push the sender back into 27 Hz -> 0 Hz starvation.

Subscribes (cable, BEST_EFFORT):
  /zed2/zed_node/left/image_rect_color/compressed
  /zed2/zed_node/depth/depth_registered/compressedDepth
Publishes (local, RELIABLE -> satisfies RELIABLE + BE subs):
  /vm/cable2/rgb    (sensor_msgs/Image rgb8)
  /vm/cable2/depth  (sensor_msgs/Image 32FC1, meters)

Depth decode: '32FC1; compressedDepth' stores INVERSE depth, NOT mm/1000.
Fit vs raw 32FC1 ground truth (215k px): png = 10108/raw - 1016 (RMS 2.5%).
Sub-mm quantization at 1 m working distance. See zed2_align/capture.py.
Camera infos stay direct-subscribed by each consumer (tiny).
"""
import os
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage, Image

RGB_SRC = '/zed2/zed_node/left/image_rect_color/compressed'
# 2026-11-04: default = robot-masked raw depth from the lab segmenter (needed
# for nvblox; same leg as ZED1). The feed collapse that forced compressed-only
# was the lab web_video_server drop storm (fixed 21:27 UTC). Fallback:
# FANIN2_DEPTH=compressed (unmasked: robot stays in the depth).
DEPTH_MODE = os.environ.get('FANIN2_DEPTH', 'seg')
DEPTH_SRC = ('/franka_seg_zed2/depth' if DEPTH_MODE == 'seg' else
             '/zed2/zed_node/depth/depth_registered/compressedDepth')
RGB_DST = '/vm/cable2/rgb'
DEPTH_DST = '/vm/cable2/depth'

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


def decode_depth_compressed(m: CompressedImage):
    raw = bytes(m.data)
    img = cv2.imdecode(np.frombuffer(raw[12:], np.uint8), cv2.IMREAD_UNCHANGED)
    if img is None:
        return None
    if img.dtype == np.uint16 and 'compressedDepth' in m.format:
        f = img.astype(np.float64)
        return np.where(f > 0, 10108.0 / (f + 1016.0), 0).astype(np.float32)
    if img.dtype == np.uint16:
        return img.astype(np.float32) / 1000.0
    return img.astype(np.float32)


def main():
    rclpy.init()
    node = Node('vm_cable_fanin2')
    sub_q = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST)
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
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        out = Image()
        out.header = m.header
        out.height, out.width = img.shape[:2]
        out.encoding = 'rgb8'
        out.is_bigendian = 0
        out.step = out.width * 3
        out.data = img.tobytes()
        rgb_pub.publish(out)
        counts[RGB_DST] += 1

    def on_depth(m: CompressedImage):
        try:
            d = decode_depth_compressed(m)
        except Exception:
            return
        if d is None:
            return
        out = Image()
        out.header = m.header
        out.height, out.width = d.shape[:2]
        out.encoding = '32FC1'
        out.is_bigendian = 0
        out.step = out.width * 4
        out.data = __import__('array').array('B', d.tobytes())
        depth_pub.publish(scale_depth(out))
        counts[DEPTH_DST] += 1

    node.create_subscription(CompressedImage, RGB_SRC, on_rgb, sub_q)
    if DEPTH_MODE == 'seg':
        def on_depth_raw(m: Image):
            depth_pub.publish(scale_depth(m))
            counts[DEPTH_DST] += 1
        node.create_subscription(Image, DEPTH_SRC, on_depth_raw, sub_q)
    else:
        node.create_subscription(CompressedImage, DEPTH_SRC, on_depth, sub_q)
    node.get_logger().info(f'fan-in2 live: compressed rgb + {DEPTH_SRC} -> /vm/cable2/*, depth scale {DEPTH_SCALE}')
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
