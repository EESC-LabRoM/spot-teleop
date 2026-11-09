#!/usr/bin/env python3
"""VM-side mask restamper (replaces lab mask_sync, which receives nothing).

Republishes the latest SAM2 mask on each robot-free depth frame, copying the
depth header (that's what nvblox matches on). Same hold/skip/release rules as
lab /tmp/nvblox_real/mask_sync.py (MaskHold).
  python3 vm_mask_sync.py [--depth TOPIC] [--sam2 TOPIC] [--out TOPIC]
"""
import argparse
import time

import cv2
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument('--depth', default='/franka_seg_zed1/depth')
ap.add_argument('--sam2', default='/zed1/segmentation_mask')
ap.add_argument('--out', default='/vm/target_mask')
ap.add_argument('--max-age', type=float, default=1.5)
ap.add_argument('--release-after', type=float, default=10.0)
ap.add_argument('--roi', default='',
                help='bbox_1000 x1,y1,x2,y2: zero the mask outside it (+15px margin)')
ap.add_argument('--dilate', type=int, default=0,
                help='grow the carve mask by N px so the object rim and depth '
                     'flying pixels never integrate into the static map')
a = ap.parse_args()

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

q = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
               reliability=ReliabilityPolicy.RELIABLE)
# Publisher stays RELIABLE (BE experiment 19:xx broke RELIABLE-reader matching;
# reverted: BE readers get truncations under loss, RELIABLE readers NACK-recover).
qp = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                reliability=ReliabilityPolicy.RELIABLE)


class MaskHold:
    def __init__(self, max_age, release_after):
        self.max_age, self.release_after = max_age, release_after
        self.mask, self.t = None, None

    def update(self, mask, now):
        self.mask, self.t = mask, now

    def decide(self, shape, now):
        if self.mask is None:
            return 'empty', np.zeros(shape, np.uint8)
        age = now - self.t
        if age <= self.max_age:
            m = self.mask
            if m.shape != shape:
                m = cv2.resize(m, (shape[1], shape[0]),
                               interpolation=cv2.INTER_NEAREST)
            return 'mask', m
        if age <= self.release_after:
            return 'skip', None
        self.mask, self.t = None, None
        return 'empty', np.zeros(shape, np.uint8)


class MaskSync(Node):
    def __init__(self):
        super().__init__('vm_mask_sync')
        self.hold = MaskHold(a.max_age, a.release_after)
        self.n = dict(mask=0, empty=0, skip=0, sam2=0)
        self.pub = self.create_publisher(Image, a.out, qp)
        self.create_subscription(Image, a.sam2, self.on_sam2, q)
        self.create_subscription(Image, a.depth, self.on_depth, q)
        self.create_timer(30.0, self.log)
        self.get_logger().info(f'vm_mask_sync: {a.depth} + {a.sam2} -> {a.out}')

    def on_sam2(self, m):
        if m.encoding != 'mono8':
            return
        g = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width]
        self.hold.update(g.copy(), time.monotonic())
        self.n['sam2'] += 1

    def on_depth(self, d):
        kind, mask = self.hold.decide((d.height, d.width), time.monotonic())
        self.n[kind] += 1
        if mask is None:
            return
        if kind == 'mask' and a.roi:
            x1, y1, x2, y2 = [int(v) for v in a.roi.split(',')]
            X1 = max(int(x1 / 1000 * d.width) - 15, 0)
            Y1 = max(int(y1 / 1000 * d.height) - 15, 0)
            X2 = min(int(x2 / 1000 * d.width) + 15, d.width)
            Y2 = min(int(y2 / 1000 * d.height) + 15, d.height)
            roi = np.zeros_like(mask)
            roi[Y1:Y2, X1:X2] = mask[Y1:Y2, X1:X2]
            mask = roi
        if kind == 'mask' and a.dilate > 0 and mask.any():
            k = 2 * a.dilate + 1
            mask = cv2.dilate(mask, cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (k, k)))
        self.pub.publish(Image(header=d.header, height=d.height, width=d.width,
                               encoding='mono8', step=d.width,
                               data=np.ascontiguousarray(mask).tobytes()))

    def log(self):
        v = self.n
        age = '' if self.hold.t is None else f' (mask age {time.monotonic() - self.hold.t:.1f} s)'
        self.get_logger().info(
            f'sam2 {v["sam2"]}, published mask {v["mask"]} / empty {v["empty"]}, skipped {v["skip"]}{age}')


rclpy.init()
try:
    rclpy.spin(MaskSync())
except KeyboardInterrupt:
    pass
