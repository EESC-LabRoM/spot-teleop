#!/usr/bin/env python3
"""ZED2 carve mask for nvblox camera_1 (2026-11-04). No second SAM2.

The drill box (fr3_link0 AABB from ZED1 live mask+depth, +ZED2 points, see
vm_offload/seg_view.py drill.json) is padded and projected into every ZED2
depth frame; the filled convex hull of the 8 corners is the mask. Header is
copied from the depth frame (nvblox pairs mask<->depth by stamp).
Hold rules like vm_mask_sync: box fresh (age <= --max-age) -> mask;
stale up to --release-after -> skip frame (nothing integrates); after -> empty.

  python3 vm_mask_sync2.py [--depth /vm/cable2/depth] [--out /vm/target_mask2]
"""
import argparse
import json
import threading
import time
import urllib.request

import cv2
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument('--depth', default='/vm/cable2/depth')
ap.add_argument('--info', default='/zed2/zed_node/depth/camera_info')
ap.add_argument('--out', default='/vm/target_mask2')
ap.add_argument('--drill-url', default='http://127.0.0.1:8791/drill.json')
ap.add_argument('--pad', type=float, default=0.03, help='box padding (m)')
ap.add_argument('--dilate', type=int, default=8)
ap.add_argument('--max-age', type=float, default=6.0)
ap.add_argument('--release-after', type=float, default=20.0)
ap.add_argument('--calib', default='/home/nexus/spot-teleop/fr3_real_repro/'
                'zed2_align/zed2_extrinsic_20261105_robotfit7.json')
a = ap.parse_args()

import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

qbe = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST,
                 reliability=ReliabilityPolicy.BEST_EFFORT)
qrel = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                  reliability=ReliabilityPolicy.RELIABLE)


def quat_R(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Box:
    """Polls drill.json; keeps (lo, hi, wall_time_of_box)."""
    def __init__(self):
        self.lo = self.hi = None
        self.t = None
        threading.Thread(target=self.loop, daemon=True).start()

    def loop(self):
        while True:
            try:
                d = json.load(urllib.request.urlopen(a.drill_url, timeout=1.0))
                if d.get('ok') and d.get('frame') == 'fr3_link0' \
                        and d.get('age_s', 999) <= a.max_age:
                    self.lo = np.array(d['aabb_min'], float) - a.pad
                    self.hi = np.array(d['aabb_max'], float) + a.pad
                    self.t = time.time() - float(d.get('age_s', 0.0))
            except Exception:
                pass
            time.sleep(0.5)


class MaskSync2(Node):
    def __init__(self):
        super().__init__('vm_mask_sync2')
        self.box = Box()
        self.info = None
        self.n = dict(mask=0, empty=0, skip=0, offimg=0)
        self.tfbuf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tfbuf, self)
        self.T_json = np.array(json.load(open(a.calib))['T_base_cam'], float)
        self.tf_src = None
        self.pub = self.create_publisher(Image, a.out, qrel)
        self.create_subscription(CameraInfo, a.info, self.on_info, qbe)
        self.create_subscription(Image, a.depth, self.on_depth, qbe)
        self.create_timer(30.0, self.log)
        self.get_logger().info(f'vm_mask_sync2: {a.depth} + drill box -> {a.out}')

    def on_info(self, m):
        self.info = m

    def RT(self, frame):
        try:
            tf = self.tfbuf.lookup_transform('fr3_link0', frame, rclpy.time.Time())
            t = tf.transform.translation
            self.tf_src = 'tf'
            return quat_R(tf.transform.rotation), np.array([t.x, t.y, t.z])
        except Exception:
            self.tf_src = 'json'
            return self.T_json[:3, :3], self.T_json[:3, 3]

    def publish(self, d, mask):
        self.pub.publish(Image(header=d.header, height=d.height, width=d.width,
                               encoding='mono8', step=d.width,
                               data=np.ascontiguousarray(mask).tobytes()))

    def on_depth(self, d):
        if self.info is None:
            return
        h, w = d.height, d.width
        bt = self.box.t
        age = 1e9 if bt is None else time.time() - bt
        if age > a.max_age:
            if age <= a.release_after:
                self.n['skip'] += 1
                return
            self.n['empty'] += 1
            return self.publish(d, np.zeros((h, w), np.uint8))
        lo, hi = self.box.lo, self.box.hi
        R, t = self.RT(d.header.frame_id or self.info.header.frame_id)
        K = np.array(self.info.k, float).reshape(3, 3)
        sx = w / float(self.info.width) if self.info.width else 1.0
        C = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1])
                      for z in (lo[2], hi[2])])
        P = (C - t) @ R  # base -> optical
        mask = np.zeros((h, w), np.uint8)
        if np.all(P[:, 2] > 0.05):
            uv = np.stack([K[0, 0] * sx * P[:, 0] / P[:, 2] + K[0, 2] * sx,
                           K[1, 1] * sx * P[:, 1] / P[:, 2] + K[1, 2] * sx], 1)
            hull = cv2.convexHull(np.round(uv).astype(np.int32))
            cv2.fillConvexPoly(mask, hull, 255)
            if a.dilate > 0:
                k = 2 * a.dilate + 1
                mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k)))
        if not mask.any():
            self.n['offimg'] += 1
        self.n['mask'] += 1
        self.publish(d, mask)

    def log(self):
        v = self.n
        age = 'none' if self.box.t is None else f'{time.time() - self.box.t:.1f}s'
        self.get_logger().info(
            f"published mask {v['mask']} (off-image {v['offimg']}) / empty {v['empty']}, "
            f"skipped {v['skip']}; box age {age}; T from {self.tf_src}")


rclpy.init()
try:
    rclpy.spin(MaskSync2())
except KeyboardInterrupt:
    pass
