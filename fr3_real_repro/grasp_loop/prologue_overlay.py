#!/usr/bin/env python3
"""Render prologue pick TCP onto a fresh frame (verification overlay).

Reads prologue.json (pick_live + quat), grabs one rgb + CameraInfo,
projects the TCP (103 mm tool offset along grasp z) into pixels using
the static chain inverse, draws a marker, saves PNG. Fails loud if the
TCP projects off-frame.
"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from live_prologue import (apply_transform, invert_transform, load_chain,
                           quat_to_matrix, Fail)

RGB_TOPIC = '/vm/cable/rgb'
INFO_TOPIC = '/zed1/zed_node/depth/camera_info'


def quat_z_axis(q):
    R = quat_to_matrix(q)
    return [R[0][2], R[1][2], R[2][2]]


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--prologue', required=True)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    pro = json.load(open(a.prologue))
    P, quat = pro['pick_live_m'], pro['quat_xyzw']
    zax = quat_z_axis(quat)
    tcp = [P[i] + zax[i] * 0.103 for i in range(3)]

    import rclpy
    from sensor_msgs.msg import Image, CameraInfo
    rclpy.init()
    node = rclpy.create_node('prologue_overlay')
    got = {}
    try:
        sub1 = node.create_subscription(Image, RGB_TOPIC,
                                        lambda m: got.setdefault('rgb', m), 10)
        sub2 = node.create_subscription(CameraInfo, INFO_TOPIC,
                                        lambda m: got.setdefault('info', m), 10)
        import time
        t0 = time.monotonic()
        while ('rgb' not in got or 'info' not in got
               and time.monotonic() - t0 < 15):
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if 'rgb' not in got or 'info' not in got:
        raise Fail('no rgb/camera_info within 15 s')
    rgb, info = got['rgb'], got['info']
    K = info.k
    fx, fy, cx, cy = K[0], K[4], K[2], K[5]

    t, q = load_chain()
    ti, qi = invert_transform(t, q)
    pc = apply_transform(ti, qi, tcp)
    if pc[2] <= 0:
        raise Fail(f'TCP behind camera (z={pc[2]:.3f})')
    u, v = int(fx * pc[0] / pc[2] + cx), int(fy * pc[1] / pc[2] + cy)
    if not (0 <= u < rgb.width and 0 <= v < rgb.height):
        raise Fail(f'TCP projects off-frame ({u},{v})')

    import numpy as np
    fr = np.frombuffer(rgb.data, np.uint8).reshape(rgb.height, rgb.width, 3)
    try:
        import cv2
        if rgb.encoding == 'rgb8':
            fr = cv2.cvtColor(fr, cv2.COLOR_RGB2BGR)
        cv2.drawMarker(fr, (u, v), (0, 0, 255), cv2.MARKER_STAR, 24, 3)
        cv2.putText(fr, f'TCP ({u},{v})', (u + 12, v - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.imwrite(a.out, fr)
    except ImportError:
        raise Fail('cv2 missing in this env')
    print(f'overlay: TCP pixel ({u},{v}) -> {a.out}')
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except Fail as e:
        print(f'OVERLAY FAIL: {e}', file=sys.stderr)
        sys.exit(2)
