"""Read-only: grab one wrist RealSense colour + aligned depth + intrinsics (domain 8) -> npz.
Run inside isaac-build:  python3 capture_wrist.py OUT.npz"""
import sys
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

out = sys.argv[1]
rclpy.init()
n = rclpy.create_node('wrist_capture_readonly')
got = {}


def img(key):
    def cb(m):
        if key in got:
            return
        dt = {'rgb8': np.uint8, 'bgr8': np.uint8, '16UC1': np.uint16, '32FC1': np.float32}[m.encoding]
        ch = 3 if m.encoding in ('rgb8', 'bgr8') else 1
        a = np.frombuffer(m.data, dt).reshape(m.height, m.width, ch) if ch == 3 else \
            np.frombuffer(m.data, dt).reshape(m.height, m.step // np.dtype(dt).itemsize)[:, :m.width]
        if m.encoding == 'bgr8':
            a = a[..., ::-1]
        got[key] = (a.copy(), m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.encoding)
    return cb


n.create_subscription(Image, '/wrist/camera/color/image_raw', img('rgb'), qos_profile_sensor_data)
n.create_subscription(Image, '/wrist/camera/aligned_depth_to_color/image_raw', img('depth'), qos_profile_sensor_data)
n.create_subscription(CameraInfo, '/wrist/camera/color/camera_info',
                      lambda m: got.setdefault('K', (np.array(m.k).reshape(3, 3), m.header.frame_id)), qos_profile_sensor_data)
t0 = time.time()
while len(got) < 3 and time.time() - t0 < 10:
    rclpy.spin_once(n, timeout_sec=0.1)
if len(got) < 3:
    raise SystemExit(f'missing {set(["rgb", "depth", "K"]) - set(got)}')
rgb, ts_rgb, _ = got['rgb']
depth, ts_d, enc = got['depth']
K, frame = got['K']
scale = 0.001 if enc == '16UC1' else 1.0
np.savez_compressed(out, rgb=rgb, depth_m=depth.astype(np.float32) * scale, K=K, frame=frame,
                    stamp_rgb=ts_rgb, stamp_depth=ts_d, wall=time.time())
print(f'saved {out} rgb {rgb.shape} depth {depth.shape} frame {frame} dt {abs(ts_rgb - ts_d) * 1000:.0f} ms')
