"""Snapshot overlay helper shared by /snap.jpg (ZED1) and /snap2.jpg (ZED2).

Same style on both: red tint on mask pixels, green bbox + centroid cross,
coverage/age label, BGR bytes ready for JPEG encode. Mask/RGB size mismatch
must resize, and an empty mask must not crash.

Run (ROS-sourced): docker exec vm-viewer bash -c
  'source /opt/ros/humble/setup.bash && python3 <this file>'
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from seg_view import _overlay_snap


def check(cond, msg):
    if not cond:
        raise AssertionError(msg)
    print(f"ok: {msg}")


rng = np.random.default_rng(3)
img = rng.integers(0, 255, (90, 160, 3)).astype(np.uint8)
m = np.zeros((90, 160), np.uint8)
m[20:60, 40:120] = 255  # 50x80 blob -> cov 3200/14400 = 22.2%

out, cov = _overlay_snap(img, m, 1.0)
check(out.shape == img.shape and out.dtype == np.uint8, "shape/dtype kept")
check(abs(cov - 100.0 * 3200 / m.size) < 1e-9, f"coverage exact ({cov:.2f}%)")
check(bool((out[30, 60] != img[30, 60]).any()), "mask pixel tinted")
check(bool((out[5, 5] == img[5, 5]).all()), "background pixel untouched")

small = np.zeros((45, 80), np.uint8)
small[10:30, 20:60] = 255
out2, cov2 = _overlay_snap(img, small, 0.5)
check(out2.shape == img.shape, "half-size mask resized to RGB")
check(cov2 > 0, "resized mask still covers")

out3, cov3 = _overlay_snap(img, np.zeros_like(m), 9.9)
check(cov3 == 0.0, "empty mask -> 0% coverage")
check(out3.shape == img.shape, "empty mask does not crash")
print("6/6 snap overlay checks passed")
