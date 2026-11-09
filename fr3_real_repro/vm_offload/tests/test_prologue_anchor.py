"""Prologue anchor must compare like-for-like: top-8cm live box vs top-8cm run model.

Regression (2026-11-06): /grasp.json anchored the pick with the FULL live
cloud p2/p98 box against the RUN's TOP-8cm min/max model. Once mask-only ZED2
added the far side face (low, y-asymmetric), the live box center sat ~26 mm in
+y off the model center on a static scene, so the dashboard showed the grasp
2.5 cm from what grasp_test.sh executes. Both sides must use the top-8cm
subset with the robust p2/p98 estimator.

Run (ROS-sourced): docker exec vm-viewer bash -c
  'source /opt/ros/humble/setup.bash && python3 <this file>'
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import seg_view


def check(cond, msg):
    if not cond:
        raise AssertionError(msg)
    print(f"ok: {msg}")


rng = np.random.default_rng(7)


def _box(n, xr, yr, zr):
    return np.column_stack([
        rng.uniform(*xr, n), rng.uniform(*yr, n), rng.uniform(*zr, n)])


# Fake run cloud: top cluster + low part + a few fringe outliers in the top
# band (the -0.308 min/max fringe seen on the real mask-only run).
RUN = '20261106_000000_fake'
RDIR = '/tmp/prologue_anchor_runs'
os.makedirs(os.path.join(RDIR, RUN), exist_ok=True)
P = np.vstack([
    _box(2000, (0.40, 0.52), (-0.28, -0.18), (0.18, 0.25)),
    _box(2000, (0.40, 0.52), (-0.28, -0.18), (0.03, 0.18)),
    np.column_stack([rng.uniform(0.40, 0.52, 6),
                     np.full(6, -0.35),
                     np.full(6, 0.24)]),
])
np.save(os.path.join(RDIR, RUN, 'cloud.npy'), P.astype(np.float32))
open(os.path.join(RDIR, RUN, 'pick.json'), 'w').write('{}')
seg_view.RUNS = RDIR
seg_view._ext_cache.clear()

mid, model = seg_view._target_model()
check(mid == RUN, "model comes from the fake run")
check(abs(model['center'][1] - (-0.23)) < 0.005,
      f"run model y-robust to fringe (got {model['center'][1]:.4f}, want -0.23)")

# Live cloud: same object + an asymmetric low far-face (the mask-only case).
W = np.vstack([
    _box(3000, (0.40, 0.52), (-0.28, -0.18), (0.18, 0.25)),
    _box(1500, (0.40, 0.52), (-0.18, -0.14), (0.04, 0.15)),
    _box(1500, (0.40, 0.52), (-0.28, -0.18), (0.03, 0.18)),
])
legacy = (np.percentile(W, 2, axis=0) + np.percentile(W, 98, axis=0)) / 2
check(abs(legacy[1] - (-0.23)) > 0.015,
      f"legacy full-cloud box really is biased (got {legacy[1]:.4f})")

from seg_view import _live_anchor_box  # noqa: E402

lo, hi = _live_anchor_box(W)
check(abs((lo[1] + hi[1]) / 2 - (-0.23)) < 0.005,
      f"live anchor ignores the low face (got {(lo[1] + hi[1]) / 2:.4f})")

tiny = _box(10, (0.40, 0.42), (-0.24, -0.22), (0.20, 0.22))
lo, hi = _live_anchor_box(tiny)
check(np.all(np.isfinite(lo)) and np.all(np.isfinite(hi)),
      "degenerate cloud falls back without NaN")
print("4/4 prologue anchor checks passed")
