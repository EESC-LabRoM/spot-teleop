"""accumulator.py: 3 recorded lying3 snaps must reproduce cloud.npy exactly.

Run: docker exec vm-viewer bash -c
  'source /opt/ros/humble/setup.bash && python3 <this file>'
"""
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from accumulator import Accumulator
from source import tick_from_grasp

R = ('/home/nexus/spot-teleop/fr3_real_repro/grasp_loop/runs/'
     '20261106_152112_drill_lying3')


def check(cond, msg):
    if not cond:
        raise AssertionError(msg)
    print(f"ok: {msg}")


acc = Accumulator()
for i in (1, 2, 3):
    acc.add(tick_from_grasp(json.load(open(f'{R}/snap_{i}.json'))))
check(len(acc) == 3, "3 ticks stacked")
P = acc.cloud()
ref = np.load(f'{R}/cloud.npy')
check(P.shape == ref.shape and bool((P == ref).all()),
      f"bit-identical to recorded cloud ({P.shape})")

for i in range(10):
    acc.add(tick_from_grasp(json.load(open(f'{R}/snap_1.json'))))
check(len(acc) == 8, "tick cap holds at 8")
check(len(acc.cloud()) <= 15000, "point cap holds")
acc.reset()
check(len(acc) == 0 and len(acc.cloud()) == 0, "reset empties")

# Per-tick scale for the occlusion count gate: the gate compares single
# live ticks against the anchor, so the anchor must record mean tick size,
# not the stacked cloud size (3x bigger -> the bar sits in the noise band
# and the loop latches OCCLUDED on a healthy cloud).
acc2 = Accumulator()
ns = []
for i in (1, 2, 3):
    t = tick_from_grasp(json.load(open(f'{R}/snap_{i}.json')))
    ns.append(len(np.asarray(t.pts)))
    acc2.add(t)
check(acc2.tick_mean_n() == round(sum(ns) / len(ns)),
      f"tick mean {acc2.tick_mean_n()} vs stack {len(acc2.cloud())}")
check(acc2.tick_mean_n() < len(acc2.cloud()),
      "mean tick scale is below the stack size")
acc2.reset()
check(acc2.tick_mean_n() == 0, "empty tick mean is 0")
print("8/8 accumulator checks passed")
