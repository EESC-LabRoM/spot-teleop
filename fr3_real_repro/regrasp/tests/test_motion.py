"""motion.py: recorded-run ICP (lying2->lying3 rigid, maskonly->lying tipped).

Ground truth: the hand-done reuse152 check read yaw -0.3 deg, t [-11,+8] mm,
fitness 0.96 both ways, RMSE 4.5 mm, z extent 0.166 vs 0.169.

Run: docker exec vm-viewer bash -c
  'source /opt/ros/humble/setup.bash && python3 <this file>'
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from motion import cloud_yaw, eval_trigger, icp_se2, invert_se2

R = '/home/nexus/spot-teleop/fr3_real_repro/grasp_loop/runs'
A = np.load(f'{R}/20261106_150037_drill_lying2/cloud.npy')[:, :3]
B = np.load(f'{R}/20261106_152112_drill_lying3/cloud.npy')[:, :3]


def check(cond, msg):
    if not cond:
        raise AssertionError(msg)
    print(f"ok: {msg}")


res = icp_se2(A, B)
check(res.rigid, f"lying2->lying3 rigid ({res.reason})")
# Ground truth (hand reuse152 check): A->B yaw -0.3 deg, t [-11,+8] mm.
# icp_se2 returns B->A, i.e. yaw ~0 and t ~[+11,-8] mm. (An earlier draft of
# this test expected t~=0 after probing RMSE at the wrong sign; the grid
# minimum and the executed grasp both confirm the ~13 mm slide is real.)
check(abs(res.yaw_deg) < 2.0, f"yaw {res.yaw_deg:.2f} ~= 0")
check(abs(res.t[0] * 1000 - 11) < 6 and abs(res.t[1] * 1000 + 8) < 6,
      f"B->A t {np.array(res.t[:2]) * 1000} ~= [+11,-8]")
iyaw, it = invert_se2(res)
check(abs(iyaw + res.yaw_deg) < 1e-9, "inverse yaw flips sign")
a = np.radians(4)
S = A.copy()
S[:, :2] = S[:, :2] @ np.array([[np.cos(a), np.sin(a)],
                                [-np.sin(a), np.cos(a)]]) + [0.012, -0.009]
syn = icp_se2(A, S)
b = np.radians(-4)
et = -(np.array([[np.cos(b), -np.sin(b)], [np.sin(b), np.cos(b)]])
       @ np.array([0.012, -0.009]))
check(abs(syn.yaw_deg - (-4)) < 1.5, f"synthetic yaw {syn.yaw_deg:.2f}")
check(np.hypot(*(np.array(syn.t[:2]) - et)) * 1000 < 6,
      f"synthetic t {(np.array(syn.t[:2]) * 1000).round(1)}")
C = np.load(f'{R}/20261106_134738_drill_maskonly/cloud.npy')[:, :3]
D = np.load(f'{R}/20261106_145517_drill_lying/cloud.npy')[:, :3]
res2 = icp_se2(C, D)
check(not res2.rigid and res2.zext_mm > 15,
      f"maskonly->lying tipped, not rigid ({res2.reason})")

ax, ay = 0.3552, -0.1477
mk = lambda w, dx, dy, n: [(w + i * 0.5, ax + dx, ay + dy, 30.0, n)
                           for i in range(5)]
f, r, _ = eval_trigger(mk(0, 0.001, -0.001, 8000), (ax, ay), 30.0, 8000)
check(not f, f"static pose holds ({r})")
f, r, p = eval_trigger(mk(0, 0.020, 0.005, 8000), (ax, ay), 30.0, 8000)
check(f and p is not None, f"20 mm shift fires ({r})")
f, r, _ = eval_trigger(mk(0, 0.020, 0.005, 3000), (ax, ay), 30.0, 8000)
check(not f, f"count drop never fires ({r})")
jit = [(i * 0.5, ax + (0.020 if i % 2 else 0.0), ay, 30.0, 8000)
       for i in range(5)]
f, r, _ = eval_trigger(jit, (ax, ay), 30.0, 8000)
check(not f, f"jitter inside the window holds ({r})")
check(isinstance(cloud_yaw(A), float), "cloud yaw runs")
print("11/11 motion checks passed")
