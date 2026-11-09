"""Live yaw riding: loop emits yaw-vs-anchor, publisher rotates the bound grasp.

Covers publish.apply_yaw_to_grasp + publish.select_yaw_rel (pure) and
Engine._emit_yaw incl. the REUSE-equivalence (sink rel == icp_se2 yaw) and
the PCA 180-flip guard. Replay style like test_loop.py (recorded R3 cloud).

Run: docker exec vm-viewer bash -c
  'source /opt/ros/humble/setup.bash && python3 <this file>'
(or host python with numpy: no ROS imports at module level here)
"""
import copy
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from loop import Engine
from motion import icp_se2, invert_se2
from publish import apply_yaw_to_grasp, select_yaw_rel
from source import tick_from_grasp

RB = '/home/nexus/spot-teleop/fr3_real_repro/grasp_loop/runs'
R3 = f'{RB}/20261107_015057_drill_loop_regen'
Q0 = np.zeros(7)


def check(cond, msg):
    if not cond:
        raise AssertionError(msg)
    print(f"ok: {msg}")


def tick_of(P, run='t'):
    return tick_from_grasp({'cloud': np.asarray(P, float)[:, :3].tolist(), 'run': run,
                            'live': {'cloud': 'live_mask_depth+zed2_mask', 'n_zed2': 3800,
                                     'zed2_T': 'tf', 'cloud_age_s': 0.5}})


def rot_z(P, deg, c):
    t = np.radians(deg)
    R = np.array([[np.cos(t), -np.sin(t), 0.0], [np.sin(t), np.cos(t), 0.0], [0.0, 0.0, 1.0]])
    return (R @ (np.asarray(P, float) - c).T).T + c


# ---- publish.apply_yaw_to_grasp ----
off, q = apply_yaw_to_grasp([1.0, 0.0, 0.0], np.eye(3), 0.0)
check(np.allclose(off, [1, 0, 0]) and np.allclose(q, [0, 0, 0, 1]), "yaw 0 is identity")
off, q = apply_yaw_to_grasp([1.0, 0.0, 0.0], np.eye(3), np.pi / 2)
check(np.allclose(off, [0, 1, 0], atol=1e-9), f"offset rotates +90: {off}")
check(abs(abs(q[2]) - np.sin(np.pi / 4)) < 1e-6 and abs(q[3] - np.cos(np.pi / 4)) < 1e-6,
      f"quat is +90 z: {np.round(q, 4).tolist()}")
off, q = apply_yaw_to_grasp([0.1, 0.2, 0.3], np.eye(3), np.pi)
check(np.allclose(off, [-0.1, -0.2, 0.3], atol=1e-9), "offset rotates 180, z kept")

# ---- publish.select_yaw_rel ----
NOW = 1794015200.0
DOC = dict(rel_rad=0.5, wall=NOW - 0.2, run_id='r1')
check(select_yaw_rel(DOC, 'r1', NOW - 100, 0.1, NOW) == 0.5, "fresh match applies")
check(select_yaw_rel(None, 'r1', NOW - 100, 0.1, NOW) == 0.1, "missing doc holds")
check(select_yaw_rel(dict(DOC, wall=NOW - 5), 'r1', NOW - 100, 0.1, NOW) == 0.1, "stale holds")
check(select_yaw_rel(DOC, 'r2', NOW - 100, 0.1, NOW) == 0.0, "run mismatch resets")
check(select_yaw_rel(DOC, 'r1', NOW - 0.05, 0.1, NOW) == 0.0, "newer binding resets")
check(select_yaw_rel({'garbage': 1}, 'r1', None, 0.1, NOW) == 0.0, "corrupt resets (no run)")
check(select_yaw_rel(DOC, 'r1', None, 0.1, NOW) == 0.5, "no bound_at still applies")

# ---- Engine yaw sink: rotated cloud emits the REUSE-equivalent yaw ----
# (15 deg: inside the ICP grid; 30 deg exceeds it, see the guard test below)
P0 = np.load(f'{R3}/cloud.npy')[:, :3].astype(float)
c = P0.mean(axis=0)
sunk = []
eng = Engine(run_base='/tmp/regrasp_yaw_test', decide_log=lambda s: None,
             yaw_sink=sunk.append)
rid = eng.adopt_run(R3)
P15 = rot_z(P0, 15.0, c)
for i, t in enumerate([0.0, 0.7, 1.4, 2.1]):
    eng.step(tick_of(P15, 'yaw15'), joints=Q0, width=0.08, lock=None, now=1000.0 + t)
check(len(sunk) == 4, f"sink emits every live tick ({len(sunk)})")
check(sunk[-1]['run_id'] == rid, "sink carries anchor run id")
rel = sunk[-1]['rel_deg']
check(abs(rel - 15.0) < 5.0, f"sink tracks +15 deg table yaw (rel={rel:.1f})")
res = icp_se2(P0, P15)
check(res.rigid, f"icp rigid on the same move ({res.reason})")
iyaw, _ = invert_se2(res)
check(abs(((rel - iyaw + 180) % 360) - 180) < 5.0,
      f"sink rel {rel:.1f} == REUSE icp yaw {iyaw:.1f}")

# ---- ICP/PCA guard: a wrong-basin rigid fit falls to REGEN, not a bad REUSE ----
logged = []
sunk2 = []
eng3 = Engine(run_base='/tmp/regrasp_yaw_test', decide_log=logged.append,
              yaw_sink=sunk2.append)
eng3.adopt_run(R3)
P30 = rot_z(P0, 30.0, c)
st = None
for t in [0.0, 0.8, 1.6, 2.4, 3.2]:
    st, _ = eng3.step(tick_of(P30, 'yaw30'), joints=Q0, width=0.08, lock=None, now=2000.0 + t)
check(any('icp-pca-disagree' in l for l in logged),
      f"guard logged, got {[l[:80] for l in logged[-3:]]}")
check(getattr(st, 'reason', '') == 'regen-unavailable',
      f"guard routes past REUSE to regen (graspgen None here), reason={getattr(st, 'reason', '?')}")

# ---- flip guard: a 170-deg single-tick jump holds instead of whipping ----
eng2 = Engine(decide_log=lambda s: None, yaw_sink=sunk.append)
eng2.anchor = dict(run_id='r', yaw=10.0)
eng2.samples = [(100.0, 0, 0, 10.0, 3000), (100.5, 0, 0, 10.0, 3000)]
eng2._emit_yaw(100.5)
check(abs(sunk[-1]['rel_deg']) < 1.0, f"aligned start rel~0 ({sunk[-1]['rel_deg']})")
eng2.samples = [(100.5, 0, 0, 10.0, 3000), (101.0, 0, 0, 180.0, 3000)]
eng2._emit_yaw(101.0)
check(abs(sunk[-1]['rel_deg']) < 5.0, f"170-deg jump held ({sunk[-1]['rel_deg']})")
eng2b = Engine(decide_log=lambda s: None, yaw_sink=sunk.append)
eng2b.anchor = dict(run_id='r', yaw=10.0)
eng2b.samples = [(101.0, 0, 0, 190.0, 3000)]
eng2b._emit_yaw(101.0)
check(sunk[-1]['rel_deg'] == 0.0, f"first-emit flip gated ({sunk[-1]['rel_deg']})")
