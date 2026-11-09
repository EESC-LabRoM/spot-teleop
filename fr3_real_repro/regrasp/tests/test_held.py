"""Held-drill freeze: closing the gripper latches the loop to LOCKED (no
re-anchor, no REUSE/REGEN, no yaw emit) until the gripper reopens, when
tracking resumes against the release pose. A closed gripper may be empty,
so the latch is width-only; the MPC side attaches collision geometry only
for closes near the grasp target.

Run: python3 tests/test_held.py (no ROS, recorded clouds only).
"""
import os
import shutil
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from loop import Engine
from source import tick_from_grasp

RB = '/home/nexus/spot-teleop/fr3_real_repro/grasp_loop/runs'
R2 = f'{RB}/20261106_150037_drill_lying2'
MESH = np.load(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            'data_mesh_test.npy'))
TMP = '/tmp/regrasp_held_test'
Q0 = np.zeros(7)


def check(cond, msg):
    if not cond:
        raise AssertionError(msg)
    print(f"ok: {msg}")


def ticks(run, n=3):
    P = np.load(f'{run}/cloud.npy')[:, :3].tolist()
    return [tick_from_grasp({'cloud': P, 'run': run.split('/')[-1],
                             'live': {'cloud': 'live_mask_depth+zed2_mask',
                                      'n_zed2': 3890, 'zed2_T': 'tf',
                                      'cloud_age_s': 0.5}})
            for _ in range(n)]


def shifted(tick, dx):
    P = (np.asarray(tick.pts) + [dx, 0, 0]).tolist()
    return tick_from_grasp({'cloud': P, 'run': 'shift',
                            'live': {'cloud': 'live_mask_depth+zed2_mask',
                                     'n_zed2': tick.n_zed2, 'zed2_T': 'tf',
                                     'cloud_age_s': 0.5}})


def stub_grasps(cloud_npy, num):
    return f'{R2}/grasps.npy'


if os.path.exists(TMP):
    shutil.rmtree(TMP)
os.makedirs(TMP)
decisions = []
yaws = []
eng = Engine(run_base=TMP, graspgen_fn=stub_grasps, mesh_V=MESH,
             decide_log=decisions.append, yaw_sink=yaws.append)

t0 = 3000.0
s2 = ticks(R2)
for i, t in enumerate(s2):
    st, tgt = eng.step(t, joints=Q0, width=0.08, now=t0 + i)
check(st.state == 'TRACKING' and st.trusted and tgt is not None,
      "bootstrap tracks with an open gripper")
anchor_run = eng.anchor['run_id']
runs_before = sorted(os.listdir(TMP))
yaws_before = len(yaws)

# Close on the drill: freeze, no target.
st, tgt = eng.step(s2[0], joints=Q0, width=0.05, now=t0 + 10)
check(st.state == 'LOCKED' and tgt is None and st.reason == 'held-closed',
      f"close latches LOCKED ({st.state}/{st.reason})")

# Drill rides the hand (+30 mm): no re-anchor, no reuse/regen, no yaw.
for i in range(6):
    st, tgt = eng.step(shifted(s2[0], 0.030), joints=Q0, width=0.05,
                       now=t0 + 11 + i * 0.5)
check(st.state == 'LOCKED' and eng.anchor['run_id'] == anchor_run
      and sorted(os.listdir(TMP)) == runs_before,
      "held drill never re-anchors")
check(len(yaws) == yaws_before, "no yaw emit while held")

# Reopen at the release pose: tracking resumes, move resolves.
states = set()
for i in range(6):
    st, tgt = eng.step(shifted(s2[0], 0.030), joints=Q0, width=0.08,
                       now=t0 + 20 + i * 0.5)
    states.add(st.state)
check('REUSE' in states and eng.anchor['run_id'] != anchor_run,
      f"reopen re-anchors at the release pose ({sorted(states)})")
check(tgt is not None, "target back after release")

# Hysteresis mirrors the publisher (close <0.070, open >=0.075).
st, _ = eng.step(s2[0], joints=Q0, width=0.05, now=t0 + 30)
check(st.state == 'LOCKED', "re-close latches again")
st, _ = eng.step(s2[0], joints=Q0, width=0.072, now=t0 + 31)
check(st.state == 'LOCKED', "0.072 stays latched (hysteresis)")
st, tgt = eng.step(s2[0], joints=Q0, width=0.08, now=t0 + 32)
check(st.state == 'TRACKING' and tgt is not None, "0.08 releases")

# Missing width never latches: fail-open tracking.
st, _ = eng.step(s2[0], joints=Q0, width=None, now=t0 + 40)
check(st.state == 'TRACKING' and st.trusted, "width=None keeps tracking")

print("held-drill freeze: all green")
