"""loop.py: replay harness over RECORDED ticks (no ROS, no lab, no viewer).

Script: bootstrap on lying2 (regen -> idx152) -> static hold (no regen) ->
13 mm slide to lying3 (UNDER the 15 mm bar: correctly no regen) -> +20 mm
rigid shift (REUSE path) -> tipped maskonly cloud (REGEN path) -> forced
lock (no target). Stub graspgen serves recorded grasps per cloud; the mesh
is the committed fixture.

Run: docker exec vm-viewer bash -c
  'source /opt/ros/humble/setup.bash && python3 <this file>'
"""
import copy
import json
import os
import shutil
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from contracts import LockState
from loop import Engine, skip_unpaired_tick, still_ok
from source import tick_from_grasp

RB = '/home/nexus/spot-teleop/fr3_real_repro/grasp_loop/runs'
R2 = f'{RB}/20261106_150037_drill_lying2'
R3 = f'{RB}/20261106_152112_drill_lying3'
RM = f'{RB}/20261106_134738_drill_maskonly'
MESH = np.load(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            'data_mesh_test.npy'))
TMP = '/tmp/regrasp_loop_test'


def check(cond, msg):
    if not cond:
        raise AssertionError(msg)
    print(f"ok: {msg}")


# Static joint evidence, like the live shell passes every tick. REGEN is
# fail-closed without it (see the stillness gate checks at the bottom).
Q0 = np.zeros(7)


NZ = {R2: 3890, R3: 3819, RM: 4000}


def ticks(run, n=3):
    P = np.load(f'{run}/cloud.npy')[:, :3].tolist()
    return [tick_from_grasp({'cloud': P, 'run': run.split('/')[-1],
                             'live': {'cloud': 'live_mask_depth+zed2_mask',
                                      'n_zed2': NZ[run], 'zed2_T': 'tf',
                                      'cloud_age_s': 0.5}})
            for _ in range(n)]


def shifted(tick, dx):
    d = copy.deepcopy(tick.__dict__)
    d['pts'] = (np.asarray(d['pts']) + [dx, 0, 0]).tolist()
    t = tick_from_grasp({'cloud': d['pts'], 'run': 'shift',
                         'live': {'cloud': 'live_mask_depth+zed2_mask',
                                  'n_zed2': tick.n_zed2, 'zed2_T': 'tf',
                                  'cloud_age_s': 0.5}})
    return t


def thinned(tick, step=10):
    d = copy.deepcopy(tick.__dict__)
    d['pts'] = np.asarray(d['pts'])[::step].tolist()
    t = tick_from_grasp({'cloud': d['pts'], 'run': 'thin',
                         'live': {'cloud': 'live_mask_depth+zed2_mask',
                                  'n_zed2': tick.n_zed2 // step,
                                  'zed2_T': 'tf', 'cloud_age_s': 0.5}})
    return t


def stub_grasps(cloud_npy, num):
    c = np.load(cloud_npy)
    if c[:, 0].mean() > 0.4:
        return f'{RM}/grasps.npy'
    return f'{R2}/grasps.npy'


if os.path.exists(TMP):
    shutil.rmtree(TMP)
os.makedirs(TMP)
decisions = []
eng = Engine(run_base=TMP, graspgen_fn=stub_grasps, mesh_V=MESH,
             decide_log=decisions.append)

t0 = 1000.0
s2 = ticks(R2)
for i, t in enumerate(s2):
    st, tgt = eng.step(t, joints=Q0, width=0.08, now=t0 + i)
check(st.state == 'TRACKING' and st.trusted and tgt is not None,
      f"bootstrap regens ({st.reason})")
runs = sorted(os.listdir(TMP))
check(len(runs) == 1 and eng.pick['idx'] == 152,
      f"bootstrap picks idx152 ({eng.pick['idx']})")
anchor_run = runs[0]

st, tgt = eng.step(s2[0], joints=Q0, width=0.08, now=t0 + 10)
check(sorted(os.listdir(TMP)) == runs and tgt.run_id == anchor_run,
      "static ticks don't regen")

for i, t in enumerate(ticks(R3) * 2):
    st, tgt = eng.step(t, joints=Q0, width=0.08, now=t0 + 20 + i * 0.5)
check(sorted(os.listdir(TMP)) == runs,
      "13 mm slide stays under the 15 mm bar (no regen)")

sh = [shifted(s2[0], 0.020) for _ in range(6)]
states = set()
for i, t in enumerate(sh):
    st, tgt = eng.step(t, joints=Q0, width=0.08, now=t0 + 40 + i * 0.5)
    states.add(st.state)
check('REUSE' in states, f"+20 mm rigid shift reuses ({sorted(states)})")
check(abs(eng.pick['pos'][0] - 0.4733) < 0.015,
      f"reused pick carried +20 mm ({eng.pick['pos'][0]:.4f})")

for i, t in enumerate(ticks(RM) * 2):
    st, tgt = eng.step(t, joints=Q0, width=0.08, now=t0 + 60 + i * 0.5)
    states.add(st.state)
check('REGEN' in states or 'TRACKING' in states, "tipped cloud regens")
check(len([r for r in os.listdir(TMP) if 'regen' in r]) >= 1, "regen ran")

st, tgt = eng.step(s2[0], joints=Q0, width=0.08,
                   lock=LockState(state='HELD_OBJECT', mtime=t0),
                   now=t0 + 80)
check(st.state == 'LOCKED' and tgt is None, "HELD_OBJECT kills the target")
check(len(decisions) > 10, "decisions logged")

# Stillness gate: REGEN (GPU inference) waits for 2 s of still joints;
# CPU REUSE is unaffected and the held target keeps publishing while
# a post-move REGEN is deferred.
from collections import deque


def ring(pairs):
    return deque([(t, np.asarray(q, float)) for t, q in pairs], maxlen=48)


check(still_ok(ring([(0.0, Q0), (1.0, Q0), (2.0, Q0)]), 2.0),
      "still_ok: 2 s of static joints passes")
check(not still_ok(ring([(0.0, Q0), (1.0, Q0 + [0.05, 0, 0, 0, 0, 0, 0]),
                         (2.0, Q0 + [0.10, 0, 0, 0, 0, 0, 0])]), 2.0),
      "still_ok: 0.05 rad/s motion fails")
check(not still_ok(ring([(1.5, Q0), (2.0, Q0)]), 2.0),
      "still_ok: 0.5 s of history is insufficient")
check(not still_ok(ring([(0.0, Q0)]), 2.0),
      "still_ok: single sample fails closed")
check(not still_ok(ring([(0.0, Q0), (1.0, Q0), (2.0, Q0)]), 5.0),
      "still_ok: stale joints fail closed")
check(still_ok(ring([(0.0, Q0), (1.0, Q0 + [0.5, 0, 0, 0, 0, 0, 0]),
                     (5.0, Q0 + [0.5, 0, 0, 0, 0, 0, 0]),
                     (6.0, Q0 + [0.5, 0, 0, 0, 0, 0, 0]),
                     (7.0, Q0 + [0.5, 0, 0, 0, 0, 0, 0])]), 7.0),
      "still_ok: motion aged out of the 2 s window passes")

calls = []
eng2 = Engine(run_base=TMP + '2', graspgen_fn=lambda c, n: calls.append(c)
              or stub_grasps(c, n), mesh_V=MESH)
os.makedirs(TMP + '2')
t1 = 2000.0
moving = [Q0 + np.array([0.05 * i, 0, 0, 0, 0, 0, 0]) for i in range(6)]
for i, t in enumerate(ticks(R2, n=6)):
    st, tgt = eng2.step(t, joints=moving[i], width=0.08, now=t1 + i)
check(calls == [] and not os.listdir(TMP + '2') and not st.trusted
      and 'deferred' in st.reason,
      f"bootstrap REGEN deferred while arm moves ({st.reason})")
for i, t in enumerate(ticks(R2, n=6)):
    st, tgt = eng2.step(t, joints=moving[-1], width=0.08,
                        now=t1 + 6 + i)
check(calls != [] and st.trusted and tgt is not None,
      "REGEN proceeds once the arm is still")
old_tgt = (tgt.pos, tgt.quat_xyzw)
n_runs = len(os.listdir(TMP + '2'))
for i, t in enumerate(ticks(RM, n=6)):
    q = moving[-1] + np.array([0.05 * i, 0, 0, 0, 0, 0, 0])
    st, tgt = eng2.step(t, joints=q, width=0.08, now=t1 + 20 + i)
check(len(os.listdir(TMP + '2')) == n_runs and st.trusted
      and tgt is not None and tgt.pos == old_tgt[0],
      "tipped REGEN deferred but the held target keeps publishing")
shutil.rmtree(TMP + '2')

# Occlusion count gate compares single live ticks vs the anchor, so the
# anchor records mean tick size. Bootstrap on 3 SPREAD ticks (dedupe keeps
# ~3x): a following full-density tick must TRACK, not latch OCCLUDED.
os.makedirs(TMP + '3')
eng3 = Engine(run_base=TMP + '3', graspgen_fn=stub_grasps, mesh_V=MESH)
t3 = 3000.0
spread = [shifted(s2[0], dx) for dx in (-0.004, 0.0, 0.004)]
for i, t in enumerate(spread):
    st, tgt = eng3.step(t, joints=Q0, width=0.08, now=t3 + i)
check(st.trusted and tgt is not None, "spread bootstrap regens")
check(eng3.anchor['n'] < len(eng3.anchor['cloud']),
      f"anchor n is per-tick scale ({eng3.anchor['n']} < "
      f"{len(eng3.anchor['cloud'])})")
st, tgt = eng3.step(spread[2], joints=Q0, width=0.08, now=t3 + 10)
check(st.state == 'TRACKING' and st.trusted and tgt is not None,
      f"full tick after spread anchor tracks ({st.state}/{st.reason})")
shutil.rmtree(TMP + '3')

# Median count gate: one thin tick (an unpaired zed2_none arrival between
# mask pairings) must not flap the target; a sustained collapse latches
# OCCLUDED and recovery un-latches. Shell skip/untrusted-stale behavior is
# ROS-side; its acceptance is the live 60 s steadiness run.
check(skip_unpaired_tick('unpaired: src=live_mask_depth+zed2_none foo'),
      "unpaired ticks skip")
check(not skip_unpaired_tick('fetch: URLError')
      and not skip_unpaired_tick('ok'),
      "fetch failures and ok never skip")
os.makedirs(TMP + '4')
eng4 = Engine(run_base=TMP + '4', graspgen_fn=stub_grasps, mesh_V=MESH)
t4 = 4000.0
for i, t in enumerate(ticks(R2, n=3)):
    st, tgt = eng4.step(t, joints=Q0, width=0.08, now=t4 + i)
for i, t in enumerate(ticks(R2, n=3)):
    st, tgt = eng4.step(t, joints=Q0, width=0.08, now=t4 + 2.5 + i * 0.5)
thin = thinned(s2[0])
st, tgt = eng4.step(thin, joints=Q0, width=0.08, now=t4 + 4.0)
check(st.state == 'TRACKING' and st.trusted and tgt is not None,
      f"single thin tick tolerated ({st.reason})")
for i in range(6):
    st, tgt = eng4.step(thin, joints=Q0, width=0.08, now=t4 + 4.5 + i * 0.5)
check(st.state == 'OCCLUDED' and not st.trusted and tgt is None,
      f"sustained collapse occludes ({st.reason})")
for i in range(4):
    st, tgt = eng4.step(s2[0], joints=Q0, width=0.08, now=t4 + 8.0 + i * 0.5)
check(st.state == 'TRACKING' and st.trusted and tgt is not None,
      "recovery un-latches")
shutil.rmtree(TMP + '4')

# adopt_run: adopt a valid run dir as the anchor without regenerating
# (restart continuity: no re-bootstrap inference, no dashboard flip).
os.makedirs(TMP + '5')
eng5 = Engine(run_base=TMP + '5', graspgen_fn=stub_grasps, mesh_V=MESH)
for i, t in enumerate(ticks(R2, n=3)):
    eng5.step(t, joints=Q0, width=0.08, now=5000.0 + i)
run5 = sorted(os.listdir(TMP + '5'))[0]
eng6 = Engine(run_base=TMP + '5', graspgen_fn=stub_grasps, mesh_V=MESH)
eng6.adopt_run(os.path.join(TMP + '5', run5))
check(eng6.anchor['run_id'] == run5 and eng6.state == 'TRACKING'
      and eng6.trusted and eng6.pick['idx'] == eng5.pick['idx'],
      f"adopt restores {run5} as trusted anchor")
st, tgt = eng6.step(s2[0], joints=Q0, width=0.08, now=5100.0)
check(st.trusted and tgt is not None
      and sorted(os.listdir(TMP + '5')) == [run5],
      "adopted anchor tracks with no regen")
shutil.copytree(os.path.join(TMP + '5', run5), os.path.join(TMP + '5', 'dirty'))
dp = os.path.join(TMP + '5', 'dirty', 'plan.json')
_pl = json.load(open(dp))
_pl['problems'] = ['test: dirty']
json.dump(_pl, open(dp, 'w'))
try:
    Engine(run_base=TMP + '5').adopt_run(os.path.join(TMP + '5', 'dirty'))
    check(False, "adopt must refuse a dirty plan")
except ValueError as e:
    check('problems' in str(e), f"dirty adopt refused ({e})")
shutil.rmtree(TMP + '5')

shutil.rmtree(TMP)
print("loop replay + stillness + anchor-scale + median + adopt checks ok")
