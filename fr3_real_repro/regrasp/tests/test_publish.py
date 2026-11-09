"""publish.py: atomic run write, pointer, R2q/message builders.

Run: docker exec vm-viewer bash -c
  'source /opt/ros/humble/setup.bash && python3 <this file>'
"""
import json
import os
import shutil
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, '/home/nexus/spot-teleop/fr3_real_repro/grasp_loop')

from publish import (bind_allowed, grasp_target_from_plan, q2r, r2q,
                       write_grasp_binding, write_pointer, write_run)
from grasp_loop import quat_from_matrix

R = ('/home/nexus/spot-teleop/fr3_real_repro/grasp_loop/runs/'
     '20261106_152112_drill_lying3')
TMP = '/tmp/regrasp_pub_test'


def check(cond, msg):
    if not cond:
        raise AssertionError(msg)
    print(f"ok: {msg}")


check(bool((abs(r2q(np.eye(3)) - [0, 0, 0, 1]) < 1e-9).all()),
      "R2q identity")
c, s = np.cos(np.radians(45)), np.sin(np.radians(45))
q = r2q(np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]))
check(abs(abs(q[2]) - np.sin(np.radians(22.5)) * 2 / 2) < 0.02
      and abs(q[3] - np.cos(np.radians(22.5))) < 1e-6, "R2q 90-deg-z shape")
pick0 = json.load(open(f'{R}/pick.json'))
R0 = q2r(pick0['quat_xyzw'])
check(bool((abs(R0 @ R0.T - np.eye(3)) < 1e-9).all()), "q2r orthonormal")
q_rt = quat_from_matrix(R0)
check(min(float(np.linalg.norm(q_rt - pick0['quat_xyzw'])),
          float(np.linalg.norm(q_rt + pick0['quat_xyzw']))) < 1e-6,
      "q2r inverts grasp_loop quat_from_matrix")

if os.path.exists(TMP):
    shutil.rmtree(TMP)
os.makedirs(TMP)
Pc = np.load(f'{R}/cloud.npy')[:, :3].astype(float)
G = np.load(f'{R}/grasps.npy').astype(float)
ranked = [dict(idx=1, score=0.5, conf=0.8,
               metrics=dict(off_xy_mm=10.0, nnear=100, bilateral=0.9,
                            wrap_mm=20.0, vert=0.95),
               weights=dict(center_w=1.0))]
plan = dict(grasp_pos=[0.4, -0.2, 0.25], hand_R=np.eye(3).tolist(),
            problems=[])
out = write_run(TMP, 'r1', Pc[:2000], G[:20], pick0, ranked, plan, 0.01,
                dict(reason='test'), False)
check(sorted(os.listdir(out)) == sorted(
    ['cloud.json', 'cloud.npy', 'grasp_plan_input.json', 'grasps.npy',
     'loop.json', 'pick.json', 'plan.json', 'rank.json', 'summary.json']),
    "run has all 9 files")
check(not os.path.exists(os.path.join(TMP, 'r1.tmp')), "no tmp leftover")
check(json.load(open(os.path.join(out, 'loop.json')))['depth'] == 0.01,
      "loop.json carries depth")
gi = json.load(open(os.path.join(out, 'grasp_plan_input.json')))
check(sorted(gi.keys()) == ['R', 'cloud', 'pick', 'run'],
      "plan input keeps the format")

ptr = write_pointer(TMP, 'r1_ptr', out, pick0, plan, 'note')
check(os.path.exists(os.path.join(ptr, 'NOTE.txt')), "pointer has NOTE")
gt = grasp_target_from_plan(plan, 'r1')
check(gt.pos == [0.4, -0.2, 0.25] and abs(gt.quat_xyzw[3] - 1.0) < 1e-9,
      "target = grasp_pos + R2q(hand_R)")

# TF binding (Murilo's simplification): byte-contract with the coord
# lane's tools/bind_grasp.py (9 keys, guards, atomic replace).
plan['run'] = 'r1'
plan['depth'] = 0.01
plan['flip_deg'] = 0.0
bp = os.path.join(TMP, 'grasp_binding.json')
pp = os.path.join(out, 'plan.json')
write_grasp_binding(bp, plan, pp, [0.39, -0.21, 0.20], now=1700000000.0)
b = json.load(open(bp))
check(sorted(b.keys()) == ['bound_at', 'depth', 'flip_deg', 'hand_R',
                            'hand_pos', 'offset', 'plan_path', 'run_id',
                            'target_object_at_bind'],
      "binding has exactly the 9 contract keys")
check(b['run_id'] == 'r1' and b['plan_path'] == os.path.abspath(pp)
      and b['bound_at'] == 1700000000.0 and b['depth'] == 0.01
      and b['flip_deg'] == 0.0 and b['hand_pos'] == [0.4, -0.2, 0.25]
      and b['hand_R'] == np.eye(3).tolist()
      and b['target_object_at_bind'] == [0.39, -0.21, 0.20],
      "binding carries plan pose + TF sample + provenance")
check([round(v, 9) for v in b['offset']] == [0.01, 0.01, 0.05],
      "offset = hand_pos - target_object")
check(not os.path.exists(bp + '.tmp'), "binding write leaves no tmp")
for bad, why in ((dict(plan, problems=['x']), 'dirty plan'),
                 (dict(plan, hand_R=None), 'missing hand_R')):
    bad = dict(bad)
    if why == 'missing hand_R':
        del bad['hand_R']
    try:
        write_grasp_binding(bp, bad, pp, [0.39, -0.21, 0.20])
        check(False, f"binding must refuse {why}")
    except ValueError as e:
        check(True, f"binding refuses {why}")
try:
    write_grasp_binding(bp, plan, pp, [9.0, 9.0, 9.0])
    check(False, "binding must refuse a wild offset")
except ValueError as e:
    check('0.35' in str(e), "binding refuses offset > 0.35 m")

# Monotonic binding: a restart must never rebind an OLDER run (loop run
# ids sort chronologically). Missing/corrupt file never blocks a bind.
old = os.path.join(TMP, 'grasp_binding_old.json')
json.dump(dict(b, run_id='20261106_203025_drill_loop_regen'),
          open(old, 'w'))
check(bind_allowed('20261106_205015_drill_loop_regen', old),
      "newer run may bind")
check(not bind_allowed('20261106_195756_drill_loop_regen', old),
      "older run must not rebind")
check(bind_allowed('20261106_203025_drill_loop_regen', old),
      "same run may rebind (fresh TF sample)")
check(bind_allowed('20261106_205015_drill_loop_regen',
                   os.path.join(TMP, 'nope.json')),
      "missing binding never blocks")
open(old, 'w').write('{garbled')
check(bind_allowed('20261106_205015_drill_loop_regen', old),
      "corrupt binding never blocks")
shutil.rmtree(TMP)
print("22/22 publish checks passed")
