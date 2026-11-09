#!/usr/bin/env python3
"""Bind a planned grasp to the target_object TF (2026-11-06). Run in vm-viewer, domain 8.
offset = plan grasp_pos (fr3_hand, incl. --depth) - TF(fr3_link0 -> target_object) now; writes
regrasp/state/grasp_binding.json atomically (tmp + rename). grasp_target_from_tf.py picks it up and re-arms.
Refuses a plan with problems, without hand_R, or an offset > 0.35 m (TF on another object / stale).
  python3 bind_grasp.py PLAN.json [--binding PATH]
"""
import argparse
import json
import os
import time

import numpy as np
import rclpy
import tf2_ros
from rclpy.node import Node

ap = argparse.ArgumentParser()
ap.add_argument('plan')
ap.add_argument('--binding', default='/home/nexus/spot-teleop/fr3_real_repro/regrasp/state/grasp_binding.json')
a = ap.parse_args()

plan = json.load(open(a.plan))
assert plan.get('problems') == [], f'plan has problems: {plan.get("problems")}'
assert 'hand_R' in plan, 'plan has no hand_R (plan it with grasp_plan_lib)'
rclpy.init()
n = Node('bind_grasp')
buf = tf2_ros.Buffer()
tf2_ros.TransformListener(buf, n)
t0, tr = time.time(), None
while time.time() - t0 < 5 and tr is None:
    rclpy.spin_once(n, timeout_sec=0.1)
    try:
        tr = buf.lookup_transform('fr3_link0', 'target_object', rclpy.time.Time()).transform.translation
    except Exception:
        pass
assert tr is not None, 'no target_object TF (is vm-target-bridge up?)'
tobj = np.array([tr.x, tr.y, tr.z])
hand = np.array(plan['grasp_pos'], float)
off = hand - tobj
assert np.linalg.norm(off) <= 0.35, f'offset {np.linalg.norm(off):.3f} m > 0.35 m: TF not on this object?'
b = dict(run_id=plan.get('run'), plan_path=os.path.abspath(a.plan), bound_at=time.time(), hand_pos=hand.tolist(),
         hand_R=plan['hand_R'], target_object_at_bind=tobj.round(4).tolist(), offset=off.round(5).tolist(),
         depth=plan.get('depth'), flip_deg=plan.get('flip_deg'))
tmp = a.binding + '.tmp'
with open(tmp, 'w') as f:
    json.dump(b, f, indent=1)
os.replace(tmp, a.binding)
print(f'bound {b["run_id"]}: hand {hand.round(4).tolist()} target_object {tobj.round(4).tolist()} offset {off.round(4).tolist()} '
      f'(|offset| {np.linalg.norm(off):.3f} m) -> {a.binding}')
