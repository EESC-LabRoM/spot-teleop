#!/usr/bin/env python3
"""Reuse a planned grasp after a rigid on-table move (manual version of the loop's REUSE path, 2026-11-06).
SE(2) ICP (x, y, yaw) of the ANCHOR cloud (the cloud the grasp was made on) to the CURRENT cloud, gates
(fitness(<1 cm) > 0.6, RMSE(<2 cm) < 12 mm, z extent within 15 mm), then moves pick + R and plans with grasp_plan_lib
against the current cloud. Run in vm-viewer (numpy/scipy).
  python3 reuse_shift.py ANCHOR_INPUT.json ANCHOR_CLOUD.npy CURRENT_URL OUT_PREFIX [--depth 0.01] [--mesh-url URL]
Writes OUT_PREFIX_input.json (grasp_plan_input format) and OUT_PREFIX_plan.json; prints the gates and PROBLEMS."""
import argparse, json, math, sys, os
import numpy as np
from scipy.spatial import cKDTree
sys.path.insert(0, '/home/nexus/spot-teleop/fr3_real_repro/grasp_loop')
import grasp_plan_lib as gp

ap = argparse.ArgumentParser()
ap.add_argument('anchor_input'); ap.add_argument('anchor_cloud'); ap.add_argument('current_url'); ap.add_argument('out_prefix')
ap.add_argument('--depth', type=float, default=0.01)
ap.add_argument('--mesh-url', default='http://127.0.0.1:18089/mesh.bin')
a = ap.parse_args()

A = np.load(a.anchor_cloud)
cur = gp.load_grasp(a.current_url)
B = np.asarray(cur['cloud'], float)
ca, cb = A.mean(0), B.mean(0)
tb = cKDTree(B)


def Rz(th):
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


best = None
for th0 in np.radians(np.arange(-180, 180, 15)):
    th, t = th0, np.r_[cb[:2] - ca[:2], 0.0]
    for _ in range(40):
        P = (A - ca) @ Rz(th).T + ca + t
        d, i = tb.query(P); m = d < np.percentile(d, 80)
        X = P[m, :2] - P[m, :2].mean(0); Y = B[i[m], :2] - B[i[m], :2].mean(0)
        H = X.T @ Y; th += math.atan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1])
        P = (A - ca) @ Rz(th).T + ca + t
        d, i = tb.query(P); m = d < np.percentile(d, 80)
        t[:2] += (B[i[m], :2] - P[m, :2]).mean(0)
    P = (A - ca) @ Rz(th).T + ca + t; d, _ = tb.query(P)
    fit = float((d < 0.01).mean()); rmse = float(np.sqrt(np.mean(d[d < 0.02] ** 2))) if (d < 0.02).any() else 1.0
    if best is None or fit > best[0]:
        best = (fit, rmse, th, t.copy())
fit, rmse, th, t = best
zA = np.ptp(np.percentile(A[:, 2], [2, 98])); zB = np.ptp(np.percentile(B[:, 2], [2, 98]))
gates = dict(fitness=round(fit, 3), rmse_mm=round(rmse * 1000, 1), yaw_deg=round(math.degrees(th), 2),
             t_mm=[round(v * 1000, 1) for v in t[:2]], z_extent_diff_mm=round(abs(zA - zB) * 1000, 1))
ok = fit > 0.6 and rmse < 0.012 and abs(zA - zB) < 0.015
print('SE2 ICP', gates, 'RIGID' if ok else 'NOT RIGID -> regenerate')
if not ok:
    sys.exit(2)
src = json.load(open(a.anchor_input))
R = np.asarray(src['R'], float)
out = {'R': (Rz(th) @ R).tolist(), 'pick': ((np.asarray(src['pick']) - ca) @ Rz(th).T + ca + t).tolist(),
       'cloud': B.tolist(), 'run': f"reuse of {src.get('run')} via SE2 ICP {gates}"}
json.dump(out, open(a.out_prefix + '_input.json', 'w'))
plan = gp.plan_grasp(out, gp.load_mesh(a.mesh_url), depth=a.depth, cart_step=0.15, wp_min_time=6)
json.dump(plan, open(a.out_prefix + '_plan.json', 'w'), indent=1)
print('pick', np.round(out['pick'], 4).tolist(), 'hand pos', plan['grasp_pos'], 'flip', plan['flip_deg'])
print('grasp_hand', plan['checks']['grasp_hand'])
print('min joint margin', min(v['joint_limit_margin_rad'] for k, v in plan['checks'].items() if k != 'grasp_hand'))
print('PROBLEMS:', plan['problems'] if plan['problems'] else 'none')
