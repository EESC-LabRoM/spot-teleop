#!/usr/bin/env python3
"""Offline model of the MPC grasp attraction with the distance measured from the HAND (2026-11-06, Murilo:
"distance relative to the hand, but the correction is added to the goal ... an attraction, not taking over").

  goal  = op + w_p(d_hand) * (g - op)          (orientation: slerp by w_r(d_hand))
  d_hand = |hand - g| + k_rot * angle(hand, g)
  w_p = pf_max_weight * smoothstep(radius -> full), w_r = pf_max_weight_rot * smoothstep(...), rate-limited.

1) statics: hand == goal at equilibrium -> r_h = (1 - w_p(d(r_h))) * r_o. Counts the equilibria for every r_o (and
   orientation offset); more than one = the hand can snap between them.
2) replay: the logged pad goal of the 22:00-22:01Z run (MPC csv wrist_target) drives a first-order arm (tau) and the
   new field; reports the closest approach with the stick as it was and the max hand speed (no jumps).
  python3 pf_hand_model.py [--csv regrasp/state/run_2201.csv]
"""
import argparse
import csv

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument('--csv', default='/home/nexus/spot-teleop/fr3_real_repro/regrasp/state/run_2201.csv')
ap.add_argument('--radius', type=float, default=0.40)
ap.add_argument('--full', type=float, default=0.05)
ap.add_argument('--wmax', type=float, default=0.65)
ap.add_argument('--wmax-rot', type=float, default=0.85)
ap.add_argument('--k-rot', type=float, default=0.03)
ap.add_argument('--rate', type=float, default=1.5, help='max dw/dt (1/s)')
ap.add_argument('--tau', type=float, default=0.15, help='arm first-order time constant (s)')
a = ap.parse_args()

G = np.array([0.3677, -0.0611, 0.3139])


def smooth(d):
    s = np.clip((a.radius - d) / (a.radius - a.full), 0.0, 1.0)
    return s * s * (3 - 2 * s)


# 1) statics
worst, n_multi = 0, 0
rh = np.linspace(0.0, 0.6, 6001)
for ang_o in np.radians([0, 20, 47]):
    for r_o in np.linspace(0.0, 0.6, 601):
        # orientation equilibrium for a given w_r: ang_h = (1 - w_r) * ang_o
        def f(r):
            d = r + a.k_rot * ang_o * (1 - a.wmax_rot * smooth(r))   # first-order: ang uses the same s
            return (1 - a.wmax * smooth(d)) * r_o - r
        v = f(rh)
        roots = rh[np.where(np.sign(v[:-1]) != np.sign(v[1:]))[0]]
        n = 1 + int(np.sum(np.diff(roots) > 0.005)) if len(roots) else 0   # roots < 5 mm apart = one (grid touch)
        worst = max(worst, n)
        n_multi += n > 1
print(f'statics: max equilibria per pad-goal distance = {worst} ({n_multi} multi-solution cases over 3x601) '
      f'-> {"OK, no snapping" if worst <= 1 else "SNAPS"}')
for r_o in (0.10, 0.20, 0.30, 0.40):
    v = (1 - a.wmax * smooth(rh)) * r_o - rh
    r = rh[np.argmin(np.abs(v))]
    print(f'  stick still, pad goal {r_o*100:4.0f} cm from grasp -> hand settles {r*100:4.1f} cm from it')

# 2) replay of the logged pad goal
rows = [r for r in csv.reader(open(a.csv)) if r[5] not in ('', 'nan')]
t = np.array([np.datetime64(r[0]).astype('datetime64[ms]').astype(float) / 1000 for r in rows])
op = np.array([[float(r[5]), float(r[6]), float(r[7])] for r in rows])
x, w, best, vmax, prev_t = op[0].copy(), 0.0, (9, 0), 0.0, t[0]
for i in range(1, len(t)):
    dt = t[i] - prev_t; prev_t = t[i]
    if dt <= 0:
        continue
    wt = a.wmax * smooth(np.linalg.norm(x - G))
    w += np.clip(wt - w, -a.rate * dt, a.rate * dt)
    goal = op[i] + w * (G - op[i])
    nx = x + (goal - x) * (1 - np.exp(-dt / a.tau))
    vmax = max(vmax, np.linalg.norm(nx - x) / dt)
    x = nx
    d = np.linalg.norm(x - G)
    if d < best[0]:
        best = (d, t[i] - t[0], np.linalg.norm(op[i] - G), w)
print(f'replay {len(t)} samples: closest hand->grasp {best[0]*100:.1f} cm at +{best[1]:.1f} s '
      f'(pad goal then {best[2]*100:.1f} cm, w={best[3]:.2f}); max hand speed {vmax*100:.1f} cm/s')
