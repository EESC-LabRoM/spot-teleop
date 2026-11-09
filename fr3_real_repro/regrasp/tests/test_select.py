"""select.py: full gate+rank+plan on the recorded lying3 pool + mesh fixture.

Also: a far-away junk grasp must never be picked, and SE(2) grasp carry
must rotate positions rigidly.

Run: docker exec vm-viewer bash -c
  'source /opt/ros/humble/setup.bash && python3 <this file>'
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, '/home/nexus/spot-teleop/fr3_real_repro/grasp_loop')

import json
from selection import (NoFeasibleGrasp, apply_gates, depths_for, flatness,
                       rank_pool, select, tip_ok, transform_grasps,
                       vert_rungs, winner_key)
from grasp_plan_lib import Rz

RB = '/home/nexus/spot-teleop/fr3_real_repro/grasp_loop/runs'
R = f'{RB}/20261106_152112_drill_lying3'
G = np.load(f'{R}/grasps.npy').astype(float)
Pc = np.load(f'{R}/cloud.npy')[:, :3].astype(float)
cloud = json.load(open(f'{R}/cloud.json'))
MESH = np.load(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            'data_mesh_test.npy'))


def check(cond, msg):
    if not cond:
        raise AssertionError(msg)
    print(f"ok: {msg}")


ranked, starved = rank_pool(G, Pc, cloud)
check(not starved and len(ranked) == 28, f"pool ranks ({len(ranked)})")
feas = apply_gates(ranked)
check(0 < len(feas) <= len(ranked), f"gates keep {len(feas)}/{len(ranked)}")
junk = np.eye(4).reshape(-1).tolist() + [0.99]
Gj = np.vstack([G, np.array([junk[:16] + [0.99]], float)])
Gj[-1, 3], Gj[-1, 7], Gj[-1, 11] = 2.0, 2.0, 2.0
ranked_j, _ = rank_pool(Gj, Pc, cloud)
check(all(e['idx'] != 200 for e in ranked_j), "far junk never ranks")

R2 = ('/home/nexus/spot-teleop/fr3_real_repro/grasp_loop/runs/'
      '20261106_150037_drill_lying2')
G2 = np.load(f'{R2}/grasps.npy').astype(float)
Pc2 = np.load(f'{R2}/cloud.npy')[:, :3].astype(float)
cloud2 = json.load(open(f'{R2}/cloud.json'))
pick, plan, depth, _, tried = select(G2, Pc2, cloud2, MESH, run='test',
                                     top_k=6)
check(plan.get('problems') == [], f"select plans clean ({pick['idx']})")
check(pick['metrics']['off_xy_mm'] <= 150, "pick respects off gate")
check(any(r.get('depth') == 0.02 for r in tried),
      "extra +0.02 attempted on deep-wrap candidates")
clean_rows = [r for r in tried
              if r.get('problems') == [] and tip_ok(r)]
win = next(r for r in clean_rows if r['idx'] == pick['idx']
           and r['depth'] == depth)
first_rung = min(r['rung'] for r in clean_rows)
check(win['rung'] == first_rung == 2,
      f"lying2 falls to rung 2 (vert-poor pool), winner rung {win['rung']}")
same = [r for r in clean_rows if r['rung'] == first_rung]
check(all(winner_key(win) <= winner_key(r) for r in same),
      f"winner is min winner_key over {len(same)} rung-{first_rung} clean")
check(win['tip_cm'] >= 3.0, f"winner clears the tip bar ({win['tip_cm']})")
check(win['pts'] >= 400, f"winner holds deep contact ({win['pts']} pts)")
check('hand_R' in plan and 'grasp_pos' in plan, "plan feeds /grasp_target")

# Replay flatness on executed grasps (values reported, ranges asserted).
fm = flatness(G2[pick['idx']], Pc2, depth)
print(f"replay lying2 idx{pick['idx']}: cov {fm['cov']:.2f} "
      f"ang {fm['ang']:.1f}")
for RR, xdepth in ((f'{RB}/20261106_203025_drill_loop_regen', None),
                   (f'{RB}/20261106_205015_drill_loop_regen', None),
                   (f'{RB}/20261106_205015_drill_loop_regen', 0.02)):
    check(os.path.isdir(RR), f"replay run present ({RR[-22:-16]})")
    rp = json.load(open(f'{RR}/pick.json'))
    rl = json.load(open(f'{RR}/loop.json'))
    rg = np.load(f'{RR}/grasps.npy').astype(float)
    rc = np.load(f'{RR}/cloud.npy')[:, :3].astype(float)
    d = rl['depth'] if xdepth is None else xdepth
    fm = flatness(rg[rp['idx']], rc, d)
    print(f"replay {RR[-22:]} idx{rp['idx']}@{d}: cov {fm['cov']:.2f} "
          f"ang {fm['ang']:.1f}")
    check(0.0 <= fm['cov'] <= 1.0 and 0.0 <= fm['ang'] <= 90.0,
          f"flatness in range ({RR[-22:-16]}@{d})")
try:
    select(G, Pc, cloud, MESH, run='test', top_k=2)
    check(False, "lying3 top-2 must raise")
except NoFeasibleGrasp as e:
    check('tried=' in str(e), "graceful failure carries tried list")

# Winner rule (coord contract): tip bar first (higher tip >= 3.0 cm under
# the top; the slip was 1.8-2.0), then more pts_between_pads, then
# verticality, then margin, then faster S1. Candidates pre-filter to
# <=15 deg tilt, <=25 deg fallback when nothing strict survives.
it = lambda pts, tip, vert, flat, ang, margin, s1: dict(
    pts=pts, tip_cm=tip, vert=vert, flat_cov=flat, flat_ang=ang,
    margin=margin, s1_t=s1)
check(tip_ok(it(411, 3.0, 0.97, 0.9, 5.0, 0.68, 10.5))
      and not tip_ok(it(999, 2.9, 0.99, 0.9, 5.0, 0.90, 8.0))
      and not tip_ok(it(999, float('nan'), 0.99, 0.9, 5.0, 0.90, 8.0)),
      "tip bar passes 3.0, rejects 2.9 and NaN")
check(winner_key(it(411, 3.8, 0.97, 0.9, 5.0, 0.68, 10.5))
      < winner_key(it(177, 3.8, 0.99, 0.9, 5.0, 0.90, 8.0)),
      "more pts beats better vert+margin")
check(winner_key(it(411, 3.8, 0.99, 0.5, 25.0, 0.50, 26.0))
      < winner_key(it(411, 3.8, 0.97, 0.9, 5.0, 0.90, 8.0)),
      "vert breaks the pts tie above flat+margin")
check(winner_key(it(411, 3.8, 0.97, 0.02, 23.0, 0.50, 26.0))
      < winner_key(it(411, 3.8, 0.97, 0.22, 87.0, 0.90, 8.0)),
      "flat angle breaks the pts+vert tie (coverage ignored: the held "
      "idx152 reads cov 0.02, the slip 0.20)")
check(winner_key(it(411, 3.8, 0.97, 0.9, 5.0, 0.90, 26.0))
      < winner_key(it(411, 3.8, 0.97, 0.1, 5.0, 0.68, 8.0)),
      "margin breaks the pts+vert+angle tie")
check(winner_key(it(411, 3.8, 0.97, 0.9, 5.0, 0.68, 8.0))
      < winner_key(it(411, 3.8, 0.97, 0.9, 5.0, 0.68, 10.5)),
      "faster S1 breaks the full tie")
mk = lambda idx, vert: dict(idx=idx, metrics=dict(vert=vert))
r0, r1, r2 = vert_rungs([mk(1, 0.97), mk(2, 0.85), mk(3, 0.966),
                         mk(4, 0.92), mk(5, 0.9063)])
check([e['idx'] for e in r0] == [1, 3], "rung 0 keeps <=15 deg")
check([e['idx'] for e in r1] == [4, 5], "rung 1 keeps <=25 deg")
check([e['idx'] for e in r2] == [2], "rung 2 keeps the rest")
check(depths_for(dict(metrics=dict(wrap_mm=68.8)))
      == (0.02, 0.01, 0.0, -0.01),
      "+0.02 tried when the wrap allows")
check(depths_for(dict(metrics=dict(wrap_mm=23.1)))
      == (0.01, 0.0, -0.01),
      "no +0.02 on a shallow wrap")
check(depths_for(dict(metrics=dict(wrap_mm=35.0)))[0] == 0.02
      and depths_for(dict(metrics=dict(wrap_mm=34.9)))[0] == 0.01,
      "+0.02 boundary at 35 mm of wrap")

# Contact-patch flatness: synthetic grasp R = Rz(-90 deg), so the HAND
# frame (Rg @ Rz(90), like the planner) is identity: X width, Y closing,
# Z approach. Origin pick, depth 0 -> TCP = origin + 0.1034 Z.
# Pad faces at |ry| ~ 42 mm.
def _grow(Rm):
    g = np.zeros(17)
    g[:16] = np.asarray(Rm, float).reshape(-1)
    g[16] = 0.9
    return g


def _Rz4(deg):
    M = np.eye(4)
    M[:3, :3] = Rz(np.radians(deg))
    return M


def _pad_pts(side, tilt_deg=0.0, quadrant=False, n_ry=9):
    rng = np.random.default_rng(0)
    rxs = np.linspace(-0.010, 0.010, 5)
    ras = np.linspace(-0.044, -0.001, n_ry)
    if quadrant:
        rxs = rxs[2:]
        ras = ras[4:]
    P = []
    for rx in rxs:
        for ra in ras:
            ry = side * (0.042 + rng.normal(0, 0.001))
            if tilt_deg:
                a = np.radians(tilt_deg)
                dy, dz = ry - side * 0.042, ra + 0.0225
                ry = side * 0.042 + dy * np.cos(a) - dz * np.sin(a)
                ra = -0.0225 + dy * np.sin(a) + dz * np.cos(a)
            P.append([rx, ry, 0.1034 + ra])
    return np.array(P)


g0 = _grow(_Rz4(-90.0))
both = np.vstack([_pad_pts(+1), _pad_pts(-1)])
f = flatness(g0, both, 0.0)
check(f['cov'] >= 0.95 and f['ang'] < 5.0,
      f"full flat patch: cov {f['cov']:.2f} ang {f['ang']:.1f}")
tilt = np.vstack([_pad_pts(+1, tilt_deg=30.0), _pad_pts(-1)])
f = flatness(g0, tilt, 0.0)
# Shell truncation on ry cuts tilted patches obliquely, biasing the fit
# ~3 deg toward flat. Documented, not fixed: the metric is exact where
# decisions happen (near-flat candidates cut along their normal).
check(23.0 < f['ang'] < 30.0, f"30-deg patch reads ang {f['ang']:.1f}")
part = np.vstack([_pad_pts(+1, quadrant=True), _pad_pts(-1, quadrant=True)])
f = flatness(g0, part, 0.0)
check(0.2 < f['cov'] < 0.45, f"quarter patch reads cov {f['cov']:.2f}")
sparse = np.vstack([_pad_pts(+1), _pad_pts(-1)[:2]])
f = flatness(g0, sparse, 0.0)
check(f['cov'] == 0.0 and f['ang'] == 90.0, "starved pad fails soft")
ff = flatness(_grow(_Rz4(90.0)), both, 0.0)
check(abs(ff['cov'] - flatness(g0, both, 0.0)['cov']) < 1e-9
      and abs(ff['ang'] - flatness(g0, both, 0.0)['ang']) < 1e-9,
      "flatness is flip-invariant")

T = transform_grasps(G[:4], 90.0, 0.1, -0.2)
p0 = G[:4, :16].reshape(-1, 4, 4)[:, :3, 3]
p1 = T[:, :16].reshape(-1, 4, 4)[:, :3, 3]
c, s = np.cos(np.radians(90)), np.sin(np.radians(90))
want = p0 @ np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]]) + [0.1, -0.2, 0]
check(bool((abs(p1 - want) < 1e-9).all()), "SE(2) carry rotates positions")
check(bool((T[:, 16] == G[:4, 16]).all()), "carry keeps confidence")
print("37/37 select checks passed")
