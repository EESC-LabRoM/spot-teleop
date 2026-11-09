"""Candidate gating + ranking + planning (Phase A select).

Extent -> density (fixed TCP) -> rank -> contracts gates -> plan_grasp per
candidate until PROBLEMS none. The planner is IMPORTED (grasp_plan_lib, the
same code grasp_test.sh runs), never copied. Proven live values are baked
in as defaults: sparse single-waypoint timing, 10 cm lift.
"""
import sys

import numpy as np

sys.path.insert(0, '/home/nexus/spot-teleop/fr3_real_repro/grasp_loop')

from grasp_loop import density_gate, quat_from_matrix, rank_grasps
from grasp_plan_lib import (QHI, QLO, TCP_OFFSET, PlanError,  # noqa: F401,E402
                            Rz, plan_grasp)

from contracts import (FLAT_CELL, FLAT_TOL, MAX_OFF_MM, MIN_SCORE,
                       MIN_VERT, PAD_PEAK_Q, PAD_RA, PAD_RX, PAD_SHELL,
                       TIP_MIN_CM, VERT_FALLBACK, VERT_MIN, WRAP_MM)

DEPTH_SCHEDULE = (0.01, 0.0, -0.01)
EXTRA_DEPTH = 0.02      # tried first when the finger channel allows it
WRAP_FOR_EXTRA_MM = 35.0  # 20 mm shift + 15 mm of remaining channel
CART_STEP = 0.15        # single-waypoint stages (11-05 S2 fix)
WP_MIN_TIME = 6.0
LIFT = 0.10


class NoFeasibleGrasp(Exception):
    pass


def plan_quality(plan):
    """(min joint margin to limits, S1 time) from a plan dict."""
    m = min(float(np.min(np.minimum(np.array(q) - QLO, QHI - np.array(q))))
            for st in plan['stages'].values() for q in st['q'])
    s1 = float(plan['stages']['S1_home_to_pre']['t'][-1])
    return m, s1


def rank_pool(G, Pc, cloud):
    """Extent + density + rank. Returns ([entries best-first], starved)."""
    G = np.asarray(G, float)
    Pc = np.asarray(Pc, float)[:, :3]
    C = np.array(cloud['centroid'])
    E = np.array(cloud.get('extent', [0.3, 0.3, 0.3]))
    lo, hi = C - E / 2 - 0.05, C + E / 2 + np.array([0.05, 0.05, 0.08])
    cand = [i for i in range(G.shape[0])
            if np.all(G[i, :16].reshape(4, 4)[:3, 3] >= lo)
            and np.all(G[i, :16].reshape(4, 4)[:3, 3] <= hi)]
    if not cand:
        return [], True
    cand, starved, _ = density_gate(G, Pc, cand)
    Cmed = np.median(Pc, axis=0)
    out = []
    for i, s, m, b in rank_grasps(G[np.array(cand)], Pc, Cmed[:2], E[:2],
                                  float(cloud['top'])):
        gi = cand[i]
        out.append(dict(
            idx=int(gi), score=float(s), conf=float(G[gi, 16]),
            metrics=dict(off_xy_mm=round(1000 * m['off_xy'], 1),
                         nnear=m['nnear'],
                         bilateral=round(m['bilateral'], 3),
                         wrap_mm=round(1000 * m['wrap'], 1),
                         vert=round(m['vert'], 3)),
            weights={k: round(float(v), 4) for k, v in b.items()}))
    return out, starved


def apply_gates(ranked):
    """Contracts bars on top of the ranker's own vert pre-gate."""
    keep = []
    for e in ranked:
        m = e['metrics']
        if e['score'] < MIN_SCORE:
            continue
        if m['vert'] < MIN_VERT:
            continue
        if not (WRAP_MM[0] <= m['wrap_mm'] <= WRAP_MM[1]):
            continue
        if m['off_xy_mm'] > MAX_OFF_MM:
            continue
        keep.append(e)
    return keep


def transform_grasps(G, yaw_deg, tx, ty):
    """Carry a grasp set by an SE(2) motion (the REUSE path)."""
    a = np.radians(yaw_deg)
    c, s = np.cos(a), np.sin(a)
    T = np.eye(4)
    T[:2, :2] = [[c, -s], [s, c]]
    T[:2, 3] = [tx, ty]
    G = np.asarray(G, float)
    out = G.copy()
    for i in range(G.shape[0]):
        out[i, :16] = (T @ G[i, :16].reshape(4, 4)).reshape(-1)
    return out


def _pad_flat(hand, side):
    """One pad's contact patch: hand coords (rx, ry, ra) relative to TCP.
    The patch is the outermost shell of the pad's side (first contact --
    the pad face meets the extreme surface first). Returns dict(n, cov,
    ang). Sides with <4 shell points fail soft (0.0, 90.0): ranking
    input, never a crash."""
    slab = hand[(np.abs(hand[:, 0]) < PAD_RX)
                & (hand[:, 2] >= PAD_RA[0]) & (hand[:, 2] <= PAD_RA[1])
                & (np.sign(hand[:, 1]) == side)]
    if len(slab) < 4:
        return dict(n=int(len(slab)), cov=0.0, ang=90.0)
    peak = float(np.quantile(slab[:, 1] * side, PAD_PEAK_Q))
    sel = slab[slab[:, 1] * side >= peak - PAD_SHELL]
    if len(sel) < 4:
        return dict(n=int(len(sel)), cov=0.0, ang=90.0)
    _, _, Vt = np.linalg.svd(sel - sel.mean(axis=0), full_matrices=False)
    n = Vt[-1]
    if np.dot(n, [0.0, side, 0.0]) < 0:
        n = -n  # pad-ward normal
    d_first = float(np.max(sel @ n))  # first-contact plane offset
    close = sel[d_first - sel @ n <= FLAT_TOL]
    cols = np.floor((close[:, 0] + PAD_RX) / FLAT_CELL).astype(int)
    rows = np.floor((close[:, 2] - PAD_RA[0]) / FLAT_CELL).astype(int)
    nc = int(np.ceil(2 * PAD_RX / FLAT_CELL))
    nr = int(np.ceil((PAD_RA[1] - PAD_RA[0]) / FLAT_CELL))
    ok = (cols >= 0) & (cols < nc) & (rows >= 0) & (rows < nr)
    cov = float(len(np.unique(cols[ok] * nr + rows[ok])) / (nc * nr))
    ang = float(np.degrees(np.arccos(
        min(1.0, abs(np.dot(n, [0.0, 1.0, 0.0]))))))
    return dict(n=int(len(sel)), cov=round(cov, 3), ang=round(ang, 1))


def flatness(G_row, Pc, depth):
    """Contact-patch flatness per pad at a (candidate, depth) pose.

    Hand origin = pick + approach*depth (the planner's p_grasp at
    z_offset 0), TCP 103.4 mm down the approach; coords relative to TCP
    in hand axes. The hand frame is Rg @ Rz(90 deg) exactly like the
    planner's _choose_flip (the 0/180 flip is skipped: pads swap and the
    min/max readouts are symmetric, so the metric is flip-invariant).
    Returns dict(cov=min pad coverage, ang=max pad angle, pads=[+,-])."""
    Rm = np.asarray(G_row[:16], float).reshape(4, 4)
    Rh = Rm[:3, :3] @ Rz(np.pi / 2)
    A = Rh[:3, 2] / np.linalg.norm(Rh[:3, 2])
    org = Rm[:3, 3] + A * depth
    tcp = org + A * TCP_OFFSET
    r = np.asarray(Pc, float)[:, :3] - tcp
    hand = np.column_stack([r @ Rh[:3, 0], r @ Rh[:3, 1], r @ A])
    pads = [_pad_flat(hand, s) for s in (+1, -1)]
    return dict(cov=min(p['cov'] for p in pads),
                ang=max(p['ang'] for p in pads), pads=pads)


def plan_candidate(G_row, Pc, mesh_V, run, depth, z_offset=0.0):
    Rm = np.asarray(G_row[:16], float).reshape(4, 4)
    g = dict(R=Rm[:3, :3].tolist(), pick=Rm[:3, 3].tolist(),
             cloud=np.asarray(Pc, float)[:, :3], run=run)
    return plan_grasp(g, np.asarray(mesh_V, float), depth=depth,
                      z_offset=z_offset, cart_step=CART_STEP,
                      wp_min_time=WP_MIN_TIME, lift=LIFT)


def pick_dict(G, entry):
    Rm = np.asarray(G[entry['idx'], :16], float).reshape(4, 4)
    pos = Rm[:3, 3]
    return dict(pos=[float(v) for v in pos],
                quat_xyzw=[float(v) for v in quat_from_matrix(Rm[:3, :3])],
                conf=entry['conf'], idx=entry['idx'], score=entry['score'],
                metrics=dict(entry['metrics']),
                weights=dict(entry['weights']))


def depths_for(entry, base=DEPTH_SCHEDULE):
    """Per-candidate depth sweep, deepest first. +0.02 only pays when the
    ranker's finger channel (wrap = TCP depth below the object top) leaves
    room for the 20 mm shift; otherwise it just burns a ~1 s plan."""
    if entry['metrics']['wrap_mm'] >= WRAP_FOR_EXTRA_MM:
        return (EXTRA_DEPTH,) + tuple(base)
    return tuple(base)


def tip_ok(row):
    """Tip bar (coord contract): the higher tip must sit TIP_MIN_CM below
    the local top. NaN (no local top found) never passes."""
    tip = row['tip_cm']
    return tip == tip and tip >= TIP_MIN_CM


def vert_rungs(feas):
    """Tilt ladder (coord contract + measured last resort): rung 0 is
    <=15 deg, rung 1 is <=25 deg, rung 2 is everything else. select()
    plans rung by rung (top-k each) and stops at the first rung with a
    tip-passing clean plan. Rung 2 exists because the recorded lying2
    pool -- which holds the only fully-executed grasp, idx152 at 32 deg
    -- has NOTHING above 0.9063 except unreachable idx28: without it the
    rule yields NoFeasibleGrasp on lying pools. Vert still ranks above
    margin inside every rung."""
    strict = [e for e in feas if e['metrics']['vert'] >= VERT_MIN]
    mid = [e for e in feas
           if VERT_FALLBACK <= e['metrics']['vert'] < VERT_MIN]
    rest = [e for e in feas if e['metrics']['vert'] < VERT_FALLBACK]
    return [strict, mid, rest]


def winner_key(item):
    """Grip evidence outranks grasp score: among tip-passing clean plans,
    more pts_between_pads wins, then verticality, then contact-patch
    normal angle (smaller wins), then margin, then S1. Coverage is
    RECORDED, not ranked: on 9 replay samples it reads 0.02-0.22 with
    the held idx152 lowest (0.02) and the slipped 203025 highest (0.20)
    -- anti-predictive, and a 60% bar would reject everything. Angle
    instead orders held (23/47 deg) above grazes (52-76) above the
    sideways slip (87)."""
    return (-item['pts'], -item['vert'], item['flat_ang'],
            -item['margin'], item['s1_t'])


def select(G, Pc, cloud, mesh_V, run='loop', depths=DEPTH_SCHEDULE,
           top_k=6, z_offset=0.0):
    """Plan the vert ladder rung by rung (top-k candidates x per-candidate
    depths each); the winner is the min winner_key among PROBLEMS-free,
    tip-passing plans in the FIRST rung that has any. Stops early: the
    fallback rungs only run when nothing above yields. Returns (pick,
    plan, depth, ranked, tried) where tried is a list of dicts
    (auditable, rung-tagged). Raises NoFeasibleGrasp."""
    G = np.asarray(G, float)
    Pc = np.asarray(Pc, float)[:, :3]
    ranked, starved = rank_pool(G, Pc, cloud)
    feas = apply_gates(ranked)
    tried = []
    for rung, pool in enumerate(vert_rungs(feas)):
        clean = []
        for e in pool[:top_k]:
            for depth in depths_for(e, depths):
                try:
                    plan = plan_candidate(G[e['idx']], Pc, mesh_V, run,
                                          depth, z_offset)
                except PlanError as ex:
                    tried.append(dict(idx=e['idx'], depth=depth, rung=rung,
                                      problems=[f'IK: {ex}']))
                    continue
                probs = list(plan.get('problems', []))
                margin, s1 = plan_quality(plan)
                gh = plan['checks']['grasp_hand']
                fl = flatness(G[e['idx']], Pc, depth)
                row = dict(idx=e['idx'], depth=depth, rung=rung,
                           problems=probs, pts=gh['pts_between_pads'],
                           tip_cm=gh['higher_tip_below_top_cm'],
                           vert=e['metrics']['vert'],
                           flat_cov=fl['cov'], flat_ang=fl['ang'],
                           margin=round(margin, 3), s1_t=round(s1, 1),
                           score=e['score'])
                tried.append(row)
                if not probs and tip_ok(row):
                    clean.append((row, e, plan))
        if clean:
            break
    if not clean:
        raise NoFeasibleGrasp(
            f'{len(feas)} feasible, starved={starved}, tried={tried}')
    row, e, plan = min(clean, key=lambda c: winner_key(c[0]))
    pick = pick_dict(G, e)
    pick['n_total'] = int(G.shape[0])
    pick['source'] = 'regrasp select'
    return pick, plan, row['depth'], ranked, tried
