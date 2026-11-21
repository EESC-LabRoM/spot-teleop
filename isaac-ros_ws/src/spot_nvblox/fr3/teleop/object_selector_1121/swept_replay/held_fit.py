"""Offline tighter held-object sphere model for a grasped drill (no ROS/robot).

Constraints, all evaluated in world (fr3_link0) at the grasp pose (end of S2,
where the hand closes and the object pose in fr3_hand is fixed):

  C1 coverage   every observed object point lies inside a sphere with an
                inward clearance m_eff(p) = clip(p_z - table_z, 0, MARGIN):
                the 12 mm v3 margin everywhere except toward the table,
                where no object can exist. Pass bar: >= 98 % (live sync bar)
                with margin, 100 % without.
  C2 table      no sphere extends below table_z at the grasp (tol 2 mm).
                The drill stands on the table, so the support column from
                the observed bottom band down to the table is added as
                required coverage (it is real but occluded).
  C3 lift       after S3 the held spheres clear the table by >= the MPC
                activation distance (0.05 m).
  C4 lateral    report overhang of the model beyond the XY convex hull of
                the observed cloud (max / p95), the quantity to reduce.

Fitting: greedy weighted set cover over a 1.5 cm grid of centres and 5 mm
radius steps, gain = newly covered points - lam * overhang_mm, then a polish
that shrinks each radius to what its own points need. Swept over lam and
sphere count. Output spheres are converted to fr3_hand like the live
attached_spheres/<run>.json files.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
from scipy.spatial import ConvexHull, cKDTree

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from fr3_spheres import FR3Spheres  # noqa: E402

MARGIN = 0.012            # live v3 margin_m
R_CAP = 0.12              # live v3 cap
COVER_BAR = 0.98          # live held_model_sync bar
TABLE_TOL = 0.002
ACTIVATION = 0.05         # node_config/mpc_override.yml collision activation
CLEAN_ABOVE_TABLE = 0.003 # points at/below this are table returns, not drill
SUPPORT_BAND = 0.02       # bottom band projected down to the table
SUPPORT_STEP = 0.005
FIT_VOXEL = 0.008
GRID = 0.015
R_STEPS = np.arange(0.02, R_CAP + 1e-9, 0.005)

_K = FR3Spheres()


def hand_T(q):
    return _K.link_poses(np.asarray(q, float))['fr3_hand']


def grasp_q(plan):
    return np.asarray(plan['stages']['S2_pre_to_grasp']['q'][-1], float)


def lift_q(plan):
    return np.asarray(plan['stages']['S3_lift']['q'][-1], float)


def to_world(spheres_hand, T):
    S = np.asarray(spheres_hand, float)
    return np.c_[S[:, :3] @ T[:3, :3].T + T[:3, 3], S[:, 3]]


def to_hand(spheres_world, T):
    S = np.asarray(spheres_world, float)
    return np.c_[(S[:, :3] - T[:3, 3]) @ T[:3, :3], S[:, 3]]


def prepare(cloud_world, table_z):
    """Drop table returns; add the occluded support column down to the table."""
    P = np.asarray(cloud_world, float)
    P = P[P[:, 2] > table_z + CLEAN_ABOVE_TABLE]
    band = P[P[:, 2] < P[:, 2].min() + SUPPORT_BAND]
    zs = np.arange(table_z + 0.0005, P[:, 2].min(), SUPPORT_STEP)
    sup = np.vstack([np.c_[band[:, :2], np.full(len(band), z)] for z in zs]) if len(zs) else np.zeros((0, 3))
    return P, sup


def m_eff(P, table_z, margin=MARGIN):
    return np.clip(P[:, 2] - table_z, 0.0, margin)


def _hull_xy(P):
    return ConvexHull(P[:, :2])


def signed_dist_xy(xy, hull):
    """Signed distance to the convex polygon (negative inside)."""
    eq = hull.equations                     # a x + b y + c <= 0 inside, (a,b) unit
    d = xy @ eq[:, :2].T + eq[:, 2]
    out = d.max(1)
    pts = hull.points[hull.vertices]
    outside = out > 0
    if outside.any():                       # exact Euclidean distance outside
        q = xy[outside]
        best = np.full(len(q), np.inf)
        n = len(pts)
        for i in range(n):
            a, b = pts[i], pts[(i + 1) % n]
            ab = b - a
            t = np.clip(((q - a) @ ab) / (ab @ ab), 0, 1)
            best = np.minimum(best, np.linalg.norm(q - (a + t[:, None] * ab), axis=1))
        out = out.copy()
        out[outside] = best
    return out


def sphere_overhang(S, hull):
    """Per-sphere lateral overhang beyond the cloud XY hull (m, >= 0)."""
    return np.maximum(0.0, signed_dist_xy(S[:, :2], hull) + S[:, 3])


def evaluate(spheres_hand, plan, cloud_world, *, samples=None, margin=MARGIN, floor_pen=0.0):
    """All constraint metrics for a hand-frame sphere set."""
    table_z = float(plan['table_z'])
    Tg = hand_T(grasp_q(plan))
    Sw = to_world(spheres_hand, Tg)
    P, sup = prepare(cloud_world, table_z)
    D = np.linalg.norm(P[:, None, :] - Sw[None, :, :3], axis=2)
    inside = (D <= Sw[None, :, 3]).any(1)
    with_m = (D <= Sw[None, :, 3] - need_clearance(P, table_z, floor_pen, margin)[:, None]).any(1)
    with_12 = (D <= Sw[None, :, 3] - MARGIN).any(1)        # plain radial 12 mm, ignores table
    gap = np.min(D - Sw[None, :, 3], axis=1)
    sup_in = (np.linalg.norm(sup[:, None, :] - Sw[None, :, :3], axis=2) <= Sw[None, :, 3]).any(1) \
        if len(sup) else np.ones(0, bool)
    hull = _hull_xy(P)
    oh = sphere_overhang(Sw, hull)
    # held clearance over the whole S3 lift (joint-linear samples, like replay)
    from replay_plan import sample_stage
    s3, _ = sample_stage(grasp_q(plan), plan['stages']['S3_lift']['q'], 0.01)
    lift_clear = [float(np.min(to_world(spheres_hand, hand_T(q))[:, 2]
                               - np.asarray(spheres_hand)[:, 3]) - table_z) for q in s3]
    rng = np.random.default_rng(0)
    box_lo, box_hi = (Sw[:, :3] - Sw[:, 3:]).min(0), (Sw[:, :3] + Sw[:, 3:]).max(0)
    X = rng.uniform(box_lo, box_hi, size=(samples or 60000, 3))
    vol = float(np.prod(box_hi - box_lo) *
                (np.linalg.norm(X[:, None, :] - Sw[None, :, :3], axis=2) <= Sw[None, :, 3]).any(1).mean())
    return dict(
        n_spheres=int(len(Sw)),
        coverage_margin=round(float(with_m.mean()), 4),
        coverage_inside=round(float(inside.mean()), 4),
        coverage_radial12=round(float(with_12.mean()), 4), margin_mm=round(margin * 1e3, 1),
        floor_pen_mm=round(floor_pen * 1e3, 1),
        worst_point_outside_mm=round(float(max(gap.max(), 0)) * 1e3, 1),
        support_coverage=round(float(sup_in.mean()) if len(sup) else 1.0, 4),
        base_band_uncovered_mm=round(float((sup[~sup_in][:, 2].max() - table_z) * 1e3), 1)
        if len(sup) and (~sup_in).any() else 0.0,
        grasp_floor_mm=round(float(np.min(Sw[:, 2] - Sw[:, 3]) - table_z) * 1e3, 1),
        lift_floor_min_mm=round(min(lift_clear) * 1e3, 1),
        lift_floor_end_mm=round(lift_clear[-1] * 1e3, 1),
        lateral_overhang_max_mm=round(float(oh.max()) * 1e3, 1),
        lateral_overhang_sum_mm=round(float(oh.sum()) * 1e3, 1),
        volume_l=round(vol * 1e3, 2),
        n_points=int(len(P)), n_support=int(len(sup)),
    )


def passes(m, support_required=True):
    reasons = []
    if m['coverage_margin'] < COVER_BAR:
        reasons.append(f"coverage {m['coverage_margin']} < {COVER_BAR}")
    if m['coverage_inside'] < 1.0:
        reasons.append(f"points outside model (worst {m['worst_point_outside_mm']} mm)")
    if support_required and m['support_coverage'] < 1.0:
        reasons.append(f"occluded support column covered {m['support_coverage']:.2f}")
    if m['grasp_floor_mm'] < -TABLE_TOL * 1e3:
        reasons.append(f"below table at grasp by {-m['grasp_floor_mm']} mm")
    if m['lift_floor_end_mm'] < ACTIVATION * 1e3:
        reasons.append(f"held model within activation after lift ({m['lift_floor_end_mm']} mm)")
    return not reasons, reasons


def _voxel_down(P, v):
    if len(P) == 0:
        return P
    _, idx = np.unique(np.floor(P / v).astype(int), axis=0, return_index=True)
    return P[np.sort(idx)]


def need_clearance(P, table_z, floor_pen=0.0, margin=MARGIN):
    """Inward clearance demanded at each point: the margin, but never more
    than the room down to the allowed floor (table_z - floor_pen). Spheres
    cannot place margin under a solid table."""
    return np.minimum(margin, np.maximum(P[:, 2] - (table_z - floor_pen), 0.0))


def _build(plan, cloud_world, floor_pen, margin, voxel, grid):
    table_z = float(plan['table_z'])
    P, _ = prepare(cloud_world, table_z)
    hull = _hull_xy(P)
    Q = _voxel_down(P, voxel)
    # voxel representatives stand for points up to voxel/2 away
    need = np.minimum(need_clearance(Q, table_z, floor_pen, margin) + voxel / 2,
                      np.maximum(Q[:, 2] - (table_z - floor_pen), 0.0))
    lo, hi = Q.min(0), Q.max(0)
    axes = [np.arange(lo[i], hi[i] + grid / 2, grid) for i in range(3)]
    C = np.stack(np.meshgrid(*axes, indexing='ij'), -1).reshape(-1, 3)
    tree = cKDTree(Q)
    rows, cols, cand = [], [], []
    for r in np.arange(0.02, R_CAP + 1e-9, 0.01):
        for c in C[C[:, 2] - r >= table_z - floor_pen - 1e-9]:
            idx = np.asarray(tree.query_ball_point(c, r))
            if len(idx) == 0:
                continue
            idx = idx[np.linalg.norm(Q[idx] - c, axis=1) <= r - need[idx]]
            if len(idx) < 2:
                continue
            rows += idx.tolist()
            cols += [len(cand)] * len(idx)
            cand.append((*c, r))
    cand = np.array(cand)
    over = np.maximum(0, signed_dist_xy(cand[:, :2], hull) + cand[:, 3]) * 1e3
    return Q, cand, rows, cols, over


def fit(plan, cloud_world, n_max=4, floor_pen=0.0, *, margin=MARGIN, voxel=FIT_VOXEL,
        grid=0.02, time_limit=45.0):
    """Two-stage MILP at the grasp pose; returns (hand-frame spheres, info).

    Stage 1 maximises the fraction of observed points covered with
    need_clearance (n_max spheres, every sphere above table_z - floor_pen).
    Stage 2 keeps that coverage (-0.2 % of points) and minimises total
    lateral overhang (mm) + a volume proxy.
    """
    from scipy.optimize import Bounds, LinearConstraint, milp
    from scipy.sparse import csr_matrix, hstack, identity

    Q, cand, rows, cols, over = _build(plan, cloud_world, floor_pen, margin, voxel, grid)
    nc, nq = len(cand), len(Q)
    A = csr_matrix((np.ones(len(rows)), (rows, cols)), shape=(nq, nc))
    cover = LinearConstraint(hstack([A, identity(nq)]), lb=1, ub=np.inf)
    count = LinearConstraint(hstack([csr_matrix(np.ones((1, nc))), csr_matrix((1, nq))]), lb=1, ub=n_max)
    integ = np.r_[np.ones(nc), np.zeros(nq)]
    bounds = Bounds(np.zeros(nc + nq), np.ones(nc + nq))
    opts = dict(time_limit=time_limit, mip_rel_gap=0.01)
    r1 = milp(np.r_[np.zeros(nc), np.ones(nq)], constraints=[cover, count],
              integrality=integ, bounds=bounds, options=opts)
    info = dict(stage1=str(r1.message)[:70], candidates=int(nc), points=int(nq),
                floor_pen_mm=round(floor_pen * 1e3, 1), margin_mm=round(margin * 1e3, 1))
    if r1.x is None:
        return None, info
    unc = float(r1.x[nc:].sum())
    info['stage1_cover_fraction'] = round(1 - unc / nq, 4)
    keep = LinearConstraint(hstack([csr_matrix((1, nc)), csr_matrix(np.ones((1, nq)))]),
                            lb=0, ub=unc + max(1.0, 0.002 * nq))
    cost = over + 4.19 * (cand[:, 3] * 100) ** 3 / 10
    r2 = milp(np.r_[cost, np.zeros(nq)], constraints=[cover, count, keep],
              integrality=integ, bounds=bounds, options=opts)
    info['stage2'] = str(r2.message)[:70]
    x = r2.x if r2.x is not None else r1.x
    S = cand[np.nonzero(x[:nc] > 0.5)[0]]
    return to_hand(S, hand_T(grasp_q(plan))), info


def load_run(name):
    root = HERE.parent
    if name.endswith('053830_drill_loop_regen'):
        d = HERE / 'ref' / name
        return (json.loads((d / 'plan.json').read_text()), np.load(d / 'cloud.npy'),
                np.array(json.loads((d / 'held.json').read_text())['spheres'], float))
    d = root / 'fit_inputs/grasp_loop/runs' / name
    held = json.loads((root / f'fit_inputs/regrasp/state/held_model_{name}.json').read_text())
    return json.loads((d / 'plan.json').read_text()), np.load(d / 'cloud.npy'), np.array(held['spheres'], float)


def sweep(name, ns=(4, 6), pens=(0.0, 0.010, 0.020), time_limit=45.0):
    plan, cloud, old = load_run(name)
    live = evaluate(old, plan, cloud)
    rows = [dict(model='live_v3', **live)]
    for pen in pens:
        for n in ns:
            S, info = fit(plan, cloud, n, pen, time_limit=time_limit)
            row = dict(model=f'milp_pen{int(round(pen * 1000))}_n{n}', fit=info)
            if S is not None:
                row.update(evaluate(S, plan, cloud, floor_pen=pen), spheres_hand=np.round(S, 4).tolist())
            rows.append(row)
    for r in rows:
        if 'coverage_inside' in r:
            r['pass'], r['reasons'] = passes(r)
        else:
            r['pass'], r['reasons'] = False, ['no solution: ' + r['fit']['stage1']]
    return rows


KEYS = ('n_spheres', 'floor_pen_mm', 'coverage_margin', 'coverage_radial12', 'coverage_inside',
        'worst_point_outside_mm', 'grasp_floor_mm', 'lift_floor_end_mm', 'lateral_overhang_max_mm',
        'lateral_overhang_sum_mm', 'volume_l')

if __name__ == '__main__':
    out = {}
    for run in ('20261121_053830_drill_loop_regen', '20261121_050322_drill_oneshot_regen'):
        rows = sweep(run)
        out[run] = rows
        for r in rows:
            print(run[9:15], r['model'], 'PASS' if r['pass'] else 'fail',
                  {k: r.get(k) for k in KEYS}, r['reasons'][:3], flush=True)
    (HERE / 'held_fit_results.json').write_text(json.dumps(out, indent=1))
