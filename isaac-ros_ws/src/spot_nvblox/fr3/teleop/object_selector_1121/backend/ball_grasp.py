"""Offline tennis-ball geometry + grasp-pose validation (no ROS, no robot).

Why a separate rule set: the drill rules (TIP_MIN_CM=3 cm below the local
top, flatness, vert ladder) encode a tall object held by its body. A sphere
held by parallel pads is only stable when the pads straddle its EQUATOR;
pinching above it squeezes the ball up and out. For r~35 mm that means the
TCP must sit >= r below the top, i.e. the ball rule is STRICTER than
TIP_MIN, never a relaxation of it.

Hand-frame convention copied from canonical/selection.py::flatness so a
GraspGen row replays identically: Rh = R @ Rz(90deg), X = pad width,
Y = closing, A = approach; hand origin = pick + A*depth; TCP = origin +
A*TCP_OFFSET; pad contact band is TCP-relative approach (-45 mm, 0).

Every tolerance here is either a measured device constant (gripper open
width, pad footprint from contracts.py) or an explicit caller-supplied
error budget. Nothing defaults the error budget to zero.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

# --- device / contract constants (sources in comments) ---
TCP_OFFSET = 0.1034          # grasp_plan_lib TCP_OFFSET (103.4 mm)
OPEN_WIDTH = 0.0799          # measured open width 79.97 mm (handoff 11-21)
PAD_RX = 0.011               # contracts.PAD_RX pad half-width
PAD_RA = (-0.045, 0.0)       # contracts.PAD_RA contact band from TCP back
VERT_MIN = 0.966             # contracts.VERT_MIN, strict rung only (no fallback for ball)
MIN_PTS_BETWEEN_PADS = 50    # planner contract pts_between_pads >= 50
TIP_MIN = 0.030              # contracts.TIP_MIN_CM; kept as an extra floor
# --- ball model bars ---
# ITF ball diameter 65.4-68.6 mm -> r 32.7-34.3 mm; +-3 mm fit slack.
BALL_R = (0.0297, 0.0373)
FIT_RMS_MAX = 0.003
FIT_INLIER_TOL = 0.005
FIT_INLIER_MIN = 0.90
MIN_POINTS = 150
MIN_LATERAL_SPAN_R = 1.4     # cloud XY span must exceed 1.4 r (more than a cap)
CLUSTER_VOXEL = 0.01
CLUSTER_MAIN_MIN = 0.95
TABLE_CONTACT_TOL = 0.015    # |fitted bottom - table_z| for a resting ball
# --- clearance bars (new, conservative; require live evidence to change) ---
FINGERTIP_TABLE_CLEAR = 0.005
MIN_SIDE_CLEAR = 0.0         # after subtracting the lateral error budget
# Closed-width window for a held ball (felt compresses a few mm). Below
# this the gripper closed past the ball: slip / miss.
HELD_SQUEEZE_MAX = 0.015
HELD_OVER_MAX = 0.004


@dataclass(frozen=True)
class ErrorBudget:
    """Pose uncertainty the caller must justify from live evidence
    (broker drift, camera registration residual, table/fit mismatch)."""
    lateral_m: float
    vertical_m: float

    def __post_init__(self):
        for v in (self.lateral_m, self.vertical_m):
            if not (np.isfinite(v) and v > 0):
                raise ValueError('Error budget must be explicit and > 0')


@dataclass
class BallFit:
    ok: bool
    reasons: list
    center: np.ndarray = None
    radius: float = float('nan')
    rms: float = float('nan')
    inlier_frac: float = 0.0
    main_cluster_frac: float = 0.0
    n_clusters: int = 0
    table_offset: float = float('nan')   # fitted bottom - table_z
    top_observed: float = float('nan')

    def to_json(self):
        return dict(ok=self.ok, reasons=self.reasons,
                    center=None if self.center is None else np.round(self.center, 4).tolist(),
                    radius=round(self.radius, 4), rms=round(self.rms, 4),
                    inlier_frac=round(self.inlier_frac, 3),
                    main_cluster_frac=round(self.main_cluster_frac, 3),
                    n_clusters=self.n_clusters, table_offset=round(self.table_offset, 4),
                    top_observed=round(self.top_observed, 4))


def voxel_components(P, voxel=CLUSTER_VOXEL):
    """(main component fraction, n components) with 26-connected voxels."""
    keys = np.floor(np.asarray(P) / voxel).astype(np.int64)
    uniq, inv = np.unique(keys, axis=0, return_inverse=True)
    inv = np.asarray(inv).ravel()
    index = {tuple(k): i for i, k in enumerate(uniq)}
    label = -np.ones(len(uniq), int)
    offs = [(a, b, c) for a in (-1, 0, 1) for b in (-1, 0, 1) for c in (-1, 0, 1)]
    n = 0
    for s in range(len(uniq)):
        if label[s] >= 0:
            continue
        stack = [s]
        label[s] = n
        while stack:
            k = uniq[stack.pop()]
            for o in offs:
                j = index.get((k[0] + o[0], k[1] + o[1], k[2] + o[2]))
                if j is not None and label[j] < 0:
                    label[j] = n
                    stack.append(j)
        n += 1
    counts = np.bincount(label[inv])
    return float(counts.max() / len(P)), int(n)


def fit_sphere(P):
    A = np.c_[2 * P, np.ones(len(P))]
    b = (P ** 2).sum(1)
    x, *_ = np.linalg.lstsq(A, b, rcond=None)
    c = x[:3]
    r2 = x[3] + c @ c
    if not r2 > 0:
        raise ValueError('degenerate sphere fit')
    r = float(np.sqrt(r2))
    res = np.linalg.norm(P - c, axis=1) - r
    return c, r, res


def fit_ball(cloud, table_z) -> BallFit:
    P = np.asarray(cloud, float)
    if P.ndim != 2 or P.shape[1] != 3 or not np.isfinite(P).all():
        return BallFit(False, ['cloud malformed'])
    if len(P) < MIN_POINTS:
        return BallFit(False, [f'only {len(P)} points < {MIN_POINTS}'])
    if not np.isfinite(table_z):
        return BallFit(False, ['table_z unknown'])
    reasons = []
    main, ncomp = voxel_components(P)
    try:
        c, r, res = fit_sphere(P)
    except (ValueError, np.linalg.LinAlgError) as exc:
        return BallFit(False, [f'sphere fit failed: {exc}'])
    rms = float(np.sqrt((res ** 2).mean()))
    inl = float((np.abs(res) < FIT_INLIER_TOL).mean())
    off = float(c[2] - r - table_z)
    span = float(np.ptp(P[:, :2], axis=0).min())
    if main < CLUSTER_MAIN_MIN:
        reasons.append(f'cloud fragmented: main cluster {main:.2f} of {ncomp}')
    if not (BALL_R[0] <= r <= BALL_R[1]):
        reasons.append(f'radius {r * 1000:.1f} mm outside tennis-ball range')
    if rms > FIT_RMS_MAX:
        reasons.append(f'sphere rms {rms * 1000:.1f} mm > {FIT_RMS_MAX * 1000:.0f}')
    if inl < FIT_INLIER_MIN:
        reasons.append(f'inliers {inl:.2f} < {FIT_INLIER_MIN}')
    if span < MIN_LATERAL_SPAN_R * r:
        reasons.append('only a cap is visible; radius poorly constrained')
    if abs(off) > TABLE_CONTACT_TOL:
        reasons.append(f'fitted bottom {off * 1000:+.1f} mm from table; not resting or z misregistered')
    return BallFit(not reasons, reasons, c, r, rms, inl, main, ncomp, off,
                   float(P[:, 2].max()))


def Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


@dataclass(frozen=True)
class HandPose:
    tcp: np.ndarray
    X: np.ndarray      # pad width axis
    Y: np.ndarray      # closing axis
    A: np.ndarray      # approach axis

    @staticmethod
    def from_graspgen(row, depth):
        Rm = np.asarray(row[:16], float).reshape(4, 4)
        Rh = Rm[:3, :3] @ Rz(np.pi / 2)
        A = Rh[:, 2] / np.linalg.norm(Rh[:, 2])
        tcp = Rm[:3, 3] + A * (depth + TCP_OFFSET)
        return HandPose(tcp, Rh[:, 0], Rh[:, 1], A)

    @staticmethod
    def top_down(tcp, yaw=0.0, tilt=0.0):
        """Helper for tests: approach straight down, optional tilt about X."""
        R = Rz(yaw) @ np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1.0]])
        ct, st = np.cos(tilt), np.sin(tilt)
        R = R @ np.array([[1, 0, 0], [0, ct, -st], [0, st, ct]])
        return HandPose(np.asarray(tcp, float), R[:, 0], R[:, 1], R[:, 2])


def check_grasp(pose: HandPose, fit: BallFit, cloud, table_z, budget: ErrorBudget):
    """Return (ok, reasons, metrics) for one hand pose against a fitted ball."""
    if not fit.ok:
        return False, ['ball fit not valid'] + fit.reasons, {}
    c, r = fit.center, fit.radius
    d = c - pose.tcp
    dx, dy, da = float(d @ pose.X), float(d @ pose.Y), float(d @ pose.A)
    vert = float(-pose.A[2])
    # pad corners (fingertip ends at the TCP plane), open width
    half = OPEN_WIDTH / 2
    corners = [pose.tcp + sx * PAD_RX * pose.X + sy * half * pose.Y
               for sx in (-1, 1) for sy in (-1, 1)]
    tip_low = float(min(p[2] for p in corners))
    P = np.asarray(cloud, float) - pose.tcp
    h = np.c_[P @ pose.X, P @ pose.Y, P @ pose.A]
    between = int(((np.abs(h[:, 0]) < PAD_RX) & (np.abs(h[:, 1]) < half)
                   & (h[:, 2] >= PAD_RA[0]) & (h[:, 2] <= PAD_RA[1])).sum())
    side_clear = half - r - abs(dy) - budget.lateral_m
    m = dict(dx_mm=dx * 1e3, dy_mm=dy * 1e3, equator_above_tcp_mm=-da * 1e3,
             vert=vert, side_clear_mm=side_clear * 1e3,
             fingertip_table_mm=(tip_low - table_z) * 1e3, pts_between_pads=between,
             tip_below_top_mm=(fit.top_observed - pose.tcp[2]) * 1e3)
    m = {k: round(v, 2) if isinstance(v, float) else v for k, v in m.items()}
    reasons = []
    if vert < VERT_MIN:
        reasons.append(f'approach tilt vert {vert:.3f} < {VERT_MIN}')
    # equator (sphere center along approach) must lie inside the pad band
    # with the vertical error budget on both sides
    if not (PAD_RA[0] + budget.vertical_m <= da <= PAD_RA[1] - budget.vertical_m):
        reasons.append('pads do not straddle the ball equator with error budget')
    if abs(dx) + budget.lateral_m > PAD_RX:
        reasons.append('ball center off the pad width with lateral budget')
    if side_clear < MIN_SIDE_CLEAR:
        reasons.append(f'open fingers clear ball by {side_clear * 1e3:.1f} mm after budget')
    if tip_low - table_z < FINGERTIP_TABLE_CLEAR + budget.vertical_m:
        reasons.append('fingertips too close to table with vertical budget')
    if between < MIN_PTS_BETWEEN_PADS:
        reasons.append(f'{between} points between pads < {MIN_PTS_BETWEEN_PADS}')
    if fit.top_observed - pose.tcp[2] < TIP_MIN:
        reasons.append('tip above drill TIP_MIN floor')
    return not reasons, reasons, m


def feasible_tcp_window(fit: BallFit, table_z, budget: ErrorBudget):
    """Vertical top-down TCP z window [lo, hi]; empty if lo > hi."""
    lo = table_z + FINGERTIP_TABLE_CLEAR + budget.vertical_m
    hi = fit.center[2] - budget.vertical_m          # equator above TCP by budget
    hi = min(hi, fit.top_observed - TIP_MIN)
    return float(lo), float(hi)


def lateral_budget_capacity(fit: BallFit):
    """Max lateral error any centered grasp can absorb (open-width limited)."""
    return float(min(OPEN_WIDTH / 2 - fit.radius, PAD_RX))


def replay_graspgen(G, fit, cloud, table_z, budget, depths=(0.03, 0.01, 0.0, -0.01)):
    """Score every (row, depth); returns list of dicts, passing ones flagged."""
    out = []
    for i, row in enumerate(np.asarray(G, float)):
        for depth in depths:
            ok, reasons, m = check_grasp(HandPose.from_graspgen(row, depth), fit,
                                         cloud, table_z, budget)
            out.append(dict(idx=i, depth=depth, ok=ok, reasons=reasons, **m))
    return out


def held_width_ok(width_m, fit: BallFit):
    """Post-close check: a held ball pins the jaws near its diameter."""
    if not (fit.ok and np.isfinite(width_m)):
        return False
    dia = 2 * fit.radius
    return dia - HELD_SQUEEZE_MAX <= width_m <= dia + HELD_OVER_MAX


def held_model(fit: BallFit, hand_from_base, margin=0.012):
    """Single sphere in fr3_hand (centre transformed), r + margin."""
    T = np.asarray(hand_from_base, float)
    c = T[:3, :3] @ fit.center + T[:3, 3]
    return [[*np.round(c, 4).tolist(), round(fit.radius + margin, 4)]]


# --- transparent bottle: assessment only, never a grasp ---------------
SPRITE_MAX_Z_GAP = 0.02
SPRITE_MAX_FOOTPRINT = 0.12   # small bottle is <~7 cm wide; wider = depth smear


def assess_sprite_cloud(cloud):
    """Explain why a transparent-bottle depth cloud is not grasp evidence."""
    P = np.asarray(cloud, float)
    reasons = ['transparent PET: stereo depth is not a trustworthy surface']
    if P.ndim != 2 or P.shape[1] != 3 or len(P) < 10 or not np.isfinite(P).all():
        return dict(ok=False, reasons=reasons + ['cloud malformed'])
    main, n = voxel_components(P)
    z = np.sort(P[:, 2])
    gap = float(np.diff(z).max())
    if main < CLUSTER_MAIN_MIN:
        reasons.append(f'fragmented: main cluster {main:.2f} of {n} components')
    if gap > SPRITE_MAX_Z_GAP:
        reasons.append(f'vertical depth gap {gap * 1000:.0f} mm at z={z[np.diff(z).argmax()]:.3f}')
    foot = float(np.ptp(P[:, :2], axis=0).max())
    if foot > SPRITE_MAX_FOOTPRINT:
        reasons.append(f'footprint {foot * 1000:.0f} mm implausible for a small bottle (depth smear)')
    return dict(ok=False, reasons=reasons, main_cluster_frac=round(main, 3),
                n_clusters=n, max_z_gap_m=round(gap, 3),
                extent_m=np.round(np.ptp(P, 0), 3).tolist())


# --- evidence-derived error budget + end-to-end offline verdict ----------
def derive_budget(fits: dict, broker_drift_m: float) -> ErrorBudget:
    """Budget from measured disagreement, never from a hand-picked number.

    fits: camera name -> BallFit of the SAME static ball. At least two
    valid cameras are required: a single view cannot reveal registration
    error. lateral = max(pairwise XY centre delta, broker drift);
    vertical = max(pairwise z delta, |fitted bottom - table|)."""
    good = {k: f for k, f in fits.items() if f.ok}
    if len(good) < 2:
        raise ValueError('need >= 2 valid camera fits to bound registration error')
    if not (np.isfinite(broker_drift_m) and broker_drift_m >= 0):
        raise ValueError('broker drift must be measured')
    cs = [f.center for f in good.values()]
    lat = max(np.linalg.norm(a[:2] - b[:2]) for i, a in enumerate(cs) for b in cs[i + 1:])
    dz = max(abs(a[2] - b[2]) for i, a in enumerate(cs) for b in cs[i + 1:])
    tab = max(abs(f.table_offset) for f in good.values())
    return ErrorBudget(max(float(lat), float(broker_drift_m), 1e-4),
                       max(float(dz), float(tab), 1e-4))


def validate_ball_binding(run, pose: HandPose, fits: dict, primary: str, cloud,
                          table_z, broker_drift_m) -> dict:
    """Produce the plan['ball_validation'] record the selector requires.
    ok is True only when every check passes with the evidence budget."""
    rec = dict(run=run, ok=False, reasons=[], primary=primary,
               fits={k: f.to_json() for k, f in fits.items()})
    fit = fits.get(primary)
    if fit is None or not fit.ok:
        rec['reasons'].append('primary camera ball fit invalid')
        return rec
    try:
        budget = derive_budget(fits, broker_drift_m)
    except ValueError as exc:
        rec['reasons'].append(str(exc))
        return rec
    rec['budget_mm'] = dict(lateral=round(budget.lateral_m * 1e3, 1),
                            vertical=round(budget.vertical_m * 1e3, 1))
    cap = lateral_budget_capacity(fit)
    rec['lateral_capacity_mm'] = round(cap * 1e3, 1)
    if budget.lateral_m > cap:
        rec['reasons'].append(f'lateral uncertainty {budget.lateral_m * 1e3:.1f} mm exceeds '
                              f'gripper capacity {cap * 1e3:.1f} mm')
    lo, hi = feasible_tcp_window(fit, table_z, budget)
    rec['tcp_z_window'] = [round(lo, 4), round(hi, 4)]
    if lo > hi:
        rec['reasons'].append('no vertical TCP window between table and equator')
    ok, reasons, m = check_grasp(pose, fit, cloud, table_z, budget)
    rec['metrics'] = m
    rec['reasons'] += reasons
    rec['ok'] = ok and not rec['reasons']
    return rec


# --- two-camera disagreement diagnosis (narrows the cause, never proves it)
def fit_center_fixed_radius(P, r, c0):
    """Gauss-Newton sphere centre with the radius held fixed."""
    c = np.asarray(c0, float).copy()
    P = np.asarray(P, float)
    for _ in range(50):
        v = P - c
        n = np.linalg.norm(v, axis=1)
        J = -v / n[:, None]
        step, *_ = np.linalg.lstsq(J, -(n - r), rcond=None)
        c += step
        if np.linalg.norm(step) < 1e-8:
            break
    return c


def diagnose_two_camera(fit_a, fit_b, cam_a, cam_b):
    """Split the centre disagreement into along-ray / across-ray parts and
    test whether a per-camera range scale (pure depth bias) can explain it.
    A large residual after the best scale means depth-along-ray bias is not
    the explanation; extrinsic/intrinsic registration, lateral fit bias or
    motion remain open."""
    ca, cb = np.asarray(fit_a.center, float), np.asarray(fit_b.center, float)
    pa, pb = np.asarray(cam_a, float), np.asarray(cam_b, float)
    out = dict(delta_mm=np.round((cb - ca) * 1e3, 1).tolist())
    for name, c, p, other in (('a', ca, pa, cb), ('b', cb, pb, ca)):
        ray = (c - p) / np.linalg.norm(c - p)
        d = c - other
        along = float(d @ ray)
        out[f'{name}_range_m'] = round(float(np.linalg.norm(c - p)), 3)
        out[f'{name}_along_mm'] = round(along * 1e3, 1)
        out[f'{name}_across_mm'] = round(float(np.linalg.norm(d - along * ray)) * 1e3, 1)
    A = np.c_[ca - pa, -(cb - pb)]
    s, *_ = np.linalg.lstsq(A, pb - pa, rcond=None)
    out['best_range_scale'] = np.round(s, 4).tolist()
    out['residual_after_depth_scale_mm'] = round(float(np.linalg.norm(
        (pa + s[0] * (ca - pa)) - (pb + s[1] * (cb - pb)))) * 1e3, 1)
    return out
