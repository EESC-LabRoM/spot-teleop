"""Motion check: SE(2) ICP vs the anchor + the move trigger. Pure numpy.

Only table-plane motion (x, y, yaw) is estimated: a drill sliding on the
table cannot roll/pitch, and full 6D ICP on partial-view clouds hallucinates
exactly those (11-06 critique). Tipping is detected, not fitted, via the
z-extent veto and forces a full regen.
"""
import numpy as np

from contracts import (COUNT_FRAC, ICP_FIT, ICP_RMSE_MM, MOVE_MM,
                       MOVE_STABLE_MM, STABLE_S, YAW_DEG, YAW_STABLE_DEG,
                       ZEXT_MM, IcpResult)

TOP_BAND = 0.08         # top cluster, same rule as the prologue anchor
VOX_M = 0.005           # ICP working resolution
VOX_COARSE_M = 0.010    # grid-search resolution
FIT_R_M = 0.010         # fitness inlier radius
TRIM_FRAC = 0.7         # keep closest fraction per iteration
ITERS = 25
# Coarse grid (covers the trigger neighborhood; bigger moves correctly fail
# into a full regen): yaw +-20 deg, tx/ty +-50 mm.
GRID_YAW = [float(y) for y in range(-20, 21, 4)]
GRID_T = [float(t) / 1000 for t in range(-50, 51, 10)]


def top_cluster(P):
    P = np.asarray(P, float)
    return P[P[:, 2] > P[:, 2].max() - TOP_BAND]


def downsample(P, vox=VOX_M):
    P = np.asarray(P, float)
    if len(P) == 0:
        return P
    key = np.floor(P / vox).astype(np.int64)
    _, keep = np.unique(key, axis=0, return_index=True)
    return P[np.sort(keep)]


def _nn2(A, B):
    """Squared distances + indices from each B row to nearest A row (brute
    force; ICP runs on confirmed moves only, not per tick)."""
    d2 = ((B[:, None, :2] - A[None, :, :2]) ** 2).sum(-1)  # (B, A)
    i = d2.argmin(1)
    return d2[np.arange(len(B)), i], i


def _kabsch2(A, B):
    """Best yaw + tx,ty mapping B rows onto paired A rows."""
    cA, cB = A.mean(0), B.mean(0)
    H = (B - cB).T @ (A - cA)
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1] *= -1
        R = Vt.T @ U.T
    yaw = float(np.arctan2(R[1, 0], R[0, 0]))
    t = cA - R @ cB
    return yaw, t


def _apply(B, yaw, tx, ty):
    cy, sy = np.cos(yaw), np.sin(yaw)
    return B @ np.array([[cy, sy], [-sy, cy]]) + [tx, ty]


def _rmse_fit(A, B):
    d2, _ = _nn2(A, B)
    rmse = float(np.sqrt(d2.mean()) * 1000)
    fit = float((d2 < FIT_R_M ** 2).mean())
    d2r, _ = _nn2(B, A)
    return rmse, min(fit, float((d2r < FIT_R_M ** 2).mean()))


def icp_se2(anchor, new, iters=ITERS, trim=TRIM_FRAC):
    """Align NEW onto ANCHOR in x,y,yaw. Returns IcpResult with the B->A
    transform; use invert_se2() to carry anchor-frame grasps onto new.

    Coarse-to-fine: point-to-point NN on the smooth drill surface is blind
    past a few mm (each shifted point just matches a neighbor), so a grid
    search seeds the basin and trimmed Kabsch refines. Runs on confirmed
    moves only, not per tick."""
    A = downsample(top_cluster(anchor))[:, :2]
    B0 = downsample(top_cluster(new))
    if len(A) < 25 or len(B0) < 25:
        return IcpResult(0.0, [0.0, 0.0, 0.0], 999.0, 0.0, 999.0, False,
                         'too-few-points')
    Ac = downsample(top_cluster(anchor), VOX_COARSE_M)[:, :2]
    Bc = downsample(top_cluster(new), VOX_COARSE_M)[:, :2]
    best, bi = None, None
    for yd in GRID_YAW:
        for tx in GRID_T:
            for ty in GRID_T:
                rmse, fit = _rmse_fit(Ac, _apply(Bc, np.radians(yd), tx, ty))
                score = (rmse, -fit)
                if best is None or score < best:
                    best, bi = score, (np.radians(yd), tx, ty)
    yaw, t = bi[0], np.array(bi[1:])
    B = _apply(B0[:, :2].copy(), yaw, *t)
    for _ in range(iters):
        d2, i = _nn2(A, B)
        k = max(25, int(trim * len(B)))
        sel = np.argsort(d2)[:k]
        dy, dt = _kabsch2(A[i[sel]], B[sel])
        B = _apply(B, dy, *dt)
        cy, sy = np.cos(dy), np.sin(dy)
        t = np.array([[cy, -sy], [sy, cy]]) @ t + dt
        yaw += dy
    rmse, fit = _rmse_fit(A, B)
    # min-z is pinned by the shared table cut, so only the top is informative.
    zext = abs(float(np.asarray(anchor)[:, 2].max()
                     - np.asarray(new)[:, 2].max())) * 1000
    rigid = bool(rmse < ICP_RMSE_MM and fit > ICP_FIT and zext < ZEXT_MM)
    reason = ('rigid' if rigid else
              f'rmse={rmse:.1f} fit={fit:.2f} zext={zext:.1f}')
    return IcpResult(float(np.degrees(yaw)), [float(t[0]), float(t[1]), 0.0],
                     rmse, fit, zext, rigid, reason)


def invert_se2(res):
    """B->A result flipped to carry anchor-frame points onto the new cloud."""
    yaw = -np.radians(res.yaw_deg)
    cy, sy = np.cos(yaw), np.sin(yaw)
    t = np.array(res.t[:2])
    ti = -np.array([[cy, -sy], [sy, cy]]) @ t
    return float(np.degrees(yaw)), [float(ti[0]), float(ti[1]), 0.0]


def cloud_yaw(P):
    """Major-axis yaw (deg) of the top cluster in the xy-plane."""
    T = top_cluster(P)
    C = np.cov(np.stack([T[:, 0], T[:, 1]]))
    w, V = np.linalg.eigh(C)
    ax = V[:, -1] / (np.linalg.norm(V[:, -1]) or 1.0)
    return float(np.degrees(np.arctan2(ax[1], ax[0])))


def _ang_diff(a, b):
    d = abs(a - b) % 360.0
    return min(d, 360.0 - d)


def eval_trigger(samples, anchor_cxy, anchor_yaw, last_good_n):
    """samples: [(wall, cx, cy, yaw_deg, n)] newest-last from recent ticks.
    Returns (fired, reason, pose|None). Positive evidence only: absence of
    points is never movement. The anchor is FIXED (last grasp run), so slow
    drift still fires once it totals past the bar."""
    if not samples:
        return False, 'no-samples', None
    now = samples[-1][0]
    win = [s for s in samples if now - s[0] <= STABLE_S]
    if not win or now - win[0][0] < STABLE_S - 0.5:
        return False, 'window-too-short', None
    med_n = float(np.median([s[4] for s in win]))
    if med_n < COUNT_FRAC * last_good_n:
        return False, f'count-drop {med_n:.0f}', None
    cx = float(np.median([s[1] for s in win]))
    cy = float(np.median([s[2] for s in win]))
    spread = max(max(s[1] for s in win) - min(s[1] for s in win),
                 max(s[2] for s in win) - min(s[2] for s in win)) * 1000
    yaws = [s[3] for s in win]
    yspread = max(_ang_diff(a, b) for a in yaws for b in yaws)
    if spread > MOVE_STABLE_MM or yspread > YAW_STABLE_DEG:
        return False, f'unstable spread={spread:.1f} yspread={yspread:.1f}', None
    yaw = float(np.median(yaws))
    dc = float(np.hypot(cx - anchor_cxy[0], cy - anchor_cxy[1]) * 1000)
    dy = _ang_diff(yaw, anchor_yaw)
    if dc > MOVE_MM or dy > YAW_DEG:
        return True, f'moved dc={dc:.1f} dy={dy:.1f}', (cx, cy, yaw)
    return False, f'static dc={dc:.1f} dy={dy:.1f}', None
