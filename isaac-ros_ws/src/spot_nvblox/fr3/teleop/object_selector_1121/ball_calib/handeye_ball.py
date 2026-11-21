"""Wrist-camera hand-eye + ball position from several views of one static ball.

Unknowns: X = fr3_hand -> camera_color_optical_frame (6 DoF) and P = ball
centre in fr3_link0. Each view i gives the hand pose T_i (FK of measured
joints) and the ball centre c_i in the camera (sphere fit on RealSense depth).
Residual: T_i * X * c_i - P. Offline, no robot or ROS access.
"""
import json
import sys
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

BALL_R = 0.0335  # ITF nominal radius (32.7-34.3 mm)


def ball_mask(rgb):
    import cv2
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    m = ((hsv[..., 0] >= 25) & (hsv[..., 0] <= 45) & (hsv[..., 1] >= 80) & (hsv[..., 2] >= 80)).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    n, lab, st, _ = cv2.connectedComponentsWithStats(m)
    if n < 2:
        return None
    best = None
    for i in range(1, n):
        x, y, w, h, a = st[i]
        if a < 300 or not 0.6 < w / max(h, 1) < 1.6:
            continue
        fill = a / float(w * h)
        if fill < 0.55:          # a sphere's disc fills ~78% of its box; the helmet does not
            continue
        if best is None or a > st[best][4]:
            best = i
    if best is None:
        return None
    return lab == best


def fit_sphere_fixed(P, r=BALL_R, iters=30):
    c = P.mean(0) + np.array([0, 0, r * 0.5])
    for _ in range(iters):
        d = P - c
        n = np.linalg.norm(d, axis=1, keepdims=True)
        J = -d / n
        res = n[:, 0] - r
        step, *_ = np.linalg.lstsq(J, -res, rcond=None)
        c = c + step
        if np.linalg.norm(step) < 1e-7:
            break
    res = np.linalg.norm(P - c, axis=1) - r
    return c, float(np.sqrt(np.mean(res ** 2)))


def ball_in_camera(npz):
    import cv2
    d = np.load(npz)
    rgb, depth, K = d['rgb'], d['depth_m'], d['K']
    m = ball_mask(rgb)
    if m is None:
        return None, 'ball not found in image'
    core = cv2.erode(m.astype(np.uint8), np.ones((7, 7), np.uint8)) > 0
    v, u = np.nonzero(core & (depth > 0.1) & (depth < 1.0))
    if len(u) < 80:
        return None, f'too few depth pixels on the ball ({len(u)})'
    z = depth[v, u]
    med = np.median(z)
    keep = np.abs(z - med) < 0.04
    u, v, z = u[keep], v[keep], z[keep]
    P = np.c_[(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z]
    c, rms = fit_sphere_fixed(P)
    if rms > 0.004:
        return None, f'sphere fit rms {rms * 1000:.1f} mm'
    # table ring around the ball (2.5..6 radii in the image), for the plane constraint
    ys, xs = np.nonzero(m)
    cu, cv, rad = xs.mean(), ys.mean(), max(np.ptp(xs), np.ptp(ys)) / 2
    vv, uu = np.mgrid[0:depth.shape[0]:3, 0:depth.shape[1]:3]
    dd = np.hypot(uu - cu, vv - cv)
    ring = (dd > 2.5 * rad) & (dd < 6 * rad) & (depth[vv, uu] > 0.1)
    zt = depth[vv, uu][ring]
    zt_ok = np.abs(zt - np.median(zt)) < 0.03 if len(zt) else np.zeros(0, bool)
    ut, vt, zt = uu[ring][zt_ok], vv[ring][zt_ok], zt[zt_ok]
    table = np.c_[(ut - K[0, 2]) * zt / K[0, 0], (vt - K[1, 2]) * zt / K[1, 1], zt]
    return (c, table), f'{len(P)} pts rms {rms * 1000:.1f} mm, range {c[2]:.3f} m, table pts {len(table)}'


def solve(T_list, c_list, tables=None, w_table=0.3):
    """P is constrained to rest on the table: P = (px, py, h + BALL_R), with h the
    table height in fr3_link0, observed in every view through the table ring."""
    T_list = [np.asarray(T, float) for T in T_list]
    c_list = [np.asarray(c, float) for c in c_list]
    tables = tables or [None] * len(T_list)
    tables = [None if t is None else np.asarray(t, float)[::max(1, len(t) // 200)] for t in tables]

    def res(p):
        Rx = Rotation.from_rotvec(p[:3]).as_matrix()
        P = np.array([p[6], p[7], p[8] + BALL_R])
        out = []
        for T, c, tb in zip(T_list, c_list, tables):
            out.append(T[:3, :3] @ (Rx @ c + p[3:6]) + T[:3, 3] - P)
            if tb is not None and len(tb):
                zb = (tb @ Rx.T + p[3:6]) @ T[:3, :3].T + T[:3, 3]
                out.append(w_table * (zb[:, 2] - p[8]) / np.sqrt(len(tb)))
        return np.concatenate(out)

    best = None
    for yaw in np.linspace(-np.pi, np.pi, 8, endpoint=False):
        rv = Rotation.from_euler('z', yaw).as_rotvec()
        P0 = np.mean([T[:3, 3] + T[:3, 2] * 0.3 for T in T_list], 0)
        r = least_squares(res, np.r_[rv, 0.05, 0.0, 0.03, P0[0], P0[1], 0.03], method='lm')
        if best is None or r.cost < best.cost:
            best = r
    p = best.x
    Rx = Rotation.from_rotvec(p[:3]).as_matrix()
    P = np.array([p[6], p[7], p[8] + BALL_R])
    e = np.array([T[:3, :3] @ (Rx @ c + p[3:6]) + T[:3, 3] - P for T, c in zip(T_list, c_list)])
    X = np.eye(4)
    X[:3, :3] = Rx
    X[:3, 3] = p[3:6]
    return dict(X=X, P=P, table_z=float(p[8]), per_view_err=np.linalg.norm(e, axis=1),
                rms=float(np.sqrt((e ** 2).sum(1).mean())))


def leave_one_out(T_list, c_list, tables=None):
    """Ball position spread when each view is dropped (needs >= 5 views)."""
    Ps = []
    tables = tables or [None] * len(T_list)
    for i in range(len(T_list)):
        keep = [j for j in range(len(T_list)) if j != i]
        r = solve([T_list[j] for j in keep], [c_list[j] for j in keep], [tables[j] for j in keep])
        Ps.append(r['P'])
    Ps = np.array(Ps)
    return float(np.linalg.norm(Ps - Ps.mean(0), axis=1).max())


if __name__ == '__main__':
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'swept_replay'))
    from fr3_spheres import FR3Spheres
    k = FR3Spheres()
    views = json.load(open(sys.argv[1]))   # [{"npz":..., "q":[7]}]
    Ts, cs, tbs = [], [], []
    for v in views:
        obs, why = ball_in_camera(v['npz'])
        print(v['npz'], why)
        if obs is not None:
            Ts.append(k.link_poses(np.array(v['q']))['fr3_hand'])
            cs.append(obs[0])
            tbs.append(obs[1])
    r = solve(Ts, cs, tbs)
    loo = leave_one_out(Ts, cs, tbs) if len(Ts) >= 5 else None
    out = dict(n_views=len(Ts), ball_base_m=r['P'].round(5).tolist(), table_z_m=round(r['table_z'], 5), rms_mm=round(r['rms'] * 1000, 2),
               per_view_err_mm=(r['per_view_err'] * 1000).round(2).tolist(),
               leave_one_out_spread_mm=None if loo is None else round(loo * 1000, 2),
               hand_to_camera=r['X'].round(6).tolist())
    print(json.dumps(out, indent=1))
    if len(sys.argv) > 2:
        Path(sys.argv[2]).write_text(json.dumps(out, indent=1))
