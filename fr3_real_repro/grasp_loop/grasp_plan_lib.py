#!/usr/bin/env python3
"""FR3 grasp planner as a library (2026-11-06, split out of grasp_plan.py for the regrasp loop).

ONE planner for the staged runner (grasp_plan.py CLI -> grasp_test.sh) and the regrasp loop (regrasp/select.py).
Direct JTC path: IK on the FR3 (modified DH, flange 0.107; hand = flange @ Rz(-pi/4)); target hand pose =
GraspGen pose @ Rz(+90) (Franka hand closes on Y, GraspGen on X), flip 0/180 deg about the approach.
Stages: S1 home->PRE (12 cm back along -approach), S2 PRE->GRASP (line), S3 GRASP->LIFT (+lift z),
S4 LIFT->GRASP, S5 GRASP->PRE, S6 PRE->home.
Checks (-> plan['problems']): table clearance >= 4 cm, S1/S6 arm vs map mesh >= 6 cm and outside the drill box,
joint margin >= 0.10 rad, hand vs cloud (palm/finger bodies <= 5 pts, fingertips below the top, >= 50 pts between pads).
"""
import json
import urllib.request

import numpy as np

# = lab ./fr3_ros2.sh real-home / dashboard Home / pad X pose (fr3_home_jtc.py HOME)
Q_HOME = np.array([0.0, -0.7853981634, 0.0, -2.3561944902, 0.0, 1.5707963268, 0.7853981634])
QLO = np.array([-2.74, -1.78, -2.90, -3.04, -2.80, 0.54, -3.01])   # FR3 limits (soft, as used by the IK clip)
QHI = np.array([2.74, 1.78, 2.90, -0.15, 2.80, 4.52, 3.01])
DH = [(0, 0, 0.333), (0, -np.pi/2, 0), (0, np.pi/2, 0.316), (0.0825, np.pi/2, 0),
      (-0.0825, -np.pi/2, 0.384), (0, np.pi/2, 0), (0.088, np.pi/2, 0)]
TCP_OFFSET = 0.1034      # fr3_hand -> fr3_hand_tcp along the approach (fingertips are 8.9 mm further)
PRE_BACK = 0.12          # pre-grasp distance back along the approach (m)
JOINT_MARGIN_MIN = 0.10  # rad
BETWEEN_PADS_MIN = 50    # cloud points


class PlanError(Exception):
    """No valid plan (IK failure at the grasp/PRE/line)."""


def Rz(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def fk(q, pts=False):
    T = np.eye(4)
    P = [T[:3, 3].copy()]
    for (al_a, al, d), th in zip(DH, q):
        ca, sa, ct, st = np.cos(al), np.sin(al), np.cos(th), np.sin(th)
        T = T @ np.array([[ct, -st, 0, al_a], [st*ca, ct*ca, -sa, -d*sa],
                          [st*sa, ct*sa, ca, d*ca], [0, 0, 0, 1]])
        P.append(T[:3, 3].copy())
    F = T.copy()
    F[:3, 3] = T[:3, 3] + T[:3, 2] * 0.107
    H = np.eye(4)
    H[:3, :3] = F[:3, :3] @ Rz(-np.pi/4)
    H[:3, 3] = F[:3, 3]
    if pts:
        tip = H[:3, 3] + H[:3, 2] * TCP_OFFSET
        P += [F[:3, 3], tip]
        return H, np.array(P)
    return H


def rot_err(Ra, Rb):
    E = Rb @ Ra.T
    return 0.5 * np.array([E[2, 1] - E[1, 2], E[0, 2] - E[2, 0], E[1, 0] - E[0, 1]])


def ik(p, R, q0, iters=300):
    q = q0.copy()
    for _ in range(iters):
        H = fk(q)
        e = np.concatenate([p - H[:3, 3], rot_err(H[:3, :3], R)])
        if np.linalg.norm(e[:3]) < 1e-4 and np.linalg.norm(e[3:]) < 1e-3:
            break
        J = np.zeros((6, 7))
        for i in range(7):
            dq = np.zeros(7)
            dq[i] = 1e-6
            Hd = fk(q + dq)
            J[:3, i] = (Hd[:3, 3] - H[:3, 3]) / 1e-6
            J[3:, i] = rot_err(H[:3, :3], Hd[:3, :3]) / 1e-6
        lam = 0.02
        q = q + J.T @ np.linalg.solve(J @ J.T + lam**2 * np.eye(6), e)
        q = np.clip(q, QLO + 0.02, QHI - 0.02)
    H = fk(q)
    return q, float(np.linalg.norm(p - H[:3, 3])), float(np.degrees(np.linalg.norm(rot_err(H[:3, :3], R))))


def load_grasp(url, timeout=5):
    """grasp.json-like dict: R (3x3 GraspGen), pick (hand position), cloud (Nx3 or JSON string), run."""
    g = json.load(urllib.request.urlopen(url, timeout=timeout))
    if isinstance(g.get('cloud'), str):
        g['cloud'] = json.loads(g['cloud'])
    return g


def load_mesh(url, timeout=10):
    """Map mesh vertices (Nx3 float32) from mesh_server's mesh.bin."""
    b = urllib.request.urlopen(url, timeout=timeout).read()
    h = np.frombuffer(b, np.uint32, 3)
    return np.frombuffer(b, np.float32, int(h[1]) * 3, 12).reshape(-1, 3)


def _choose_flip(p_grasp, Rg, approach, seed_q):
    """Flip (0/180 deg about the approach) closest to home whose grasp AND PRE are both reachable."""
    cands = []
    for flip in (0.0, np.pi):  # gripper is symmetric about the approach axis
        Rh = Rg @ Rz(np.pi/2) @ Rz(flip)
        q, ep, er = ik(p_grasp, Rh, Q_HOME if seed_q is None else seed_q)
        cands.append((np.abs(q - Q_HOME).sum(), flip, Rh, q, ep, er))
    cands.sort(key=lambda c: c[0])
    for c in cands:
        if c[4] > 0.002 or c[5] > 0.5:
            continue
        for seed in (c[3], Q_HOME):
            _, e1, e2 = ik(p_grasp - approach * PRE_BACK, c[2], seed)
            if e1 < 0.002 and e2 < 0.5:
                return c, seed is Q_HOME
    c0 = cands[0]
    raise PlanError(f'IK failed at the grasp or PRE for both flips: grasp pos err {c0[4]*1000:.1f} mm rot err {c0[5]:.2f} deg')


def _line(p0, p1, q_seed, Rh, step=0.01):
    """Straight hand line. IK solved DENSELY (1 cm) so the arm configuration stays continuous; every `step` m kept."""
    L = np.linalg.norm(p1 - p0)
    n = max(1, int(np.ceil(L / 0.01)))
    keep_every = max(1, int(round(step / 0.01)))
    qs, q = [], q_seed
    for k in range(1, n + 1):
        q, e1, e2 = ik(p0 + (p1 - p0) * k / n, Rh, q)
        if e1 > 0.002 or e2 > 0.5:
            raise PlanError(f'IK failed on a line waypoint ({e1*1000:.1f} mm, {e2:.2f} deg)')
        if k % keep_every == 0 or k == n:
            qs.append(q)
    return qs


def _times(qs, start, vmax, wp_min_time):
    """Per-waypoint times from the speed cap (>= 1.5 s per stage)."""
    out, t, q0 = [], 0.0, start
    for q in qs:
        t += max(np.abs(np.array(q) - q0).max() / vmax, wp_min_time)
        out.append(round(t, 3))
        q0 = np.array(q)
    if out[-1] < 1.5:
        out = [round(x * 1.5 / out[-1], 3) for x in out]
    return out


def hand_checks(q_grasp_end, cloud):
    """Hand at the grasp vs the drill cloud (hand Y = closing axis, Z = approach; TCP = fingertip centre)."""
    Hg = fk(q_grasp_end)
    A_, Y_, X_ = Hg[:3, 2], Hg[:3, 1], Hg[:3, 0]
    tcp_g = Hg[:3, 3] + A_ * TCP_OFFSET
    Cl = np.asarray(cloud, float)
    r_ = Cl - tcp_g
    ry, ra, rx = r_ @ Y_, r_ @ A_, r_ @ X_
    palm_pts = int(((ra < -0.05) & (ra > -0.11) & (np.abs(ry) < 0.105) & (np.abs(rx) < 0.035)).sum())
    finger_pts = int(((np.abs(np.abs(ry) - 0.045) < 0.01) & (np.abs(rx) < 0.012) & (ra > -0.05) & (ra < 0.005)).sum())
    between_pts = int(((np.abs(ry) < 0.035) & (np.abs(rx) < 0.011) & (ra > -0.045) & (ra < 0.0)).sum())
    col = np.linalg.norm(Cl[:, :2] - tcp_g[:2], axis=1) < 0.03
    top_z = float(Cl[col, 2].max()) if col.any() else float('nan')
    tip_hi = float(max((tcp_g + Y_ * 0.04)[2], (tcp_g - Y_ * 0.04)[2]))
    rep = dict(palm_pts_in_drill=palm_pts, finger_body_pts=finger_pts, pts_between_pads=between_pts,
               tcp_below_local_top_cm=round((top_z - tcp_g[2]) * 100, 1),
               higher_tip_below_top_cm=round((top_z - tip_hi) * 100, 1))
    probs = []
    if palm_pts > 5 or finger_pts > 5:
        probs.append('grasp_hand: palm/finger body inside the drill cloud (reduce --depth)')
    if top_z - tip_hi < 0.005:
        probs.append('grasp_hand: a fingertip is at/above the drill top (shallow grasp; raise --depth)')
    if between_pts < BETWEEN_PADS_MIN:
        probs.append(f'grasp_hand: only {between_pts} cloud points between the pads (< {BETWEEN_PADS_MIN}): '
                     'grasp would close on air or on an edge')
    return rep, probs


def plan_grasp(g, mesh_V, depth=0.0, z_offset=0.0, map_z_bias=0.0, seed_q=None, lift=0.10,
               vmax=0.10, cart_step=0.01, wp_min_time=0.4):
    """Plan the staged grasp for grasp dict `g` (R, pick, cloud, run) against map vertices `mesh_V`.
    Returns the plan dict (same format as grasp_plan.py OUT.json, plus hand_R). Raises PlanError if IK fails.
    plan['problems'] == [] means every check passed."""
    g = dict(g)
    cloud = np.asarray(g['cloud'], float)
    pick = np.asarray(g['pick'], float)
    if map_z_bias:
        pick = pick + [0, 0, map_z_bias]
        cloud = cloud + [0, 0, map_z_bias]
    Rg = np.asarray(g['R'], float)
    approach = Rg[:, 2] / np.linalg.norm(Rg[:, 2])
    p_grasp = pick + [0, 0, z_offset] + approach * depth

    (_, flip, Rh, q_grasp, ep, er), pre_seed_home = _choose_flip(p_grasp, Rg, approach, seed_q)
    p_pre = p_grasp - approach * PRE_BACK
    p_lift = p_grasp + np.array([0, 0, lift])
    q_pre, e1, e2 = ik(p_pre, Rh, Q_HOME if pre_seed_home else q_grasp)
    if not (e1 < 0.002 and e2 < 0.5):
        raise PlanError('IK failed at PRE')
    S = {}
    S['S1_home_to_pre'] = [q_pre]
    S['S2_pre_to_grasp'] = _line(p_pre, p_grasp, q_pre, Rh, step=cart_step)
    S['S3_lift'] = _line(p_grasp, p_lift, S['S2_pre_to_grasp'][-1], Rh, step=cart_step)
    S['S4_put_back'] = _line(p_lift, p_grasp, S['S3_lift'][-1], Rh, step=cart_step)
    S['S5_grasp_to_pre'] = _line(p_grasp, p_pre, S['S4_put_back'][-1], Rh, step=cart_step)
    S['S6_pre_to_home'] = [Q_HOME]

    V = np.asarray(mesh_V, np.float32) + np.array([0, 0, map_z_bias], np.float32)
    O = V[(V[:, 2] > 0.06) & (V[:, 0] > 0.1)]
    DR = np.array([cloud.min(0) - 0.03, cloud.max(0) + 0.03])
    table_z = float(np.percentile(V[(V[:, 2] < 0.08) & (V[:, 0] > 0.3)][:, 2], 50))
    report = {}
    prev = Q_HOME
    for name, qs in S.items():
        worst_obst, worst_tab, in_drill = 9.0, 9.0, False
        for q_end in qs:
            for t in np.linspace(0, 1, 8):
                qq = prev + (q_end - prev) * t
                if not (np.all(qq >= QLO) and np.all(qq <= QHI)):
                    raise PlanError(f'{name}: joint limit')
                _, P = fk(qq, pts=True)
                Sg = np.array([P[i] * (1 - s) + P[i + 1] * s for i in range(3, len(P) - 1) for s in np.linspace(0, 1, 10)])
                arm = Sg[: -10 * 2]  # links up to the flange (hand/fingers checked via table only)
                if len(O):
                    worst_obst = min(worst_obst, float(np.min(np.linalg.norm(arm[:, None] - O[None, ::2], axis=2))))
                worst_tab = min(worst_tab, float(Sg[:, 2].min() - table_z))
                in_drill |= bool(np.any(np.all((arm > DR[0]) & (arm < DR[1]), 1)))
            prev = q_end
        jm = min(float(np.min(np.minimum(np.asarray(qe) - QLO, QHI - np.asarray(qe)))) for qe in qs)
        report[name] = dict(min_obstacle_m=round(worst_obst, 3), min_table_clear_m=round(worst_tab, 3),
                            arm_links_in_drill_box=in_drill, n_waypoints=len(qs), joint_limit_margin_rad=round(jm, 3))
    problems = [k for k, r in report.items() if r['min_table_clear_m'] < 0.04
                or (k in ('S1_home_to_pre', 'S6_pre_to_home') and (r['arm_links_in_drill_box'] or r['min_obstacle_m'] < 0.06))]
    problems += [f'{k}: joint within {r["joint_limit_margin_rad"]:.3f} rad of its limit (< {JOINT_MARGIN_MIN:.2f})'
                 for k, r in report.items() if r['joint_limit_margin_rad'] < JOINT_MARGIN_MIN]
    report['grasp_hand'], hp = hand_checks(S['S2_pre_to_grasp'][-1], cloud)
    problems += hp

    plan = dict(run=g.get('run'), grasp_pos=p_grasp.round(4).tolist(), grasp_tcp=(p_grasp + approach * TCP_OFFSET).round(4).tolist(),
                approach=approach.round(3).tolist(), map_z_bias=map_z_bias, depth=depth, flip_deg=round(float(np.degrees(flip)), 1),
                z_offset=z_offset, table_z=round(table_z, 4), ik_err_mm=round(ep * 1000, 2), ik_err_deg=round(er, 3),
                stages={}, checks=report, problems=problems,
                hand_R=np.round(Rh, 6).tolist())  # fr3_hand rotation at the grasp (for /grasp_target)
    prev = Q_HOME
    for name, qs in S.items():
        plan['stages'][name] = dict(q=[np.round(q, 5).tolist() for q in qs], t=_times(qs, prev, vmax, wp_min_time))
        prev = np.array(qs[-1])
    return plan
