"""Object-specific grasp plans for the selector (VM side, used by the grasp broker).

Only targets in VALIDATED are planned. Plans use the same stage format as
grasp_plan_lib (direct-JTC executor) and carry held_spheres_hand for the
teleop held-object model. Read-only: no ROS, no motion.
"""
import time
from pathlib import Path

import numpy as np

from fr3_spheres import FR3Spheres

PRE_UP = 0.12
LIFT = 0.10
TABLE_Z_DEFAULT = 0.029
_K = None


def _kin():
    global _K
    if _K is None:
        _K = FR3Spheres()
    return _K


BOTTLE_R = 0.032          # 500 ml PET body radius
ZED1_POS = np.array([0.219, -0.599, 0.629])   # ZED1 optical centre (live TF 11-19)
ZED1_XY = ZED1_POS[:2]


def _band_axis(P, table_z, r=BOTTLE_R):
    """Bottle axis XY from the densest 5 cm band (opaque label) with a fixed-radius circle fit."""
    Q = P[P[:, 2] > table_z + 0.05]
    if len(Q) < 50:
        raise ValueError(f'too few bottle points above the table ({len(Q)})')
    # Clear PET: ZED1 depth also hits whatever stands behind the bottle (11-21: router
    # box behind it -> 63 mm fit rms). Keep only the front surface (nearest 6 cm in range).
    rng = np.linalg.norm(Q - ZED1_POS, axis=1)
    Q = Q[rng < np.percentile(rng, 5) + 0.06]
    zs = np.arange(table_z + 0.05, Q[:, 2].max(), 0.01)
    cnt = [int(((Q[:, 2] >= z) & (Q[:, 2] < z + 0.05)).sum()) for z in zs]
    z0 = float(zs[int(np.argmax(cnt))])
    B = Q[(Q[:, 2] >= z0) & (Q[:, 2] < z0 + 0.05)][:, :2]
    away = B.mean(0) - ZED1_XY
    away /= np.linalg.norm(away)
    c = B.mean(0) + away * r * 0.8
    for _ in range(40):
        dv = B - c
        n = np.linalg.norm(dv, axis=1)
        J = -dv / np.maximum(n, 1e-6)[:, None]
        step = np.linalg.lstsq(J, -(n - r), rcond=None)[0]
        c = c + step
        if np.linalg.norm(step) < 1e-6:
            break
    rms = float(np.sqrt(np.mean((np.linalg.norm(B - c, axis=1) - r) ** 2)))
    return c, len(B), rms, z0


def sprite_cap(cloud, table_z):
    """Cap top: XY = bottle axis from the opaque label band (transparent body depth is
    smeared/see-through), Z = topmost points (cap/neck). Cross-checked; problems listed."""
    P = np.asarray(cloud, float)
    axis, nband, rms, z0 = _band_axis(P, table_z)
    top = P[P[:, 2] > z0 + 0.07]
    probs = []
    if nband < 100:
        probs.append(f'only {nband} label-band points')
    if rms > 0.020:
        probs.append(f'label-band circle fit rms {rms * 1000:.0f} mm')
    if len(top) < 3:
        raise ValueError(f'cap not seen ({len(top)} points above the label)')
    ztop = float(np.max(top[:, 2])) if len(top) < 10 else float(np.percentile(top[:, 2], 90))
    cap_xy = np.median(top[:, :2], 0)
    off = float(np.linalg.norm(cap_xy - axis))
    if off > 0.020:
        probs.append(f'cap points {off * 1000:.0f} mm from the bottle axis')
    if not 0.15 <= ztop - table_z <= 0.32:
        probs.append(f'bottle height {ztop - table_z:.3f} m implausible')
    return np.array([axis[0], axis[1], ztop]), off, probs


def _times(qs, start, vmax=0.10, wp_min=0.4, stage_min=1.5):
    out, t, q0 = [], 0.0, np.asarray(start)
    for q in qs:
        t += max(np.abs(np.asarray(q) - q0).max() / vmax, wp_min)
        out.append(round(t, 3))
        q0 = np.asarray(q)
    if out[-1] < stage_min:
        out = [round(x * stage_min / out[-1], 3) for x in out]
    return out


def _sweep_check(gp, stages, start, mesh_V, exclude_xy, exclude_r, held, table_z,
                 min_obst=0.02, min_table=0.02):
    """Full-body sphere sweep (<=0.01 rad) against map vertices outside the target column."""
    k = _kin()
    V = np.asarray(mesh_V, float)
    O = V[(V[:, 2] > table_z + 0.015) & (np.linalg.norm(V[:, :2] - exclude_xy, axis=1) > exclude_r)]
    O = O[::2].astype(np.float32)
    rep, probs, prev = {}, [], np.asarray(start, float)
    for name, qs in stages.items():
        carried = name in ('S3_lift', 'S4_put_back')
        wo, wt = 9.0, 9.0
        for q in qs:
            q = np.asarray(q, float)
            n = max(2, int(np.ceil(np.abs(q - prev).max() / 0.01)) + 1)
            for s in np.linspace(0, 1, n):
                S, lab = k.spheres(prev + (q - prev) * s, held if carried else None)
                lab = np.array(lab)
                arm = lab != 'fr3_link1'      # link1 stands on the base; mapped base geometry is constant
                if len(O):
                    d = np.sqrt(((S[arm, None, :3].astype(np.float32) - O[None]) ** 2).sum(-1)).min(1) - S[arm, 3]
                    wo = min(wo, float(d.min()))
                body = ~np.isin(lab, ('held',)) if carried else np.ones(len(lab), bool)
                wt = min(wt, float((S[body, 2] - S[body, 3] - table_z).min()))
            prev = q
        rep[name] = dict(min_obstacle_m=round(wo, 3), min_table_clear_m=round(wt, 3), n_waypoints=len(qs))
        if wo < min_obst:
            probs.append(f'{name}: swept clearance {wo * 1000:.0f} mm < {min_obst * 1000:.0f} mm')
        if wt < min_table:
            probs.append(f'{name}: table clearance {wt * 1000:.0f} mm')
    return rep, probs


def _axis_rot(a, th):
    a = np.asarray(a, float) / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K


def plan_topdown(gp, target, tcp, mesh_V, start_q, table_z, held, exclude_r, extra_probs=(),
                 tilts=(0.0, 0.26, 0.4, 0.6)):
    R0 = gp.fk(gp.Q_HOME)[:3, :3]
    radial = np.array([tcp[0], tcp[1], 0.0])
    radial /= max(np.linalg.norm(radial), 1e-6)
    axis = np.cross([0.0, 0.0, 1.0], radial)          # horizontal, perpendicular to the base->object ray
    best = None
    for tilt in tilts:
        for yaw in np.linspace(-np.pi / 2, np.pi / 2, 13):
            c, s = np.cos(yaw), np.sin(yaw)
            Rt = _axis_rot(axis, tilt)
            R = Rt @ np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]) @ R0
            a = R[:, 2]
            if tilt and a @ radial < 0:                  # approach must lean away from the base
                R = _axis_rot(axis, -tilt) @ np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]) @ R0
                a = R[:, 2]
            hg = tcp - a * gp.TCP_OFFSET
            hp, hl = hg - a * PRE_UP, hg + [0, 0, LIFT]
            try:
                qp, e1, e2 = gp.ik(hp, R, np.asarray(start_q))
                if e1 > 0.001 or e2 > 0.3:
                    continue
                S2 = gp._line(hp, hg, qp, R, step=0.01)
                S3 = gp._line(hg, hl, S2[-1], R, step=0.01)
                S4 = gp._line(hl, hg, S3[-1], R, step=0.01)
                S5 = gp._line(hg, hp, S4[-1], R, step=0.01)
            except gp.PlanError:
                continue
            allq = [qp] + S2 + S3 + S4 + S5
            m = min(float(np.min(np.minimum(np.asarray(q) - gp.QLO, gp.QHI - np.asarray(q)))) for q in allq)
            score = m - 0.5 * tilt                      # prefer vertical when it is reachable
            if best is None or score > best[0]:
                best = (score, yaw, R, dict(S1_home_to_pre=[qp], S2_pre_to_grasp=S2, S3_lift=S3, S4_put_back=S4,
                                            S5_grasp_to_pre=S5, S6_pre_to_home=[gp.Q_HOME]), m, tilt)
        if best is not None and tilt == 0:
            break
    if best is None:
        raise gp.PlanError(f'no reachable top-down grasp for {target}')
    _, yaw, R, st, m, tilt_used = best
    probs = list(extra_probs)
    if m < gp.JOINT_MARGIN_MIN:
        probs.append(f'joint margin {m:.3f} rad')
    held = held(R) if callable(held) else held
    checks, p2 = _sweep_check(gp, st, start_q, mesh_V, tcp[:2], exclude_r, held, table_z)
    probs += p2
    run = time.strftime('%Y%m%d_%H%M%S') + f'_{target}_selector'
    plan = dict(run=run, object=target, grasp_tcp=np.round(tcp, 4).tolist(), yaw_deg=round(float(np.degrees(yaw)), 1), tilt_deg=round(float(np.degrees(tilt_used)), 1),
                table_z=round(float(table_z), 4), depth=0.0, approach=np.round(R[:, 2], 3).tolist(),
                grasp_pos=np.round(tcp - R[:, 2] * gp.TCP_OFFSET, 4).tolist(), checks=checks, problems=probs,
                held_spheres_hand=np.round(held, 4).tolist(), start_q=np.round(np.asarray(start_q), 5).tolist(),
                hand_R=np.round(R, 6).tolist(), stages={})
    prev = np.asarray(start_q)
    for k_, qs in st.items():
        plan['stages'][k_] = dict(q=[np.round(np.asarray(q), 5).tolist() for q in qs], t=_times(qs, prev))
        prev = np.asarray(qs[-1])
    return plan


def plan_sprite(gp, cloud, mesh_V, start_q, table_z=TABLE_Z_DEFAULT):
    cap, spread, probs = sprite_cap(cloud, table_z)
    tcp = cap - np.array([0, 0, 0.022])           # pads straddle the cap/neck, tips ~31 mm below the top
    H = cap[2] - table_z
    zs = np.linspace(0.04, max(0.06, H - 0.05), 4)  # bottle body below the cap (world vertical)

    def held(R):
        # rigid grip: the bottle keeps its grasp-time vertical in the hand frame
        down = R.T @ np.array([0.0, 0.0, -1.0])
        cap_h = np.array([0.0, 0.0, gp.TCP_OFFSET]) + R.T @ np.array([0.0, 0.0, 0.022])
        return np.array([[*(cap_h + down * z), 0.040] for z in zs])
    plan = plan_topdown(gp, 'sprite', tcp, mesh_V, start_q, table_z, held, exclude_r=0.06, extra_probs=probs)
    plan.update(cap_top=np.round(cap, 4).tolist(), cap_axis_offset_mm=round(spread * 1000, 1),
                expected_width_mm=[15.0, 45.0])
    return plan


def plan_sprite_side(gp, cloud, mesh_V, start_q, table_z=TABLE_Z_DEFAULT,
                     pitches_deg=(0, 15, 30, 45, 60), min_margin=0.2, axis_inset=0.012, max_checks=8):
    """Near-horizontal body grasp around the opaque label band (user request 11-21).
    Approach along the base->bottle ray, pitched down; closing axis horizontal and
    perpendicular to the ray (cross-camera error lies mostly along the ray, which the
    45 mm fingers absorb). Bottle axis sits axis_inset inside the fingertips so the
    65 mm body clears the palm."""
    P = np.asarray(cloud, float)
    axis, nband, rms, z0 = _band_axis(P, table_z)
    probs = []
    if nband < 100:
        probs.append(f'only {nband} label-band points')
    if rms > 0.020:
        probs.append(f'label-band circle fit rms {rms * 1000:.0f} mm')
    above = P[P[:, 2] > z0 + 0.07]
    top = float(np.max(above[:, 2])) if len(above) else z0 + 0.15
    zg = z0 + 0.025
    radial = np.r_[axis, 0.0] / np.linalg.norm(axis)
    side = np.cross([0.0, 0.0, 1.0], radial)
    side /= np.linalg.norm(side)
    ax_h = np.array([0.0, 0.0, gp.TCP_OFFSET - axis_inset])
    dz = np.linspace(-(zg - table_z - 0.04), max(0.0, top - zg - 0.03), 4)
    # candidates: approach azimuth around the bottle x pitch; flattest pitch first,
    # then nearest to the base->bottle ray; full sweep check in that order.
    cands = []
    for pd in pitches_deg:
        for az in (0.0, -0.5, 0.5, -1.0, 1.0, -1.4, 1.4):
            c, s_ = np.cos(az), np.sin(az)
            d = np.array([c * radial[0] - s_ * radial[1], s_ * radial[0] + c * radial[1]])
            cands.append((pd, az, d))
    tried, chosen, checked = [], None, 0
    for pd, az, d in cands:
        if chosen is not None and pd > chosen[1]:
            break
        ph = np.radians(pd)
        a = np.r_[np.cos(ph) * d, -np.sin(ph)]
        sd = np.cross([0.0, 0.0, 1.0], np.r_[d, 0.0])
        for flip in (1, -1):
            yy = sd * flip
            R = np.c_[np.cross(yy, a), yy, a]
            tcp = np.r_[axis, zg] + a * axis_inset
            hg = tcp - a * gp.TCP_OFFSET
            hp, hl = hg - a * PRE_UP, hg + [0, 0, LIFT]
            try:
                qp, e1, e2 = gp.ik(hp, R, np.asarray(start_q))
                if e1 > 0.001 or e2 > 0.3:
                    continue
                S2 = gp._line(hp, hg, qp, R, step=0.01)
                S3 = gp._line(hg, hl, S2[-1], R, step=0.01)
                S4 = gp._line(hl, hg, S3[-1], R, step=0.01)
                S5 = gp._line(hg, hp, S4[-1], R, step=0.01)
            except gp.PlanError:
                continue
            allq = [qp] + S2 + S3 + S4 + S5
            m = min(float(np.min(np.minimum(np.asarray(q) - gp.QLO, gp.QHI - np.asarray(q)))) for q in allq)
            if m < min_margin or checked >= max_checks:
                continue
            st = dict(S1_home_to_pre=[qp], S2_pre_to_grasp=S2, S3_lift=S3, S4_put_back=S4,
                      S5_grasp_to_pre=S5, S6_pre_to_home=[gp.Q_HOME])
            up = R.T @ np.array([0.0, 0.0, 1.0])
            held = np.array([[*(ax_h + up * dz_), 0.040] for dz_ in dz])
            checks, p2 = _sweep_check(gp, st, start_q, mesh_V, axis, 0.06, held, table_z)
            checked += 1
            tried.append((pd, round(float(np.degrees(az))), flip, round(m, 2), len(p2)))
            if not p2 and (chosen is None or m > chosen[0]):
                chosen = (m, pd, R, tcp, st, held, checks)
    if chosen is None:
        raise gp.PlanError(f'no collision-free side grasp for the bottle (tried {tried})')
    m, pd, R, tcp, st, held, checks = chosen
    run = time.strftime('%Y%m%d_%H%M%S') + '_sprite_selector'
    plan = dict(run=run, object='sprite', grasp='side', pitch_deg=pd, bottle_axis=np.round(axis, 4).tolist(),
                grasp_tcp=np.round(tcp, 4).tolist(), table_z=round(float(table_z), 4), depth=0.0,
                approach=np.round(R[:, 2], 3).tolist(), grasp_pos=np.round(tcp - R[:, 2] * gp.TCP_OFFSET, 4).tolist(),
                joint_margin_rad=round(m, 3), band_fit_rms_mm=round(rms * 1000, 1), checks=checks, problems=probs,
                held_spheres_hand=np.round(held, 4).tolist(), start_q=np.round(np.asarray(start_q), 5).tolist(),
                hand_R=np.round(R, 6).tolist(), expected_width_mm=[45.0, 75.0], stages={})
    prev = np.asarray(start_q)
    for k_, qs in st.items():
        plan['stages'][k_] = dict(q=[np.round(np.asarray(q), 5).tolist() for q in qs], t=_times(qs, prev))
        prev = np.asarray(qs[-1])
    return plan


# Side (body) grasp replaced the cap grasp after the 11-21 cap miss.
PLANNERS = {'sprite': plan_sprite_side}
