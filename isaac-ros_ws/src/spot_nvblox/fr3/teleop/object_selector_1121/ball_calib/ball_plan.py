"""Top-down tennis-ball grasp plan from a wrist-camera ball fix (offline planner).

Stages (direct-JTC executor format, dense Cartesian waypoints):
  S1_home_to_pre   current -> PRE (TCP 12 cm above the grasp point)
  S2_pre_to_grasp  straight vertical descent, 1 cm waypoints
  S3_lift          straight lift 10 cm
  S4_put_back      lift -> grasp (place)
  S5_grasp_to_pre  grasp -> PRE
  S6_pre_to_home   PRE -> Home
Grasp: TCP 6 mm below the ball centre: equator inside the 45 mm fingers, ball top
5.5 mm below the palm, fingertips (8.9 mm beyond the TCP) ~18 mm above the table.
Closing-axis yaw is free for a sphere; choose the one with the best joint margin.
"""
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'swept_replay' / 'ref'))
import grasp_plan_lib as gp  # noqa: E402

BALL_R = 0.0335
TCP_BELOW_CENTRE = 0.006       # ball top 5.5 mm below the palm (fingers 45 mm)
FINGER_LEN = 0.045
FINGERTIP_BEYOND_TCP = 0.0089
PRE_UP = 0.12
LIFT = 0.10
OPEN_HALF = 0.03995           # 79.9 mm open
PAD_HALF_WIDTH = 0.011


def times(qs, start, vmax=0.10, wp_min=0.4, stage_min=1.5):
    out, t, q0 = [], 0.0, np.asarray(start)
    for q in qs:
        t += max(np.abs(np.asarray(q) - q0).max() / vmax, wp_min)
        out.append(round(t, 3))
        q0 = np.asarray(q)
    if out[-1] < stage_min:
        out = [round(x * stage_min / out[-1], 3) for x in out]
    return out


def checks(ball, table_z, lateral_err):
    """Static geometry gates (ball_grasp.py rules) for a centred top-down grasp."""
    tcp_z = ball[2] - TCP_BELOW_CENTRE
    probs = []
    if not 0.003 <= ball[2] - tcp_z <= FINGER_LEN - 0.005:
        probs.append('equator outside the finger pads')
    if ball[2] + BALL_R - tcp_z > FINGER_LEN - 0.003:
        probs.append('ball top would reach the palm')
    if tcp_z - FINGERTIP_BEYOND_TCP - table_z < 0.008:
        probs.append('fingertips too close to the table')
    if OPEN_HALF - BALL_R - lateral_err < 0.002:
        probs.append(f'open-finger side clearance {1000 * (OPEN_HALF - BALL_R - lateral_err):.1f} mm < 2 mm')
    if lateral_err > PAD_HALF_WIDTH * 0.5:
        probs.append('ball centre may leave the pad width')
    return probs


def plan_ball(ball, table_z, lateral_err, start_q, yaw_candidates=np.linspace(-np.pi / 2, np.pi / 2, 13)):
    ball = np.asarray(ball, float)
    probs = checks(ball, table_z, lateral_err)
    R0 = gp.fk(gp.Q_HOME)[:3, :3]
    tcp = np.array([ball[0], ball[1], ball[2] - TCP_BELOW_CENTRE])
    best = None
    for yaw in yaw_candidates:
        c, s = np.cos(yaw), np.sin(yaw)
        R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]) @ R0
        a = R[:, 2]                      # approach (down)
        hand_g = tcp - a * gp.TCP_OFFSET
        hand_pre = hand_g + np.array([0, 0, PRE_UP])
        hand_lift = hand_g + np.array([0, 0, LIFT])
        try:
            q_pre, e1, e2 = gp.ik(hand_pre, R, np.asarray(start_q))
            if e1 > 0.001 or e2 > 0.3:
                continue
            S2 = gp._line(hand_pre, hand_g, q_pre, R, step=0.01)
            S3 = gp._line(hand_g, hand_lift, S2[-1], R, step=0.01)
            S4 = gp._line(hand_lift, hand_g, S3[-1], R, step=0.01)
            S5 = gp._line(hand_g, hand_pre, S4[-1], R, step=0.01)
        except gp.PlanError:
            continue
        allq = [q_pre] + S2 + S3 + S4 + S5
        margin = min(float(np.min(np.minimum(q - gp.QLO, gp.QHI - q))) for q in allq)
        if best is None or margin > best[0]:
            best = (margin, yaw, R, q_pre, S2, S3, S4, S5)
    if best is None:
        raise gp.PlanError('no reachable top-down ball grasp')
    margin, yaw, R, q_pre, S2, S3, S4, S5 = best
    if margin < gp.JOINT_MARGIN_MIN:
        probs.append(f'joint margin {margin:.3f} rad')
    st = dict(S1_home_to_pre=[q_pre], S2_pre_to_grasp=S2, S3_lift=S3, S4_put_back=S4,
              S5_grasp_to_pre=S5, S6_pre_to_home=[gp.Q_HOME])
    plan = dict(object='tennis_ball', ball_centre=ball.round(4).tolist(), table_z=round(float(table_z), 4),
                grasp_tcp=tcp.round(4).tolist(), yaw_deg=round(float(np.degrees(yaw)), 1),
                joint_margin_rad=round(margin, 3), lateral_err_mm=round(1000 * lateral_err, 1),
                expected_width_mm=[round(2000 * BALL_R - 15, 1), round(2000 * BALL_R + 4, 1)],
                held_spheres_hand=[[0.0, 0.0, round(gp.TCP_OFFSET + TCP_BELOW_CENTRE, 4), round(BALL_R + 0.012, 4)]],
                problems=probs, start_q=np.round(np.asarray(start_q), 5).tolist(), stages={}, hand_R=np.round(R, 6).tolist())
    prev = np.asarray(start_q)
    for k, qs in st.items():
        plan['stages'][k] = dict(q=[np.round(q, 5).tolist() for q in qs], t=times(qs, prev))
        prev = np.asarray(qs[-1])
    return plan
