"""Ball ground-truth calibration motions (offline planner). Stages for grasp_stage_exec.
TCP height at grasp = ball centre - 6 mm (table 0.029, r 0.0335)."""
import json, sys
import numpy as np
sys.path.insert(0, '../swept_replay/ref'); sys.path.insert(0, '../live')
import grasp_plan_lib as gp
import object_grasps as og

TABLE = 0.029; R_BALL = 0.0335
Z_TCP = TABLE + R_BALL - 0.006          # 0.0565
UP_ROT = 0.075                          # rise before rotating (fingertips clear the ball top)
PRE = 0.12
R0 = gp.fk(gp.Q_HOME)[:3, :3]

def Rz(a):
    c, s = np.cos(a), np.sin(a); return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

def hand(tcp, R):
    return np.asarray(tcp, float) - R[:, 2] * gp.TCP_OFFSET

def line(p0, p1, q, R):
    return gp._line(hand(p0, R), hand(p1, R), q, R, step=0.01)

def ik(tcp, R, seed):
    q, e1, e2 = gp.ik(hand(tcp, R), R, seed)
    if e1 > 1e-3 or e2 > 0.3: raise gp.PlanError(f'ik {tcp}')
    return q

def best_yaw(spot, yaws=np.linspace(-np.pi / 2, np.pi / 2, 13)):
    """yaw0 such that yaw0 and yaw0+90 deg are both reachable with margin."""
    best = None
    for y in yaws:
        try:
            qa = ik([*spot, Z_TCP + PRE], Rz(y) @ R0, gp.Q_HOME)
            qb = ik([*spot, Z_TCP + PRE], Rz(y + np.pi / 2) @ R0, qa)
        except gp.PlanError:
            continue
        m = min(float(np.min(np.minimum(q - gp.QLO, gp.QHI - q))) for q in (qa, qb))
        if best is None or m > best[0]: best = (m, y)
    return best[1]

def centre_stages(spot):
    """Home -> pre -> hover(open, at grasp height); then the squeeze sequence."""
    y = best_yaw(spot); Ra, Rb = Rz(y) @ R0, Rz(y + np.pi / 2) @ R0
    g = np.array([*spot, Z_TCP]); pre = g + [0, 0, PRE]; up = g + [0, 0, UP_ROT]
    st = {}
    q_pre = ik(pre, Ra, gp.Q_HOME); st['A1_home_to_pre'] = [q_pre]
    st['A2_down_hover'] = line(pre, g, q_pre, Ra)
    st['A3_up_rot'] = line(g, up, st['A2_down_hover'][-1], Ra)
    q_rot = ik(up, Rb, st['A3_up_rot'][-1]); st['A4_rotate'] = [q_rot]
    st['A5_down'] = line(up, g, q_rot, Rb)
    st['A6_up_pre'] = line(g, pre, st['A5_down'][-1], Rb)
    st['A7_home'] = [gp.Q_HOME]
    return st, dict(spot=list(spot), yaw_a=float(y), yaw_b=float(y + np.pi / 2))

def carry_stages(src, dst, yaw):
    R = Rz(yaw) @ R0
    g0 = np.array([*src, Z_TCP]); g1 = np.array([*dst, Z_TCP]); lift = 0.10
    st = {}
    q = ik(g0 + [0, 0, PRE], R, gp.Q_HOME); st['B1_home_to_pre'] = [q]
    st['B2_down'] = line(g0 + [0, 0, PRE], g0, q, R)
    st['B3_lift'] = line(g0, g0 + [0, 0, lift], st['B2_down'][-1], R)
    st['B4_carry'] = line(g0 + [0, 0, lift], g1 + [0, 0, lift], st['B3_lift'][-1], R)
    st['B5_place'] = line(g1 + [0, 0, lift], g1, st['B4_carry'][-1], R)
    st['B6_up_pre'] = line(g1, g1 + [0, 0, PRE], st['B5_place'][-1], R)
    st['B7_home'] = [gp.Q_HOME]
    return st

def to_plan(st, name, extra=None):
    plan = dict(run=name, stages={}, **(extra or {}))
    prev = gp.Q_HOME
    for k, qs in st.items():
        plan['stages'][k] = dict(q=[np.round(np.asarray(q), 5).tolist() for q in qs], t=og._times(qs, prev, wp_min=0.5))
        prev = np.asarray(qs[-1])
    return plan

if __name__ == '__main__':
    b = open('mesh.bin', 'rb').read(); h = np.frombuffer(b, np.uint32, 3)
    V = np.frombuffer(b, np.float32, int(h[1]) * 3, 12).reshape(-1, 3)
    spots = dict(A=(0.42, -0.27), B=(0.38, -0.15), C=(0.52, -0.30))
    held = np.array([[0, 0, gp.TCP_OFFSET + 0.006, 0.04]])
    out = {}
    stA, meta = centre_stages(spots['A'])
    pA = to_plan(stA, 'ball_truth_centre_A', meta)
    out['centre_A'] = pA
    pB = to_plan(carry_stages(spots['A'], spots['B'], meta['yaw_b']), 'ball_truth_carry_AB', dict(src='A', dst='B'))
    pC = to_plan(carry_stages(spots['B'], spots['C'], meta['yaw_b']), 'ball_truth_carry_BC', dict(src='B', dst='C'))
    out['carry_AB'], out['carry_BC'] = pB, pC
    for name, p in out.items():
        st = {k: v['q'] for k, v in p['stages'].items()}
        carried = {'B3_lift', 'B4_carry', 'B5_place'}
        rep, probs = {}, []
        prev = gp.Q_HOME
        for k, qs in st.items():
            chk, pr = og._sweep_check(gp, {k: qs}, prev, V, np.array(spots['A']), 0.07,
                                      held if k in carried else None, TABLE, min_obst=0.02, min_table=0.008)
            # also exclude other spots' ball column
            rep[k] = chk[k]; probs += pr; prev = np.asarray(qs[-1])
        p['checks'] = rep; p['problems'] = probs
        print(name, 'problems', probs)
        for k, v in rep.items(): print('   ', k, v, 't', p['stages'][k]['t'][-1])
        json.dump(p, open(f'{name}.json', 'w'), indent=1)
    print('yaw_a/b deg', round(np.degrees(meta['yaw_a'])), round(np.degrees(meta['yaw_b'])))
