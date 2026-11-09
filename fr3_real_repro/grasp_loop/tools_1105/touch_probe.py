#!/usr/bin/env python3
"""Touch probe: fingertip FK z at contact on the two tables (+ drill top).

Runs INSIDE fr3-ros2 (jazzy, domain 77). Micro-steps on the LIVE JTC
(fr3_arm_controller): no controller switch, no map edit, no MPC.
  python3 touch_probe.py [--targets A,B,C] [--selftest]

Guards (mirrors grasp_stage_exec.py): refuses on foreign JTC traffic (1.5 s
before + mid-run abort); arrival bar 0.05 rad; contact = |Fz-base|>6 N or
|F|>10 N twice in a row (F from /franka_robot_state_broadcaster/robot_state);
floor z=-0.01 m; 55 steps of 5 mm max; aborts if robot_mode != 2.
Operator: gripper OPEN at 80 mm first (tips assumed +-40 mm), ENTER per
target, hand on the e-stop. --selftest runs IK/FK only (no ROS, no motion).
Ctrl+C sends HOLD (current joints, no spin) then exits 130, so the JTC stops
in place instead of coasting the accepted goal out.
"""
import json
import signal
import sys
import time

import numpy as np

Q_HOME = np.array([0.0, -0.7853981634, 0.0, -2.3561944902, 0.0, 1.5707963268, 0.7853981634])
QLO = np.array([-2.74, -1.78, -2.90, -3.04, -2.80, 0.54, -3.01])
QHI = np.array([2.74, 1.78, 2.90, -0.15, 2.80, 4.52, 3.01])
DH = [(0, 0, 0.333), (0, -np.pi/2, 0), (0, np.pi/2, 0.316), (0.0825, np.pi/2, 0),
      (-0.0825, -np.pi/2, 0.384), (0, np.pi/2, 0), (0.088, np.pi/2, 0)]
TOOL = 0.1034
R_DOWN = [np.diag([1.0, -1.0, -1.0]) @ np.array([[np.cos(f), -np.sin(f), 0],
                                                 [np.sin(f), np.cos(f), 0], [0, 0, 1]])
          for f in (0.0, np.pi/2, np.pi, -np.pi/2)]

TARGETS = {'A': (0.15, 0.20, 'Franka table'), 'B': (0.30, -0.25, 'drill table'),
           'C': (0.41, -0.35, 'drill top')}
PRE_Z, FLOOR_Z, STEP, MAX_STEPS = 0.25, -0.01, 0.005, 55
NAMES = [f'fr3_joint{i}' for i in range(1, 8)]
TAG = 'touch_probe'


def fk(q):
    T = np.eye(4)
    for (al_a, al, d), th in zip(DH, q):
        ca, sa, ct, st = np.cos(al), np.sin(al), np.cos(th), np.sin(th)
        T = T @ np.array([[ct, -st, 0, al_a], [st*ca, ct*ca, -sa, -d*sa],
                          [st*sa, ct*sa, ca, d*ca], [0, 0, 0, 1]])
    F = T.copy()
    F[:3, 3] = T[:3, 3] + T[:3, 2]*0.107
    H = np.eye(4)
    H[:3, :3] = F[:3, :3] @ np.array([[np.cos(-np.pi/4), -np.sin(-np.pi/4), 0],
                                      [np.sin(-np.pi/4), np.cos(-np.pi/4), 0], [0, 0, 1]])
    H[:3, 3] = F[:3, 3]
    return H


def rot_err(Ra, Rb):
    E = Rb @ Ra.T
    return 0.5*np.array([E[2, 1]-E[1, 2], E[0, 2]-E[2, 0], E[1, 0]-E[0, 1]])


def ik(p, R, q0, iters=300):
    q = q0.copy()
    for _ in range(iters):
        H = fk(q)
        e = np.concatenate([p-H[:3, 3], rot_err(H[:3, :3], R)])
        if np.linalg.norm(e[:3]) < 1e-4 and np.linalg.norm(e[3:]) < 1e-3:
            break
        J = np.zeros((6, 7))
        for i in range(7):
            dq = np.zeros(7)
            dq[i] = 1e-6
            Hd = fk(q+dq)
            J[:3, i] = (Hd[:3, 3]-H[:3, 3])/1e-6
            J[3:, i] = rot_err(H[:3, :3], Hd[:3, :3])/1e-6
        q = q+J.T @ np.linalg.solve(J @ J.T+0.02**2*np.eye(6), e)
        q = np.clip(q, QLO+0.02, QHI-0.02)
    H = fk(q)
    return q, float(np.linalg.norm(p-H[:3, 3])), float(np.degrees(np.linalg.norm(rot_err(H[:3, :3], R))))


def ik_down(p, q0):
    seeds = [q0] if np.abs(q0-Q_HOME).max() < 1e-6 else [q0, Q_HOME]
    cands = []
    for s in seeds:
        for R in R_DOWN:
            q, ep, er = ik(p, R, s)
            cands.append((ep, er, np.abs(q-q0).sum(), q, R))
    cands.sort(key=lambda c: (c[0], c[1], c[2]))
    _, _, _, q, R = cands[0]
    _, ep, er = ik(p, R, q, iters=50)
    H = fk(q)
    return q, ep, er, R


def tcp_of(q):
    H = fk(q)
    return H[:3, 3]+H[:3, 2]*TOOL, H


def tilt_of(H):
    return float(np.degrees(np.arccos(max(-1, min(1, -H[2, 2])))))


def selftest():
    q = Q_HOME.copy()
    ok = True
    for k, (x, y, label) in TARGETS.items():
        qp, ep, er, _ = ik_down(np.array([x, y, PRE_Z]), q)
        _, H = tcp_of(qp)
        t = tilt_of(H)
        print(f'{k} {label}: pre err {ep*1000:.1f} mm/{er:.2f} deg tilt {t:.2f} deg')
        ok &= ep < 0.002 and er < 0.5 and t < 3.0
        q = qp
    print('SELFTEST', 'OK' if ok else 'FAIL')
    sys.exit(0 if ok else 1)


if '--selftest' in sys.argv:
    selftest()

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from franka_msgs.msg import FrankaRobotState

want = [t for t in sys.argv[1:] if not t.startswith('-')]
if want:
    keys = want[0].split(',') if '=' not in want[0] else want[0].split('=')[1].split(',')
else:
    keys = ['A', 'B', 'C']
keys = [k for k in keys if k in TARGETS]

rclpy.init()
n = Node('touch_probe')
q, F, mode = {}, [0.0, 0.0, 0.0], [2]
n.create_subscription(JointState, '/joint_states', lambda m: q.update(zip(m.name, m.position)), 10)
n.create_subscription(FrankaRobotState, '/franka_robot_state_broadcaster/robot_state',
                      lambda m: (F.__setitem__(0, m.o_f_ext_hat_k.wrench.force.x),
                                 F.__setitem__(1, m.o_f_ext_hat_k.wrench.force.y),
                                 F.__setitem__(2, m.o_f_ext_hat_k.wrench.force.z),
                                 mode.__setitem__(0, m.robot_mode)),
                      QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT))
pub = n.create_publisher(JointTrajectory, '/fr3_arm_controller/joint_trajectory', 10)
foreign = []
n.create_subscription(JointTrajectory, '/fr3_arm_controller/joint_trajectory',
                      lambda m: foreign.append(time.time()) if m.header.frame_id != TAG else None, 10)


def spin(t):
    t0 = time.time()
    while time.time()-t0 < t:
        rclpy.spin_once(n, timeout_sec=0.05)


def joints():
    return np.array([q[j] for j in NAMES])


def send(qq, T):
    msg = JointTrajectory()
    msg.header.frame_id = TAG
    msg.joint_names = NAMES
    p = JointTrajectoryPoint()
    p.positions = [float(v) for v in qq]
    p.time_from_start = Duration(sec=int(T), nanosec=int((T % 1)*1e9))
    p.velocities = [0.0]*7
    msg.points.append(p)
    for _ in range(3):
        pub.publish(msg)
        rclpy.spin_once(n, timeout_sec=0.05)


def wait_arrival(qq, T, poll=None):
    t0 = time.time()
    while time.time()-t0 < T+8:
        rclpy.spin_once(n, timeout_sec=0.1)
        if foreign:
            print('ABORT mid-move: foreign JTC traffic; stopping here (JTC holds last target)')
            sys.exit(4)
        if mode[0] != 2:
            print(f"ABORT: robot_mode={mode[0]} (reflex?); JTC holds last target")
            sys.exit(7)
        if poll and poll():
            return 'contact'
        err = np.abs(joints()-qq).max()
        if time.time()-t0 > T and err < 0.01:
            break
    err = np.abs(joints()-qq).max()
    print(f'  arrival max|err| {err:.4f} rad')
    return 'ok' if err < 0.05 else 'fail'


t0 = time.time()
while (len(q) < 7 or pub.get_subscription_count() < 1) and time.time()-t0 < 15:
    rclpy.spin_once(n, timeout_sec=0.1)
if len(q) < 7 or pub.get_subscription_count() < 1:
    print('ABORT: no joint_states or JTC subscriber (fr3_arm_controller active?)')
    sys.exit(2)
spin(1.5)
if foreign:
    print(f'ABORT: {len(foreign)} foreign trajectories (teleop/pad active?)')
    sys.exit(3)
if mode[0] != 2:
    print(f'ABORT: robot_mode={mode[0]}')
    sys.exit(7)
def hold_now():
    """Publish current joints as an immediate hold goal (signal-safe: no spin)."""
    try:
        cur = [float(v) for v in joints()]
    except Exception:
        return False
    try:
        msg = JointTrajectory()
        msg.header.frame_id = TAG
        msg.joint_names = NAMES
        p = JointTrajectoryPoint()
        p.positions = cur
        p.time_from_start = Duration(sec=1, nanosec=0)
        p.velocities = [0.0]*7
        msg.points.append(p)
        for _ in range(5):
            pub.publish(msg)
    except Exception:
        return False
    return True


def _on_int(signum, frame):
    print('\nCtrl+C: HOLD sent (current joints); JTC stops in place. Partial:',
          json.dumps(results) if 'results' in globals() else '{}')
    hold_now()
    sys.exit(130)


signal.signal(signal.SIGINT, _on_int)

print('gripper check: OPEN to 80 mm on the dashboard now (tips assumed +-40 mm).')
print('Ctrl+C at any time = HOLD in place (no coast).')
results = {}
for k in keys:
    x, y, label = TARGETS[k]
    ans = input(f'>>> target {k} {label} ({x},{y}) [ENTER=go, q=abort] ')
    if ans.strip().lower() == 'q':
        break
    foreign.clear()
    spin(1.5)
    if foreign:
        print('ABORT: foreign traffic before move')
        sys.exit(3)
    cur = joints()
    qp, ep, er, _ = ik_down(np.array([x, y, PRE_Z]), cur)
    _, H = tcp_of(qp)
    if ep > 0.002 or er > 0.5 or tilt_of(H) > 3.0:
        print(f'ABORT {k}: IK {ep*1000:.1f} mm/{er:.2f} deg tilt {tilt_of(H):.2f}')
        sys.exit(5)
    send(qp, max(np.abs(qp-cur).max()/0.10, 8.0))
    if wait_arrival(qp, max(np.abs(qp-cur).max()/0.10, 8.0)) == 'fail':
        print(f'{k}: pre-touch arrival FAILED, stopping')
        sys.exit(6)
    spin(1.0)
    t0 = time.time()
    fz0 = []
    while time.time()-t0 < 0.5:
        rclpy.spin_once(n, timeout_sec=0.05)
        fz0.append(F[2])
    fz0 = float(np.mean(fz0))
    print(f'{k}: hover Fz base {fz0:+.1f} N, descending 5 mm steps (hand on e-stop)')
    st = [0, None]
    cur = joints()

    def poll():
        fmag = float(np.linalg.norm(F))
        if abs(F[2]-fz0) > 6.0 or fmag > 10.0:
            st[0] += 1
            if st[0] >= 2:
                st[1] = (float(F[2]), fmag)
                return True
        else:
            st[0] = 0
        return False

    for s in range(MAX_STEPS):
        tip, _ = tcp_of(cur)
        if tip[2]-STEP < FLOOR_Z:
            print(f'{k}: FLOOR reached, no contact')
            break
        qn, ep, er, _ = ik_down(tip-np.array([0, 0, STEP]), cur)
        if ep > 0.003 or er > 1.0:
            print(f'{k}: step IK bad ({ep*1000:.1f} mm), stopping descent')
            break
        foreign.clear()
        send(qn, 2.0)
        r = wait_arrival(qn, 2.0, poll=poll)
        cur = joints()
        if r == 'contact':
            break
        if r == 'fail':
            print(f'{k}: step arrival FAILED, stopping')
            sys.exit(6)
    else:
        print(f'{k}: STEPS exhausted, no contact')
    send(cur+np.zeros(7), 0.5)
    contact = st[1]
    tip, H = tcp_of(cur)
    tips = [tip+H[:3, 1]*0.04, tip-H[:3, 1]*0.04]
    if contact:
        print(f'{k} CONTACT: tcp z {tip[2]:+.4f} m, lower tip z {min(t[2] for t in tips):+.4f} m, '
              f'Fz {contact[0]:+.1f} N |F| {contact[1]:.1f} N')
        results[k] = {'tcp_z': float(tip[2]), 'tip_z': float(min(t[2] for t in tips)),
                      'Fz': contact[0], 'F': contact[1]}
    else:
        results[k] = {'tcp_z': None}
        print(f'{k}: NO CONTACT recorded')
    send(joints(), 0.5)
    qp, _, _, _ = ik_down(np.array([x, y, PRE_Z]), joints())
    send(qp, 4.0)
    wait_arrival(qp, 4.0)
fn = f'/tmp/touch_probe_{time.strftime("%H%M%S")}.json'
json.dump({'targets': TARGETS, 'results': results}, open(fn, 'w'))
print('results:', json.dumps(results), '\nfile', fn)
print('done: re-home with the pad or HOME mode; paste the results here')
