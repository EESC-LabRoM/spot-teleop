#!/usr/bin/env python3
"""Execute ONE stage of a grasp_plan.py plan on the FR3 JTC (2026-11-05).
Runs inside fr3-ros2 (jazzy, domain 77).   python3 grasp_stage_exec.py PLAN.json STAGE
Guards (same as the calibration mover): refuses if anyone else publishes on
/fr3_arm_controller/joint_trajectory in the 1.5 s before; if a foreign
trajectory appears mid-stage it stops publishing and exits 4 (JTC keeps its
last target). Refuses if the start joints are > 0.10 rad from where the stage
expects to start (previous stage's end), so stages cannot be run out of order.
Arrival bar is 0.05 rad max error (wrist stiction after long slews).
"""
import json
import sys
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

plan = json.load(open(sys.argv[1]))
stage = sys.argv[2]
# 2026-11-06: optional arrival bar (rad); the lift/put-back legs sag ~0.05 under a held object
BAR = float(sys.argv[3]) if len(sys.argv) > 3 else 0.05
names = list(plan['stages'])
# = lab ./fr3_ros2.sh real-home / dashboard Home / pad X pose (fr3_home_jtc.py HOME, Paiva motion.yaml);
# was Murilo's [-0.0024,-0.6809,-0.1069,-2.3397,-0.0077,1.5728,0.7784] (0.107 rad off -> S1 refused after real-home)
Q_HOME = np.array([0.0, -0.7853981634, 0.0, -2.3561944902, 0.0, 1.5707963268, 0.7853981634])
if stage != 'HOME':
    st = plan['stages'][stage]
    Q = [np.array(q) for q in st['q']]
    TT = st['t']
    k = names.index(stage)
    expect_start = Q_HOME if k == 0 else np.array(plan['stages'][names[k - 1]]['q'][-1])
NAMES = [f'fr3_joint{i}' for i in range(1, 8)]
TAG = 'grasp_stage_exec'

rclpy.init()
n = Node('grasp_stage_exec')
q, foreign = {}, []
n.create_subscription(JointState, '/joint_states', lambda m: q.update(zip(m.name, m.position)), 10)
pub = n.create_publisher(JointTrajectory, '/fr3_arm_controller/joint_trajectory', 10)
n.create_subscription(JointTrajectory, '/fr3_arm_controller/joint_trajectory',
                      lambda m: foreign.append(time.time()) if m.header.frame_id != TAG else None, 10)
t0 = time.time()
while (len(q) < 7 or pub.get_subscription_count() < 1) and time.time() - t0 < 10:
    rclpy.spin_once(n, timeout_sec=0.1)
if len(q) < 7 or pub.get_subscription_count() < 1:
    print('ABORT: no joint_states or no JTC subscriber (is fr3_arm_controller active?)'); sys.exit(2)
t0 = time.time()
while time.time() - t0 < 1.5:
    rclpy.spin_once(n, timeout_sec=0.05)
if foreign:
    print(f'ABORT: {len(foreign)} foreign trajectories on the JTC topic (teleop/pad/forwarder active?)'); sys.exit(3)
cur = np.array([q[j] for j in NAMES])
if stage == 'HOME':
    # HOME from anywhere in this plan: if near a stage end, first retreat to PRE (S1 end,
    # 12 cm above the grasp along the approach), then PRE -> home (= S7, clearance-checked).
    # Refuses if the arm is not near home or any stage end (unknown pose: use the pad).
    q_pre = np.array(plan['stages']['S1_home_to_pre']['q'][-1])
    last = names[-1]
    ends = {nm: np.array(plan['stages'][nm]['q'][-1]) for nm in names}
    near = min(ends, key=lambda nm: np.abs(cur - ends[nm]).max())
    dnear = np.abs(cur - ends[near]).max()
    if np.abs(cur - Q_HOME).max() < 0.02:
        print('HOME: already at home'); sys.exit(0)
    if dnear > 0.15 and np.abs(cur - Q_HOME).max() > 0.15:
        print(f'ABORT HOME: arm is {dnear:.3f} rad from the nearest plan pose ({near}); not a known pose, bring it home with the pad'); sys.exit(5)
    Q, TT, t = [], [], 0.0
    prev = cur
    via = [q_pre, Q_HOME] if np.abs(cur - q_pre).max() > 0.02 and dnear <= 0.15 and near != last else [Q_HOME]
    for qq in via:
        t += max(np.abs(qq - prev).max() / 0.10, 6.0)
        Q.append(qq); TT.append(round(t, 3)); prev = qq
    print(f'HOME: nearest plan pose {near} ({dnear:.3f} rad); via {"PRE -> " if len(via) == 2 else ""}home', flush=True)
    expect_start = cur
dev = np.abs(cur - expect_start).max()
if dev > 0.10 and stage != 'HOME' and np.abs(cur - Q[-1]).max() < 0.20 and len(Q) == 1:
    # already close to this stage's target (e.g. resuming after a re-plan with --depth): short correction move
    print(f'NOTE: {dev:.3f} rad from the expected start, but only {np.abs(cur - Q[-1]).max():.3f} rad from the {stage} target: short correction move')
    dev = 0.0
if dev > 0.10:
    print(f'ABORT: arm is {dev:.3f} rad from where {stage} must start (run the stages in order)'); sys.exit(5)

def run_segment(Qs, Ts, label):
    """Publish one JTC trajectory and wait for arrival; returns the final max|err|."""
    msg = JointTrajectory()
    msg.header.frame_id = TAG
    msg.joint_names = NAMES
    for qq, t in zip(Qs, Ts):
        p = JointTrajectoryPoint()
        p.positions = [float(v) for v in qq]
        p.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
        msg.points.append(p)
    msg.points[-1].velocities = [0.0] * 7
    for _ in range(3):
        pub.publish(msg)
        rclpy.spin_once(n, timeout_sec=0.05)
    T = Ts[-1]
    print(f'{label}: {len(Qs)} waypoints over {T:.1f} s', flush=True)
    t0 = time.time()
    while time.time() - t0 < T + 8:
        rclpy.spin_once(n, timeout_sec=0.1)
        if foreign:
            print(f'ABORT mid-stage: foreign trajectories ({len(foreign)}); this script stops publishing'); sys.exit(4)
        err = np.abs(np.array([q[j] for j in NAMES]) - Qs[-1]).max()
        if time.time() - t0 > T and err < 0.01:
            break
    err = np.abs(np.array([q[j] for j in NAMES]) - Qs[-1]).max()
    print(f'{label} done: max|err| {err:.4f} rad', flush=True)
    return err


if stage == 'HOME':
    # one single-waypoint trajectory per leg (multi-point JTC trajectories track badly here)
    t_prev = 0.0
    for i_leg, (qq, t) in enumerate(zip(Q, TT)):
        err = run_segment([qq], [t - t_prev], 'HOME leg %d/%d (%s)' % (i_leg + 1, len(Q), 'PRE' if i_leg < len(Q) - 1 else 'home'))
        t_prev = t
        if err >= 0.05:
            sys.exit(6)
    sys.exit(0)
err = run_segment(Q, TT, stage)
sys.exit(0 if err < BAR else 6)
