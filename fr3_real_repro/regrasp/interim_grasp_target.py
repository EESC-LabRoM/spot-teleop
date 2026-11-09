#!/usr/bin/env python3
"""INTERIM /grasp_target publisher (2026-11-06), until regrasp/loop.py publishes. Domain 8 (VM), run in vm-viewer.

Publishes the PLANNED fr3_hand pose (plan.json from grasp_plan_lib: grasp_pos + hand_R, frame fr3_link0) at 10 Hz for the
cuRobo MPC 6D attraction (pf_mode:=grasp, via regrasp/lab/grasp_target_relay_d77.py). HELD_OBJECT semantics:
  - publishes only while the gripper is OPEN (width >= --open-min, from /joint_states fr3_finger_joint1+2)
    and /tmp/regrasp.lock (--lock, repo path visible in vm-viewer) is not HELD_OBJECT;
  - EXITS for good at the first close (width drops below --close-below after having been open), so the MPC field
    releases (pf_grasp_timeout_s 0.5) and never fights the lift; restart it by hand for the next grasp;
  - exits after --max-min minutes.
  python3 interim_grasp_target.py PLAN.json [--rate 10] [--max-min 20]
"""
import argparse
import json
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

ap = argparse.ArgumentParser()
ap.add_argument('plan')
ap.add_argument('--rate', type=float, default=10.0)
ap.add_argument('--max-min', type=float, default=20.0)
ap.add_argument('--open-min', type=float, default=0.075, help='m, gripper counts as open above this')
ap.add_argument('--close-below', type=float, default=0.070, help='m, first drop below this after open = close -> exit')
ap.add_argument('--lock', default='/home/nexus/spot-teleop/fr3_real_repro/regrasp/state/regrasp.lock')
ap.add_argument('--topic', default='/grasp_target')
a = ap.parse_args()


def R2q(R):
    t = np.trace(R)
    if t > 0:
        s = 2 * np.sqrt(t + 1)
        return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, s / 4])
    i = int(np.argmax(np.diag(R))); j, k = (i + 1) % 3, (i + 2) % 3
    s = 2 * np.sqrt(1 + R[i, i] - R[j, j] - R[k, k]); q = np.zeros(4)
    q[i] = s / 4; q[j] = (R[j, i] + R[i, j]) / s; q[k] = (R[k, i] + R[i, k]) / s; q[3] = (R[k, j] - R[j, k]) / s
    return q / np.linalg.norm(q)  # x, y, z, w


plan = json.load(open(a.plan))
assert plan.get('problems') == [], f'plan has problems: {plan.get("problems")}'
assert 'hand_R' in plan, 'plan has no hand_R (make it with grasp_plan_lib / the current grasp_plan.py)'
P = np.array(plan['grasp_pos'], float)
Q = R2q(np.array(plan['hand_R'], float))


def lock_state():
    try:
        return json.load(open(a.lock)).get('state', 'NONE')
    except (OSError, ValueError):
        return 'NONE'


class Pub(Node):
    def __init__(self):
        super().__init__('interim_grasp_target')
        self.pub = self.create_publisher(PoseStamped, a.topic, QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                                                                          history=HistoryPolicy.KEEP_LAST))
        self.create_subscription(JointState, '/joint_states', self.on_js, 10)
        self.width, self.was_open, self.n, self.t0 = None, False, 0, time.time()
        self.done = None
        self.create_timer(1.0 / a.rate, self.tick)
        self.get_logger().info(f'interim /grasp_target from {a.plan}: p {np.round(P, 4).tolist()} q(xyzw) {np.round(Q, 4).tolist()}')

    def on_js(self, m):
        d = dict(zip(m.name, m.position))
        if 'fr3_finger_joint1' in d and 'fr3_finger_joint2' in d:
            self.width = d['fr3_finger_joint1'] + d['fr3_finger_joint2']
            if self.width >= a.open_min:
                self.was_open = True
            elif self.was_open and self.width < a.close_below:
                self.done = f'gripper closing (width {self.width*1000:.0f} mm): HELD_OBJECT, target released for good'

    def tick(self):
        if self.done is None and lock_state() == 'HELD_OBJECT':
            self.done = 'lock HELD_OBJECT'
        if self.done is None and time.time() - self.t0 > a.max_min * 60:
            self.done = f'max lifetime {a.max_min} min'
        if self.done:
            return
        if self.width is None or self.width < a.open_min:
            return  # not open (yet): publish nothing
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'fr3_link0'
        m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in P)
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = (float(v) for v in Q)
        self.pub.publish(m)
        self.n += 1
        if self.n in (1, 10) or self.n % 600 == 0:
            self.get_logger().info(f'published {self.n} (width {self.width*1000:.0f} mm)')


rclpy.init()
node = Pub()
while rclpy.ok() and node.done is None:
    rclpy.spin_once(node, timeout_sec=0.1)
node.get_logger().info(f'EXIT after {node.n} msgs: {node.done}')
node.destroy_node()
rclpy.shutdown()
