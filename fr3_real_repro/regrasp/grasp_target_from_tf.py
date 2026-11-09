#!/usr/bin/env python3
"""/grasp_target = target_object TF + the bound grasp offset (2026-11-06, Murilo: "instead of just keeping the TF
publishing, publish the 6D target, the grasp"). Domain 8, run in vm-viewer.

The existing tracker pipeline (vm-tracker-prod sam2_tracker -> /tracking_3d_point -> vm-target-bridge) broadcasts
fr3_link0 -> target_object at 20 Hz and LATCHES the last point through occlusion. This node rides on it:
  /grasp_target.position    = TF(target_object) + binding.offset
  /grasp_target.orientation = binding.hand_R (fr3_hand, the MPC ee_link)
published at --rate Hz, so the 6D target never drops out on occlusion. The binding (regrasp/state/grasp_binding.json)
is written when a grasp is planned: offset = plan grasp_pos - target_object at that moment. The regrasp loop
rewrites it on REGEN/REUSE (rotation changes come from a rebind; the TF is position-only).

Gating (HELD_OBJECT semantics, the MPC field must never fight the lift):
  - publishes only while the gripper is open (width >= --open-min, /joint_states fingers);
  - at a close (width < --close-below after open) it goes QUIET while closed and resumes the SAME binding as soon as
    the gripper is open again (2026-11-06: the object is the same, the tracker follows it; it used to wait 30-75 s
    for the loop's next binding, which felt like "sometimes it doesn't attract");
  - lock HELD_OBJECT (scripted grasp) keeps it quiet until a NEW binding arrives (different 'bound_at').
  - quiet if the binding is missing/invalid.
Jump gate (object_gate.py, 2026-11-06 21:21Z pad test): the object position starts at the binding's
target_object_at_bind and only follows the TF when the arm is still for ~1 s, the TF is stable and the hand is not
near the grasp; TF jumps while the arm moves (tracker on the table/arm) are ignored, so the 6D target cannot snap.
  python3 grasp_target_from_tf.py [--binding PATH] [--rate 20]
"""
import argparse
import json
import time

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

from object_gate import ObjectGate, gate_yaw
from publish import apply_yaw_to_grasp, select_yaw_rel

ST = '/home/nexus/spot-teleop/fr3_real_repro/regrasp/state'
ap = argparse.ArgumentParser()
ap.add_argument('--binding', default=f'{ST}/grasp_binding.json')
ap.add_argument('--yaw', default=f'{ST}/live_yaw.json',
                help='loop yaw feed; rotates offset+hand_R so table yaws ride without rebinding')
ap.add_argument('--lock', default=f'{ST}/regrasp.lock')
ap.add_argument('--rate', type=float, default=20.0)
ap.add_argument('--open-min', type=float, default=0.075)
ap.add_argument('--close-below', type=float, default=0.070)
ap.add_argument('--max-offset', type=float, default=0.35, help='m, refuse bindings with a larger hand-to-target offset')
ap.add_argument('--topic', default='/grasp_target')
a = ap.parse_args()


def R2q(R):
    t = np.trace(R)
    if t > 0:
        s = 2 * np.sqrt(t + 1)
        q = np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, s / 4])
    else:
        i = int(np.argmax(np.diag(R))); j, k = (i + 1) % 3, (i + 2) % 3
        s = 2 * np.sqrt(1 + R[i, i] - R[j, j] - R[k, k]); q = np.zeros(4)
        q[i] = s / 4; q[j] = (R[j, i] + R[i, j]) / s; q[k] = (R[k, i] + R[i, k]) / s; q[3] = (R[k, j] - R[j, k]) / s
    return q / np.linalg.norm(q)  # x, y, z, w


def read_json(path):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


class GraspTargetFromTF(Node):
    def __init__(self):
        super().__init__('grasp_target_from_tf')
        self.pub = self.create_publisher(PoseStamped, a.topic, QoSProfile(
            depth=5, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST))
        self.buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buf, self)
        self.create_subscription(JointState, '/joint_states', self.on_js, 10)
        self.width, self.was_open = None, False
        self.binding, self.bound_at, self.q = None, None, None
        self.yaw_rel, self.yaw_logged = 0.0, None
        self.gate, self.gate_reason = None, None
        self.quiet_for = None      # bound_at of the binding we went quiet on (closed / HELD_OBJECT)
        self.held_lock = False     # lock HELD_OBJECT (scripted grasp): stays quiet until a new binding, even if reopened
        self.n, self.last_state, self.t_log = 0, None, 0.0
        self.create_timer(1.0 / a.rate, self.tick)
        self.get_logger().info(f'/grasp_target = TF target_object + binding ({a.binding}), {a.rate:.0f} Hz')

    def on_js(self, m):
        d = dict(zip(m.name, m.position))
        if 'fr3_finger_joint1' in d and 'fr3_finger_joint2' in d:
            self.width = d['fr3_finger_joint1'] + d['fr3_finger_joint2']
            if self.width >= a.open_min:
                if self.quiet_for is not None and not self.held_lock:
                    # reopened after a close (Murilo 11-06: "the grasp was already generated once ... the object is the
                    # same"): resume the SAME binding at once, riding on the tracked TF; no wait for a new grasp
                    self.get_logger().info('gripper reopened: resuming the current binding')
                    self.quiet_for = None
                self.was_open = True
            elif self.was_open and self.width < a.close_below and self.bound_at is not None:
                self.quiet_for, self.was_open = self.bound_at, False   # closing: release until a new binding

    def load_binding(self):
        b = read_json(a.binding)
        if not b or b.get('bound_at') == self.bound_at:
            return
        try:
            off = np.array(b['offset'], float); R = np.array(b['hand_R'], float)
            assert off.shape == (3,) and R.shape == (3, 3) and np.linalg.norm(off) <= a.max_offset
            assert abs(np.linalg.det(R) - 1) < 1e-3 and np.allclose(R.T @ R, np.eye(3), atol=1e-3)
        except (KeyError, ValueError, AssertionError):
            self.get_logger().warn(f'invalid binding ignored: {str(b)[:200]}', throttle_duration_sec=10.0)
            return
        self.binding, self.bound_at, self.q = b, b.get('bound_at'), R2q(R)
        self.held_lock = False
        self.yaw_rel = 0.0         # a fresh binding already includes the yaw
        self.gate = ObjectGate(b.get('target_object_at_bind') or b['hand_pos'] - off, off)
        self.gate_reason = None
        self.get_logger().info(f'new binding {b.get("run_id")} bound_at {self.bound_at}: offset {np.round(off, 4).tolist()}')

    def state(self):
        if self.binding is None:
            return 'no binding'
        if self.quiet_for is not None and self.quiet_for == self.bound_at:
            return 'released (closed; waiting for a new binding)'
        lk = read_json(a.lock)
        if lk and lk.get('state') == 'HELD_OBJECT':
            self.quiet_for, self.held_lock = self.bound_at, True
            return 'released (lock HELD_OBJECT)'
        if self.width is None or self.width < a.open_min:
            return 'gripper not open'
        return 'publishing'

    def lookup(self, child):
        try:
            t = self.buf.lookup_transform('fr3_link0', child, rclpy.time.Time()).transform.translation
            return np.array([t.x, t.y, t.z])
        except Exception:
            return None

    def tick(self):
        self.load_binding()
        st = self.state()
        p = None
        if st == 'publishing':
            tf, hand = self.lookup('target_object'), self.lookup('fr3_hand')
            held, why = self.gate.update(time.monotonic(), tf, hand)
            rel = select_yaw_rel(read_json(a.yaw), self.binding.get('run_id'),
                                 self.bound_at, self.yaw_rel, time.time())
            rel = gate_yaw(rel, self.yaw_rel, why)   # yaw only moves on gate-trusted ticks, like the position
            if abs(rel - self.yaw_rel) > np.radians(2.0) or \
                    (rel == 0.0) != (self.yaw_rel == 0.0):
                self.get_logger().info(
                    f'yaw ride: {np.degrees(self.yaw_rel):.1f} -> {np.degrees(rel):.1f} deg')
            self.yaw_rel = rel
            off, self.q = apply_yaw_to_grasp(self.binding['offset'],
                                             self.binding['hand_R'], rel)
            p = held + np.array(off, float)
            kind = why
            if why == 'updated':
                self.get_logger().info(f'object position updated to {np.round(held, 4).tolist()} (arm still, TF stable)')
            elif kind != self.gate_reason and why not in ('hold: collecting',):
                self.get_logger().info(f'gate: {why}' + (f' (TF {np.round(tf, 3).tolist()})' if tf is not None else ''),
                                       throttle_duration_sec=1.0)
            self.gate_reason = kind
        if st != self.last_state or time.time() - self.t_log > 30:
            self.get_logger().info(f'state: {st} (sent {self.n})' + (f' target {np.round(p, 4).tolist()}' if p is not None else ''))
            self.last_state, self.t_log = st, time.time()
        if p is None:
            return
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'fr3_link0'
        m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in p)
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = (float(v) for v in self.q)
        self.pub.publish(m)
        self.n += 1


rclpy.init()
rclpy.spin(GraspTargetFromTF())
