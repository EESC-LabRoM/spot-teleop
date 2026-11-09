#!/usr/bin/env python3
"""Direct goal relay: /teleop/command_tool -> /fr3_joystick_mpc/wrist_goal.

Restores the joystick->planner leg the retired hardware bridge used to own
(it subscribed command_tool and fed the MPC; without it the planner holds
its last goal and the stick does nothing). No bridge process involved.

Forwarding rule (mirrors the retired bridge's freshness gates):
  - engaged must be True (pad Start held),
  - command frame must be fr3_link0,
  - command stamp at most 0.25 s old.
Otherwise silent: the planner keeps holding. Re-stamps on publish.

Frame (fix 2026-11-06, Murilo's pad test): /teleop/command_tool is the fr3_hand_tcp pose (teleop tool_frame), the
MPC's ee_link is fr3_hand. The retired bridge converted it (goal_math.tcp_to_hand with TF fr3_hand_tcp->fr3_hand);
this relay forwarded the raw TCP pose, so the pad goal sat 10.34 cm off along the approach axis while the 6D grasp
target is a true fr3_hand pose: the attraction blended two different frames. Now converted the same way; nothing is
forwarded until that fixed transform has been read from TF.
"""
import argparse

GOAL_TOPIC = '/fr3_joystick_mpc/wrist_goal'
SRC_TOPIC = '/teleop/command_tool'
WANT_FRAME = 'fr3_link0'
FRESH_S = 0.25


def fresh_enough(now_ns, stamp_ns, window_s=FRESH_S):
    """True if 0 <= now - stamp <= window (same bounds the bridge used)."""
    dt = (now_ns - stamp_ns) * 1e-9
    return 0.0 <= dt <= window_s


def should_forward(engaged, frame_id, now_ns, stamp_ns):
    """Pure forward gate. Returns (ok, reason)."""
    if not engaged:
        return False, 'idle: waiting for joystick Start'
    if frame_id != WANT_FRAME:
        return False, 'wrong frame %r (want %r)' % (frame_id, WANT_FRAME)
    if not fresh_enough(now_ns, stamp_ns):
        return False, 'command stale'
    return True, 'forwarding joystick goal'


def quat_mul(a, b):                                    # [x, y, z, w]
    x, y, z, w = a; u, v, s, t = b
    return [w*u + x*t + y*s - z*v, w*v - x*s + y*t + z*u, w*s + x*v - y*u + z*t, w*t - x*u - y*v - z*s]


def tcp_to_hand(p, q, t, r):
    """Hand pose from the TCP pose (p, q xyzw) and the fixed hand-in-TCP transform (t, r). Same as goal_math."""
    qv, w = q[:3], q[3]
    c1 = [qv[1]*t[2] - qv[2]*t[1] + w*t[0], qv[2]*t[0] - qv[0]*t[2] + w*t[1], qv[0]*t[1] - qv[1]*t[0] + w*t[2]]
    c2 = [qv[1]*c1[2] - qv[2]*c1[1], qv[2]*c1[0] - qv[0]*c1[2], qv[0]*c1[1] - qv[1]*c1[0]]
    return [p[i] + t[i] + 2*c2[i] for i in range(3)], quat_mul(q, r)


def main():
    import time
    import math
    import rclpy
    import tf2_ros
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Bool

    ap = argparse.ArgumentParser()
    ap.add_argument('--rate', type=float, default=30.0)
    a = ap.parse_args()

    rclpy.init()
    n = rclpy.create_node('direct_goal_fwd')
    pub = n.create_publisher(PoseStamped, GOAL_TOPIC, 10)
    box = {'m': None, 'engaged': False, 'T': None}
    tfb = tf2_ros.Buffer()
    tf2_ros.TransformListener(tfb, n)

    def on_cmd(m):
        box['m'] = m

    def on_eng(m):
        box['engaged'] = bool(m.data)

    n.create_subscription(PoseStamped, SRC_TOPIC, on_cmd, 10)
    n.create_subscription(Bool, '/teleop/engaged', on_eng, 10)
    print('direct_goal_fwd: %s (fr3_hand_tcp) -> %s (fr3_hand) while engaged' % (SRC_TOPIC, GOAL_TOPIC), flush=True)
    dt = 1.0 / a.rate
    fwd = rej = 0
    t0 = time.monotonic()
    while rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.0)
        if box['T'] is None:
            try:
                tr = tfb.lookup_transform('fr3_hand_tcp', 'fr3_hand', rclpy.time.Time()).transform
                t, r = tr.translation, tr.rotation
                box['T'] = ([t.x, t.y, t.z], [r.x, r.y, r.z, r.w])
                n.get_logger().info('hand in TCP frame: t=%s r=%s (|t| %.4f m)' % (
                    [round(v, 4) for v in box['T'][0]], [round(v, 4) for v in box['T'][1]], math.dist(box['T'][0], [0, 0, 0])))
            except Exception:
                time.sleep(dt)
                continue
        m = box['m']
        # Same clock the teleop stamps with (ROS clock, sim-time aware).
        # Wall/monotonic time here silently fails the freshness check forever.
        now_ns = n.get_clock().now().nanoseconds
        if m is not None:
            stamp = m.header.stamp.sec * 10 ** 9 + m.header.stamp.nanosec
            ok, _ = should_forward(box['engaged'], m.header.frame_id, now_ns, stamp)
            if ok:
                p, q = m.pose.position, m.pose.orientation
                hp, hq = tcp_to_hand([p.x, p.y, p.z], [q.x, q.y, q.z, q.w], *box['T'])
                g = PoseStamped()
                g.header.frame_id = m.header.frame_id
                g.header.stamp = n.get_clock().now().to_msg()
                g.pose.position.x, g.pose.position.y, g.pose.position.z = hp
                g.pose.orientation.x, g.pose.orientation.y, g.pose.orientation.z, g.pose.orientation.w = hq
                pub.publish(g)
                fwd += 1
            else:
                rej += 1
        if time.monotonic() - t0 > 10:
            t0 = time.monotonic()
            n.get_logger().info('forwarded %d, held %d' % (fwd, rej))
        time.sleep(dt)


if __name__ == '__main__':
    main()
