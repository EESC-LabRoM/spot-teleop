"""Single-domain gated forwarder (domain 77). TEST ONLY per Murilo.

/fr3_joystick_mpc/joint_preview (JointTrajectory, one-point) ->
/fr3_arm_controller/joint_trajectory, ONLY while /teleop/engaged is True
(pad Start held). Pad released -> silent, arm holds. No tracker, no map,
and no bridge in this path: the only shaping is the 0.03 rad/packet slew
clip (same value vm_real.sh always used) plus non-finite refusal.
"""
import argparse
import math

import rclpy
from builtin_interfaces.msg import Duration
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ap = argparse.ArgumentParser()
ap.add_argument('--topic-in', default='/fr3_joystick_mpc/joint_preview')
ap.add_argument('--topic-out', default='/fr3_arm_controller/joint_trajectory')
ap.add_argument('--period', type=float, default=0.02)
ap.add_argument('--max-step', type=float, default=0.03)
a = ap.parse_args()

rclpy.init()
n = rclpy.create_node('direct_topic_fwd')
pub = n.create_publisher(JointTrajectory, a.topic_out, 10)
engaged = {'v': False, 'seen': False}
st = {'last': None, 'fwd': 0, 'rej': 0, 't': 0.0, 'meas': {}, 'armed': False}
# 2026-11-05 grasp lane fix: 'last' used to survive a disengage, so the first packet after the next
# Start was slewed (1.5 rad/s) from the PREVIOUS session's pose -> joint_velocity_violation reflexes
# at 20:20:24Z, 20:21:54Z, 21:10:37Z. Now: seed 'last' from the MEASURED joints on every engage edge,
# clear it on disengage, and refuse (stay silent) if the MPC's command is > FIRST_MAX from measured.
FIRST_MAX = 0.05
st['t'] = __import__('time').monotonic()


def on_js(m):
    st['meas'].update(zip(m.name, m.position))


def on_engaged(m):
    v = bool(m.data)
    if v != engaged['v'] or not engaged['seen']:
        st['last'] = None          # new session: never slew from an old pose
        st['armed'] = False
    engaged['v'] = v
    engaged['seen'] = True


def slew(q, last, max_step):
    out = [y + max(-max_step, min(max_step, x - y)) for x, y in zip(q, last)]
    return out, any(abs(x - o) > 1e-12 for x, o in zip(q, out))


def on_preview(m):
    import time
    if not m.points:
        st['rej'] += 1
        return
    if not engaged['seen'] or not engaged['v']:
        return  # pad released: hold position, forward nothing
    p = m.points[0]
    q = [float(x) for x in p.positions]
    if st['last'] is None:
        meas = [st['meas'].get(j) for j in m.joint_names]
        if any(x is None for x in meas):
            st['rej'] += 1
            return                  # no measured joints yet: forward nothing
        dev = max(abs(x - y) for x, y in zip(q, meas))
        if dev > FIRST_MAX:
            st['rej'] += 1
            if not st['armed']:
                n.get_logger().warn('engage refused: MPC command %.3f rad from measured joints (> %.2f); '
                                    'not forwarding (re-activate the MPC from the measured pose)' % (dev, FIRST_MAX))
                st['armed'] = True
            return
        st['last'] = list(meas)     # slew starts at the measured pose
    v = [float(x) for x in p.velocities] if p.velocities else [0.0] * len(q)
    if any(not math.isfinite(x) for x in q + v):
        st['rej'] += 1
        return
    if st['last'] is not None:
        q, _ = slew(q, st['last'], a.max_step)
    st['last'] = list(q)
    out = JointTrajectory()
    out.joint_names = list(m.joint_names)
    pt = JointTrajectoryPoint()
    pt.positions, pt.velocities = q, v
    pt.time_from_start = Duration(sec=0, nanosec=int(a.period * 1e9))
    out.points = [pt]
    pub.publish(out)
    st['fwd'] += 1
    if time.monotonic() - st['t'] > 10:
        st['t'] = time.monotonic()
        n.get_logger().info('forwarded %d, refused %d' % (st['fwd'], st['rej']))


n.create_subscription(Bool, '/teleop/engaged', on_engaged, 10)
n.create_subscription(JointState, '/joint_states', on_js, 10)
n.create_subscription(JointTrajectory, a.topic_in, on_preview, 10)
print('direct_topic_fwd: %s -> %s while engaged; silent until Start' % (a.topic_in, a.topic_out), flush=True)
rclpy.spin(n)
