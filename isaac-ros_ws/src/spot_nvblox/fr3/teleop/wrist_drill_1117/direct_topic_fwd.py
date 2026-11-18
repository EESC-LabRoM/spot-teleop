"""Single-domain gated forwarder (domain 77). TEST ONLY per Murilo.

/fr3_joystick_mpc/joint_preview (JointTrajectory, one-point) ->
/fr3_arm_controller/joint_trajectory, ONLY while /teleop/engaged is True
(pad Start held). Pad released -> silent, arm holds. No tracker, no map,
and no bridge in this path: the only shaping is the 0.03 rad/packet slew
clip (same value vm_real.sh always used) plus non-finite refusal.
"""
import argparse
import math
import time,json,collections
from mpc_packet_gate import SocketGate

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
packet_gate=SocketGate()
engaged = {'v': False, 'seen': False}
st = {'last': None, 'fwd': 0, 'rej': 0, 't': 0.0, 'meas': {}, 'armed': False}
# 2026-11-05 grasp lane fix: 'last' used to survive a disengage, so the first packet after the next
# Start was slewed (1.5 rad/s) from the PREVIOUS session's pose -> joint_velocity_violation reflexes
# at 20:20:24Z, 20:21:54Z, 21:10:37Z. Now: seed 'last' from the MEASURED joints on every engage edge,
# clear it on disengage, and refuse (stay silent) if the MPC's command is > FIRST_MAX from measured.
FIRST_MAX = 0.15
st['t'] = time.monotonic()
st['measured_at']=0.;st['preview_ready']=False;st['preview_at']=0.;st['seed_deviation']=float('inf')
reasons=collections.Counter()
def refuse(reason):
    st['rej']+=1;reasons[reason]+=1
def status():
    now=time.monotonic();data=dict(wall=time.time(),engaged=engaged['v'],measured_age_s=now-st['measured_at'],preview_age_s=now-st['preview_at'],metadata_ok=st['preview_ready'],seed_deviation=st['seed_deviation'],ready=st['preview_ready'] and now-st['preview_at']<.2 and now-st['measured_at']<.2 and st['seed_deviation']<FIRST_MAX,forwarded=st['fwd'],refused=st['rej'],reasons=dict(reasons))
    with open('/tmp/wrist_forwarder_status.json','w') as f:json.dump(data,f)



def startup_blend(q, v, seed, elapsed, duration=1.0):
    u=max(0.,min(1.,elapsed/duration))
    alpha=10*u**3-15*u**4+6*u**5
    rate=(30*u**2-60*u**3+30*u**4)/duration
    return ([a+alpha*(b-a) for a,b in zip(seed,q)],
            [alpha*x+rate*(b-a) for a,b,x in zip(seed,q,v)])


def on_js(m):
    measured=dict(zip(m.name,m.position))
    needed=[f'fr3_joint{i}' for i in range(1,8)]
    age=(n.get_clock().now().nanoseconds-(m.header.stamp.sec*10**9+m.header.stamp.nanosec))*1e-9
    if all(k in measured and math.isfinite(measured[k]) for k in needed) and 0<=age<.2:
        st['meas']=measured;st['measured_at']=time.monotonic()-age


def on_engaged(m):
    v = bool(m.data)
    if v != engaged['v'] or not engaged['seen']:
        st['last'] = None          # new session: never slew from an old pose
        st['armed'] = False
        st['ramp_start'] = None
        st['seed'] = None
    engaged['v'] = v
    engaged['seen'] = True


def slew(q, last, max_step):
    out = [y + max(-max_step, min(max_step, x - y)) for x, y in zip(q, last)]
    return out, any(abs(x - o) > 1e-12 for x, o in zip(q, out))


def on_preview(m):
    import time
    if not m.points:
        refuse('empty_preview')
        return
    p=m.points[0];q=[float(x) for x in p.positions]
    metadata_ok=packet_gate.accept(list(m.joint_names),q)
    st['preview_ready']=metadata_ok;st['preview_at']=time.monotonic()
    st['seed_deviation']=max((abs(x-st['meas'][k]) for k,x in zip(m.joint_names,q)),default=float('inf')) if all(k in st['meas'] for k in m.joint_names) else float('inf')
    if not engaged['seen'] or not engaged['v']:
        return  # pad released: hold position, forward nothing
    p = m.points[0]
    q = [float(x) for x in p.positions]
    if not metadata_ok:
        refuse('metadata')
        n.get_logger().warn('MPC preview missing fresh matching feasible metadata; holding',throttle_duration_sec=2.)
        return
    if time.monotonic()-st['measured_at']>.2:
        refuse('joint_state_stale');return
    if st['last'] is None:
        meas = [st['meas'].get(j) for j in m.joint_names]
        if any(x is None for x in meas):
            refuse('missing_joint')
            return                  # no measured joints yet: forward nothing
        dev = max(abs(x - y) for x, y in zip(q, meas))
        if dev > FIRST_MAX:
            refuse('seed_distance')
            if not st['armed']:
                n.get_logger().warn('engage refused: MPC command %.3f rad from measured joints (> %.2f); '
                                    'not forwarding (re-activate the MPC from the measured pose)' % (dev, FIRST_MAX))
                st['armed'] = True
            return
        st['last'] = list(meas)     # slew starts at the measured pose
        st['seed'] = list(meas)
        st['ramp_start'] = time.monotonic()
    v = [float(x) for x in p.velocities] if p.velocities else [0.0] * len(q)
    if any(not math.isfinite(x) for x in q + v):
        refuse('nonfinite')
        return
    if st.get('ramp_start') is not None:
        q, v = startup_blend(q, v, st['seed'], time.monotonic()-st['ramp_start'])
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
n.create_timer(.2,status)
try:rclpy.spin(n)
finally:status()
