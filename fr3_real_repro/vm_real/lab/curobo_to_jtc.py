"""cuRobo MPC output (sensor_msgs/JointState, ~50 Hz) -> one-point trajectory_msgs/JointTrajectory for the native
JointTrajectoryController (fr3_arm_controller): the command's positions + velocities, reached time_from_start = period
after receipt (header.stamp 0 = start now). Two halves so the publisher can run where the driver's data path works:
  --in ros  --out ros   one process, same domain (sim)
  --in ros  --out udp   subscriber side (MPC's domain): forwards to 127.0.0.1:<port>
  --in udp  --out ros   publisher side (docker exec inside fr3-ros2 on the robot)
Refuses non-finite values. Two limits can shape what is forwarded (limit() is OFF by default, enable with --max-accel 10), neither ever stops the stream (no feasibility, freshness or
arming gate): limit() bounds the joint ACCELERATION of the forwarded command (--max-accel rad/s^2, 0 = off; a bad packet becomes
a short ramp, a stall catch-up a smooth one) and slew() is the hard backstop on joint steps above --max-step rad per command.
python3 curobo_to_jtc.py --in ros --out ros [--period 0.02] [--topic-in /curobo/joint_command]"""
import argparse
import json
import math
import os
import socket
import time

import rclpy
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ap = argparse.ArgumentParser()
ap.add_argument("--in", dest="src", choices=["ros", "udp"], default="ros")
ap.add_argument("--out", dest="dst", choices=["ros", "udp"], default="ros")
ap.add_argument("--topic-in", default="/curobo/joint_command")
ap.add_argument("--msg", choices=["joint_state", "joint_trajectory"], default="joint_state", help="message type on the ROS input side")
ap.add_argument("--topic-out", default="/fr3_arm_controller/joint_trajectory")
ap.add_argument("--period", type=float, default=0.02)
ap.add_argument("--max-step", type=float, default=0.1)
ap.add_argument("--max-accel", type=float, default=0.0, help="rad/s^2 on the forwarded command (cuRobo/libfranka limit 10); 0 = off (default, like Spot: only the --max-step clip)")
ap.add_argument("--port", type=int, default=47802)
a, _ = ap.parse_known_args()
ARM = [f"fr3_joint{i}" for i in range(1, 8)]
rclpy.init()
n = rclpy.create_node("curobo_to_jtc")
pub = n.create_publisher(JointTrajectory, a.topic_out, 10) if a.dst == "ros" else None
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
st = {"last": None, "vlast": None, "fwd": 0, "rej": 0, "lim": 0, "tlast": 0.0, "t": time.monotonic()}
RESEED_GAP_S = 2.0   # no command for this long = a new activation: forget the old `last` pose (see emit())


def slew(q, last, max_step):
    """Move from `last` toward `q`, at most max_step rad per joint per command (no drop, no stop-and-go)."""
    out = [y + max(-max_step, min(max_step, x - y)) for x, y in zip(q, last)]
    return out, any(abs(x - o) > 1e-12 for x, o in zip(q, out))


def limit(q, v, last, vlast, dt, amax):
    """Acceleration-limited follower (per joint). Aims at the target q with the MPC's own velocity v as feed-forward; the closing
    speed relative to the target is bounded by what amax/2 can brake over the remaining error (no overshoot), and the commanded
    velocity may change by at most amax*dt per command. Returns (q, v, limited); identical to the input while nothing binds."""
    qo, vo, lim = [], [], False
    for x, vt, y, vy in zip(q, v, last, vlast):
        err = x - y
        rel = err / dt - vt
        b = math.sqrt(amax * abs(err))  # braking at amax/2: discrete steps overshoot at full amax (16-24 mm in a unit test)
        vd = vt + max(-b, min(b, rel))
        vn = max(vy - amax * dt, min(vy + amax * dt, vd))
        if vn != vd or abs(rel) > b:
            lim = True; qo.append(y + vn * dt); vo.append(vn)
        else:
            qo.append(x); vo.append(vt)
    return qo, vo, lim


def emit(q, v):
    if any(not math.isfinite(x) for x in q + v):
        st["rej"] += 1; return
    now = time.monotonic()
    if st["last"] is not None:
        if now - st["tlast"] > 3 * a.period:  # gap: the controller finished its last point and held still, so restart from rest
            st["vlast"] = [0.0] * len(q)
        if now - st["tlast"] > RESEED_GAP_S:  # NEW SESSION, not a stall: `last` is where the arm was at the PREVIOUS activation (minutes/hours ago), not where it is now.
            # The planner re-seeds from the measured joints, so its first command IS the arm pose: start from it. (2026-11-01 03:50 UTC: the limiter ramped the first
            # commands from the stale `last` of the earlier attempt -> joints 1/7 kicked 14/19 mrad with ~10 N, guard TRIP.) Short stalls keep their smooth catch-up.
            st["last"] = list(q); st["vlast"] = [0.0] * len(q)
        if a.max_accel > 0:
            q, v, lim = limit(q, v, st["last"], st["vlast"], a.period, a.max_accel)
            if lim:
                st["lim"] += 1
                n.get_logger().warn("acceleration limit active: command smoothed", throttle_duration_sec=1.0)
        q, clipped = slew(q, st["last"], a.max_step)
        if clipped:
            vmax = a.max_step / a.period  # keep the velocity consistent with the clipped step, never zero it
            st["rej"] += 1; v = [max(-vmax, min(vmax, x)) for x in v]
            n.get_logger().warn("step above max-step: clipped toward the target", throttle_duration_sec=1.0)
    st["last"] = q; st["vlast"] = v; st["tlast"] = now
    if a.dst == "udp":
        sock.sendto(json.dumps({"q": q, "v": v}).encode(), (os.environ.get("CMD_HOST", "127.0.0.1"), a.port))
    else:
        m = JointTrajectory()
        m.joint_names = ARM
        p = JointTrajectoryPoint()
        p.positions, p.velocities = q, v
        p.time_from_start = Duration(sec=0, nanosec=int(a.period * 1e9))
        m.points = [p]
        pub.publish(m)
    st["fwd"] += 1
    if time.monotonic() - st["t"] > 10:
        st["t"] = time.monotonic()
        n.get_logger().info(f"forwarded {st['fwd']}, refused {st['rej']}, accel-limited {st['lim']}")


def on_js(m):
    d = dict(zip(m.name, m.position)); dv = dict(zip(m.name, m.velocity)) if m.velocity else {}
    if not all(j in d for j in ARM):
        st["rej"] += 1; return
    emit([float(d[j]) for j in ARM], [float(dv.get(j, 0.0)) for j in ARM])


def on_traj(m):
    if not m.points or not all(j in m.joint_names for j in ARM):
        st["rej"] += 1; return
    p = m.points[0]; d = dict(zip(m.joint_names, p.positions)); dv = dict(zip(m.joint_names, p.velocities)) if p.velocities else {}
    emit([float(d[j]) for j in ARM], [float(dv.get(j, 0.0)) for j in ARM])


print(f"curobo_to_jtc: {a.src} -> {a.dst} ({a.topic_in if a.src == 'ros' else 'udp ' + str(a.port)} -> "
      f"{a.topic_out if a.dst == 'ros' else 'udp ' + str(a.port)}), period {a.period}", flush=True)
if a.src == "ros":
    if a.msg == "joint_trajectory": n.create_subscription(JointTrajectory, a.topic_in, on_traj, 10)
    else: n.create_subscription(JointState, a.topic_in, on_js, 10)
    rclpy.spin(n)
else:
    sock.bind(("127.0.0.1", a.port)); sock.settimeout(0.2)
    while rclpy.ok():
        try:
            d = json.loads(sock.recv(65536)); emit(d["q"], d["v"])
        except socket.timeout:
            pass
        rclpy.spin_once(n, timeout_sec=0)
