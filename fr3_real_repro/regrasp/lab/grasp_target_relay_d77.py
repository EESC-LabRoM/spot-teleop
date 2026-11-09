"""Relay the regrasp loop's /grasp_target from the live domain 8 (VM loop) to the MPC's direct domain 77
(lab, LOCALHOST discovery), for curobo_mpc_node pf_mode:=grasp (6D attraction). Phase B of
fr3_real_repro/regrasp/DESIGN.md. NOT deployed yet (needs Murilo's go together with the MPC relaunch).

Same two-context pattern as /tmp/esdf_proxy_d77.py (run it inside codex-fr3-live-mpc with python.sh), separate process.
Forwards each valid message immediately (the MPC's own pf_grasp_timeout_s 0.5 s does the release when the loop stops
publishing: LOCKED/OCCLUDED/untrusted). Drops anything that is not a sane fr3_hand pose in fr3_link0:
wrong frame, non-finite, |q| far from 1, or outside the reachable box. Never invents or holds a target.

  /isaac-sim/python.sh grasp_target_relay_d77.py [--topic /grasp_target]
"""
import argparse
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

ap = argparse.ArgumentParser()
ap.add_argument('--topic', default='/grasp_target')
ap.add_argument('--frame', default='fr3_link0')
ap.add_argument('--in-domain', type=int, default=8)
ap.add_argument('--out-domain', type=int, default=77)
a, _ = ap.parse_known_args()

BOX_LO, BOX_HI = (0.10, -0.70, 0.00), (0.90, 0.70, 0.80)   # fr3_hand position sanity box (m, fr3_link0)


def valid(m):
    if m.header.frame_id != a.frame:
        return 'frame %r' % m.header.frame_id
    p, q = m.pose.position, m.pose.orientation
    v = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
    if not all(math.isfinite(x) for x in v):
        return 'non-finite'
    if abs(math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2) - 1.0) > 0.01:
        return 'quaternion norm'
    if not all(lo <= x <= hi for x, lo, hi in zip((p.x, p.y, p.z), BOX_LO, BOX_HI)):
        return 'outside box (%.3f, %.3f, %.3f)' % (p.x, p.y, p.z)
    return None


qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
c77, c8 = Context(), Context()
rclpy.init(context=c77, domain_id=a.out_domain)
rclpy.init(context=c8, domain_id=a.in_domain)
n77 = rclpy.create_node('grasp_target_relay_out', context=c77)
n8 = rclpy.create_node('grasp_target_relay_in', context=c8)
pub = n77.create_publisher(PoseStamped, a.topic, qos)
st = {'in': 0, 'out': 0, 'drop': 0}


def on_msg(m):
    st['in'] += 1
    why = valid(m)
    if why:
        st['drop'] += 1
        if st['drop'] <= 10 or st['drop'] % 100 == 0:
            print(f'[relay] drop #{st["drop"]}: {why}', flush=True)
        return
    pub.publish(m)
    st['out'] += 1
    if st['out'] <= 3 or st['out'] % 600 == 0:
        p = m.pose.position
        print(f'[relay] in={st["in"]} out={st["out"]} drop={st["drop"]} last=({p.x:.3f},{p.y:.3f},{p.z:.3f})', flush=True)


n8.create_subscription(PoseStamped, a.topic, on_msg, qos)
e8 = SingleThreadedExecutor(context=c8)
e8.add_node(n8)
print(f'[relay] {a.topic}: domain {a.in_domain} -> domain {a.out_domain} ({a.frame}, sanity box {BOX_LO}..{BOX_HI})', flush=True)
e8.spin()
