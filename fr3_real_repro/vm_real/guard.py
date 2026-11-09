"""Runs in vm-relay: watches /joint_states (domain 143, from the real driver via the cable). Reference = pose when started.
Writes /out/guard.csv; touches /out/TRIP when any arm joint is more than LIM rad from the reference (LIM env, default 0.015 ~ 8-10 mm at the hand)."""
import os, time, rclpy
from sensor_msgs.msg import JointState
LIM = float(os.environ.get("LIM", "0.015")); ARM = [f"fr3_joint{i}" for i in range(1, 8)]
rclpy.init(); n = rclpy.create_node("vm_guard"); ref = {}; st = {"max": 0.0, "n": 0, "t": time.time()}
f = open("/out/guard.csv", "w"); f.write("t,max_drift_rad\n")
def cb(m):
    d = dict(zip(m.name, m.position))
    if not all(j in d for j in ARM): return
    q = [d[j] for j in ARM]
    if "q" not in ref: ref["q"] = q; return
    dr = max(abs(a - b) for a, b in zip(q, ref["q"])); st["max"] = max(st["max"], dr); st["n"] += 1
    if dr > LIM: open("/out/TRIP", "w").write(f"{dr}\n")
    if st["n"] % 25 == 0: f.write(f"{time.time()-st['t']:.2f},{dr:.5f}\n"); f.flush()
n.create_subscription(JointState, "/joint_states", cb, 50); rclpy.spin(n)
