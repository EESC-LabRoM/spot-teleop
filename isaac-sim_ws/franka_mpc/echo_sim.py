# Kinematic stand-in for franka_ros_arm.py, to time curobo_mpc_node.py without Isaac's renderer on the GPU: no Kit, no
# physics. Publishes /clock (default 200 Hz) paced to wall time (sim time = wall time) and /joint_states_isaac (100 Hz) as
# the latest /joint_command_curobo (perfect joint tracking), starting at retract. Logs every command's arrival.
#   python echo_sim.py --out <dir> --seconds 60       (Isaac image Python + bundled Jazzy rclpy: run_echo_timing.sh)
import argparse, json, os, threading, time
import numpy as np
import rclpy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState

ap = argparse.ArgumentParser()
ap.add_argument("--out", required=True)
ap.add_argument("--seconds", type=float, default=60.0)
ap.add_argument("--clock-hz", type=float, default=200.0, help="/clock rate (sim time advances in 1/clock-hz steps)")
ap.add_argument("--js-every", type=int, default=2, help="/joint_states_isaac every N clock ticks")
a = ap.parse_args(); os.makedirs(a.out, exist_ok=True)
NAMES = [f"fr3_joint{i + 1}" for i in range(7)] + ["fr3_finger_joint1", "fr3_finger_joint2"]
q = np.array([0.0, -1.3, 0.0, -2.5, 0.0, 1.0, 0.0, 0.04, 0.04]); qd = np.zeros(9)
lock = threading.Lock(); log = {"wall": [], "sim": [], "stamp": []}
rclpy.init(); node = rclpy.create_node("echo_franka")
clock_pub = node.create_publisher(Clock, "/clock", 10)
js_pub = node.create_publisher(JointState, "/joint_states_isaac", 10)
t0 = time.time()
def on_cmd(m):
    now = time.time()
    with lock:
        for nm, p_, v_ in zip(m.name, m.position, m.velocity if len(m.velocity) else [0.0] * len(m.position)):
            if nm in NAMES: i = NAMES.index(nm); q[i], qd[i] = p_, v_
        log["wall"].append(now - t0); log["sim"].append(now - t0); log["stamp"].append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
node.create_subscription(JointState, "/joint_command_curobo", on_cmd, 10)
threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
def stamp(msg_stamp, t):
    msg_stamp.sec = int(t); msg_stamp.nanosec = int(round((t - int(t)) * 1e9)) % 1000000000
n = 0
while time.time() - t0 < a.seconds:
    n += 1; t = n / a.clock_hz
    c = Clock(); stamp(c.clock, t); clock_pub.publish(c)
    if n % a.js_every == 0:
        js = JointState(); stamp(js.header.stamp, t); js.name = NAMES
        with lock: js.position = [float(v) for v in q]; js.velocity = [float(v) for v in qd]
        js_pub.publish(js)
    lead = t - (time.time() - t0)
    if lead > 0: time.sleep(lead)
with lock: L = {k: np.array(v) for k, v in log.items()}
np.savez(os.path.join(a.out, "echo_trace.npz"), **L)
w = L["wall"]; first = w[0] if len(w) else float("nan")
dt = np.diff(w) * 1000 if len(w) > 1 else np.array([float("nan")])
res = {"seconds": a.seconds, "clock_hz": a.clock_hz, "joint_states_hz": a.clock_hz / a.js_every, "commands": int(len(w)), "first_command_s": round(float(first), 2),
       "commands_per_s_after_first": round(float(len(w) / (w[-1] - first)), 1) if len(w) > 1 else None,
       "command_interval_ms": {"median": round(float(np.median(dt)), 2), "p95": round(float(np.percentile(dt, 95)), 2),
                               "max": round(float(dt.max()), 1), "over_30ms_pct": round(float((dt > 30).mean()) * 100, 2)},
       "arrival_minus_stamp_ms": {"median": round(float(np.median((w - L["stamp"]) * 1000)), 2),
                                  "p95": round(float(np.percentile((w - L["stamp"]) * 1000, 95)), 2)}}
json.dump(res, open(os.path.join(a.out, "RESULT_echo.json"), "w"), indent=1)
print("[echo] RESULT " + json.dumps(res), flush=True)
node.destroy_node(); rclpy.shutdown()
