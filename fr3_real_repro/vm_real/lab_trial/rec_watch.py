"""Recorder + watchdog for the lab trial. Runs in fr3-ros2 (driver domain 77), read-only on ROS.
Records 100 Hz: measured joints, the command the controller was last given, robot mode / error flags, hand pose (tf). Trips (SIGKILL of the
lab-half forwarder, exact pid) on: tracking error, joint speed, robot mode outside {1,2}, any current_errors flag, hand farther than --max-drift
from where it started, stale robot state. The JTC then keeps its last target; the e-stop stays the real stop.
python3 rec_watch.py --fwd-pid N --out /tmp/rec.csv [--seconds 300]"""
import argparse, csv, math, os, signal, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from franka_msgs.msg import FrankaRobotState
import tf2_ros

ap = argparse.ArgumentParser()
ap.add_argument("--fwd-pid", type=int, required=True); ap.add_argument("--out", required=True); ap.add_argument("--seconds", type=float, default=300)
ap.add_argument("--max-err", type=float, default=0.05); ap.add_argument("--max-qd", type=float, default=1.5); ap.add_argument("--max-drift", type=float, default=0.07)
a, _ = ap.parse_known_args()
ARM = [f"fr3_joint{i}" for i in range(1, 8)]
BE = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)

class W(Node):
    def __init__(self):
        super().__init__("claude_rec_watch")
        self.cmd = None; self.cmd_t = 0.0; self.mode = -1; self.err = False; self.rs_t = 0.0; self.p0 = None; self.hand = None; self.bad = 0; self.tripped = None; self.t_start = time.time(); self.n = 0
        self.f = open(a.out, "w"); self.w = csv.writer(self.f)
        self.w.writerow(["t"] + [f"q{i}" for i in range(7)] + [f"qd{i}" for i in range(7)] + [f"c{i}" for i in range(7)] + ["cmd_age", "mode", "err", "hx", "hy", "hz", "hqx", "hqy", "hqz", "hqw", "trk_err", "tripped"])
        self.tfb = tf2_ros.Buffer(); self.tfl = tf2_ros.TransformListener(self.tfb, self)
        self.create_subscription(JointState, "/joint_states", self.on_js, BE)
        self.create_subscription(JointTrajectory, "/fr3_arm_controller/joint_trajectory", self.on_cmd, 10)
        self.create_subscription(FrankaRobotState, "/franka_robot_state_broadcaster/robot_state", self.on_rs, BE)
        self.create_timer(0.02, self.on_tf)
    def on_cmd(self, m):
        if m.points and all(j in m.joint_names for j in ARM):
            d = dict(zip(m.joint_names, m.points[0].positions)); self.cmd = [d[j] for j in ARM]; self.cmd_t = time.time()
    def on_rs(self, m):
        self.mode = int(m.robot_mode); self.rs_t = time.time()
        e = m.current_errors; self.err = any(bool(getattr(e, k)) for k in e.get_fields_and_field_types())
    def on_tf(self):
        try:
            t = self.tfb.lookup_transform("fr3_link0", "fr3_hand", rclpy.time.Time()).transform
            self.hand = (t.translation.x, t.translation.y, t.translation.z, t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)
            if self.p0 is None: self.p0 = self.hand[:3]
        except Exception: pass
    def trip(self, why):
        if self.tripped: return
        self.tripped = why; self.get_logger().error(f"TRIP: {why}: killing forwarder pid {a.fwd_pid}")
        try: os.kill(a.fwd_pid, signal.SIGKILL)
        except ProcessLookupError: pass
        open(a.out + ".TRIP", "w").write(why + "\n")
    def on_js(self, m):
        d = dict(zip(m.name, m.position)); dv = dict(zip(m.name, m.velocity)) if m.velocity else {}
        if not all(j in d for j in ARM): return
        q = [d[j] for j in ARM]; qd = [dv.get(j, 0.0) for j in ARM]; now = time.time()
        age = now - self.cmd_t if self.cmd else 99.0; c = self.cmd or [float("nan")] * 7
        trk = max(abs(x - y) for x, y in zip(q, c)) if self.cmd and age < 0.5 else 0.0
        why = None
        if trk > a.max_err: why = f"tracking error {trk:.3f} rad"
        elif max(abs(x) for x in qd) > a.max_qd: why = f"joint speed {max(abs(x) for x in qd):.2f} rad/s"
        elif self.rs_t and self.mode not in (1, 2): why = f"robot mode {self.mode}"
        elif self.err: why = "robot error flag"
        elif self.rs_t and now - self.rs_t > 1.0: why = "robot state stale"
        elif self.hand and self.p0 and math.dist(self.hand[:3], self.p0) > a.max_drift: why = f"hand {math.dist(self.hand[:3], self.p0)*100:.1f} cm from start"
        self.bad = self.bad + 1 if why else 0
        if self.bad >= 3: self.trip(why)
        h = self.hand or (float("nan"),) * 7
        self.w.writerow([f"{now:.4f}"] + [f"{x:.6f}" for x in q + qd + c] + [f"{age:.3f}", self.mode, int(self.err)] + [f"{x:.6f}" for x in h] + [f"{trk:.5f}", int(bool(self.tripped))])
        self.n += 1
        if self.n % 200 == 0: self.f.flush()
        if now - self.t_start > a.seconds: raise SystemExit
rclpy.init(); n = W()
try: rclpy.spin(n)
except (SystemExit, KeyboardInterrupt): pass
n.f.flush(); print(f"rec_watch done: {n.n} rows, tripped={n.tripped}", flush=True)
