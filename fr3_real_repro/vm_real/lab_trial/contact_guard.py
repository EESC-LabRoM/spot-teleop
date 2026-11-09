"""Contact logger + secondary guard for the real avoidance run. Runs INSIDE fr3-ros2 (jazzy rclpy). Never commands the arm.
Logs /franka_robot_state_broadcaster/robot_state at 100 Hz (q, external force/torque estimate, robot_mode) to CSV. Baseline = mean over the first BASE seconds (arm still).
TRIP (unless --dry): |F - F0| > FTRIP N or any |tau_ext - tau0| > TTRIP Nm for N consecutive 1 kHz messages, or robot_mode 4 (reflex): stops the command forwarder
(curobo_to_jtc.py --out ros: the JTC then holds its last point) and writes the trip to the log. NOTE events (log only) at |dF| > FNOTE N.
  python3 contact_guard.py --out /tmp/x.csv --secs 120 [--dry] [--ftrip 8 --ttrip 5 --fnote 3 --base 4]"""
import argparse, csv, os, signal, time
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from franka_msgs.msg import FrankaRobotState
ap = argparse.ArgumentParser(); ap.add_argument("--out", required=True); ap.add_argument("--secs", type=float, default=120); ap.add_argument("--dry", action="store_true")
ap.add_argument("--ftrip", type=float, default=8.0); ap.add_argument("--ttrip", type=float, default=5.0); ap.add_argument("--fnote", type=float, default=3.0); ap.add_argument("--base", type=float, default=4.0); ap.add_argument("--n", type=int, default=5)
ap.add_argument("--qtrip", type=float, default=0.0)   # rad: also TRIP if any joint is further than this from its posture at the end of the baseline (0 = off)
a = ap.parse_args()
def stop_forwarder():
    k = []
    for p in os.listdir("/proc"):
        if p.isdigit():
            try: c = open(f"/proc/{p}/cmdline", "rb").read().replace(b"\0", b" ").decode()
            except Exception: continue
            if "curobo_to_jtc.py" in c and "--out ros" in c and "guard" not in c: os.kill(int(p), signal.SIGTERM); k.append(int(p))
    return k
class G(Node):
    def __init__(self):
        super().__init__("claude_contact_guard"); self.t0 = time.time(); self.n = 0; self.f0 = None; self.t0v = None; self.bf, self.bt, self.bq = [], [], []; self.q0 = None; self.cnt = 0; self.tripped = False
        self.maxdF = 0.0; self.maxdT = 0.0; self.notes = 0; self.f = open(a.out, "w", newline=""); self.w = csv.writer(self.f)
        self.w.writerow(["t"] + [f"q{i}" for i in range(7)] + ["Fx", "Fy", "Fz", "dF"] + [f"tau{i}" for i in range(7)] + ["dTmax", "mode", "event"])
        self.create_subscription(FrankaRobotState, "/franka_robot_state_broadcaster/robot_state", self.cb, QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT))
    def cb(self, m):
        t = time.time() - self.t0; F = np.array([m.o_f_ext_hat_k.wrench.force.x, m.o_f_ext_hat_k.wrench.force.y, m.o_f_ext_hat_k.wrench.force.z]); tau = np.array(m.tau_ext_hat_filtered.effort[:7], float)
        if t < a.base: self.bf.append(F); self.bt.append(tau); self.bq.append(np.array(m.measured_joint_state.position[:7], float)); return
        if self.f0 is None: self.q0 = np.mean(self.bq, 0); self.f0 = np.mean(self.bf, 0); self.t0v = np.mean(self.bt, 0); self.get_logger().info(f"baseline F0 {self.f0.round(2).tolist()} N, tau0 {self.t0v.round(2).tolist()}")
        dF = float(np.linalg.norm(F - self.f0)); dT = float(np.max(np.abs(tau - self.t0v))); self.maxdF = max(self.maxdF, dF); self.maxdT = max(self.maxdT, dT)
        ev = ""; exc = float(np.max(np.abs(np.array(m.measured_joint_state.position[:7]) - self.q0)))
        bad = dF > a.ftrip or dT > a.ttrip or m.robot_mode == 4 or (a.qtrip > 0 and exc > a.qtrip)
        self.cnt = self.cnt + 1 if bad else 0
        if dF > a.fnote and not ev: ev = "NOTE"; self.notes += 1
        if self.cnt >= a.n and not self.tripped:
            self.tripped = True; ev = "TRIP"; killed = [] if a.dry else stop_forwarder()
            self.get_logger().error(f"TRIP at t={t:.2f}: dF {dF:.1f} N dTau {dT:.1f} Nm mode {m.robot_mode} posture-excursion {exc:.2f} rad (limit {a.qtrip}); forwarder stopped: {killed} (dry={a.dry})")
        self.n += 1
        if self.n % 10 == 0 or ev: self.w.writerow([f"{t:.3f}"] + [f"{x:.5f}" for x in m.measured_joint_state.position[:7]] + [f"{v:.2f}" for v in F] + [f"{dF:.2f}"] + [f"{v:.2f}" for v in tau] + [f"{dT:.2f}", m.robot_mode, ev])
n = rclpy.init() or G(); stop = {"v": False}; signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__("v", True)); signal.signal(signal.SIGINT, lambda *_: stop.__setitem__("v", True))
while not stop["v"] and time.time() - n.t0 < a.secs: rclpy.spin_once(n, timeout_sec=0.05)
n.f.flush(); n.get_logger().info(f"done: {n.n} msgs, max dF {n.maxdF:.2f} N, max dTau {n.maxdT:.2f} Nm, NOTE rows {n.notes}, tripped {n.tripped}")
print(f"GUARD SUMMARY max dF {n.maxdF:.2f} N, max dTau {n.maxdT:.2f} Nm, notes {n.notes}, tripped {n.tripped}", flush=True)
