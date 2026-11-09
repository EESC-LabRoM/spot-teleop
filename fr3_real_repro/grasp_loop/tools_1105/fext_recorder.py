#!/usr/bin/env python3
"""Passive F_ext recorder for the FR3 cartesian_reflex lock-ups (2026-11-05). NEVER commands the arm.
Runs INSIDE fr3-ros2 (jazzy, domain 77).  python3 fext_recorder.py OUTDIR [--hours 6]
- Subscribes /franka_robot_state_broadcaster/robot_state (1 kHz, best effort).
- OUTDIR/rec100.csv: 100 Hz decimated stream (t, mode, |F|, F xyz, M xyz, tau_ext j1-7, dq j1-7, tau_J_d j1-7).
- Keeps the last 4 s at full rate; when robot_mode enters 4 (reflex) or 5, or a last_motion_error appears,
  it records 0.5 s more and dumps OUTDIR/event_<n>_<mode>.npz with a summary line in OUTDIR/events.log:
  peak |F| and per-joint |tau_ext| in the 1 s before the event, as a fraction of the thresholds
  (force 50/50/50 N, 30 Nm; torque 40,40,36,36,16,14,12 Nm), the collision flags, the error names, peak |dq|,
  peak tau_J_d step per 1 ms tick.
- 1 Hz heartbeat in OUTDIR/heartbeat.log.
"""
import argparse, collections, os, signal, sys, time
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from franka_msgs.msg import FrankaRobotState

ap = argparse.ArgumentParser(); ap.add_argument("out"); ap.add_argument("--hours", type=float, default=6.0)
a = ap.parse_args(); os.makedirs(a.out, exist_ok=True)
TQ = np.array([40, 40, 36, 36, 16, 14, 12.0]); FF = np.array([50, 50, 50, 30, 30, 30.0])
ERRS = None
COLS = ["t", "mode"] + ["F", "Fx", "Fy", "Fz", "Mx", "My", "Mz"] + [f"te{i}" for i in range(1, 8)] + [f"dq{i}" for i in range(1, 8)] + [f"tjd{i}" for i in range(1, 8)]


def errnames(e):
    global ERRS
    if ERRS is None:
        ERRS = [f for f in e.get_fields_and_field_types()]
    return [f for f in ERRS if getattr(e, f)]


class R(Node):
    def __init__(self):
        super().__init__("claude_fext_recorder")
        self.t0 = time.time(); self.n = 0; self.ring = collections.deque(maxlen=4000); self.mode_prev = None; self.err_prev = []
        self.pending = None; self.nev = 0; self.last_hb = 0.0; self.hb_max = np.zeros(2)
        self.csv = open(os.path.join(a.out, "rec100.csv"), "w"); self.csv.write(",".join(COLS) + "\n")
        self.ev = open(os.path.join(a.out, "events.log"), "a"); self.hb = open(os.path.join(a.out, "heartbeat.log"), "a")
        self.create_subscription(FrankaRobotState, "/franka_robot_state_broadcaster/robot_state", self.cb,
                                 QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT))

    def cb(self, m):
        w = m.o_f_ext_hat_k.wrench
        W = np.array([w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z])
        te = np.array(m.tau_ext_hat_filtered.effort[:7], float)
        dq = np.array(m.measured_joint_state.velocity[:7], float)
        tjd = np.array(m.desired_joint_state.effort[:7], float)
        ci = m.collision_indicators
        coll = np.r_[[ci.is_cartesian_linear_collision.x, ci.is_cartesian_linear_collision.y, ci.is_cartesian_linear_collision.z,
                      ci.is_cartesian_angular_collision.x, ci.is_cartesian_angular_collision.y, ci.is_cartesian_angular_collision.z],
                     list(ci.is_joint_collision)[:7]]
        row = np.r_[m.time, m.robot_mode, np.linalg.norm(W[:3]), W, te, dq, tjd, coll]
        self.ring.append(row); self.n += 1
        if self.n % 10 == 0:
            self.csv.write(",".join(f"{v:.4f}" for v in row[:len(COLS)]) + "\n")
        self.hb_max = np.maximum(self.hb_max, [row[2], np.max(np.abs(te) / TQ)])
        now = time.time()
        if now - self.last_hb > 1.0:
            self.last_hb = now; self.csv.flush()
            self.hb.write(f"{time.strftime('%H:%M:%S', time.gmtime())}Z mode {m.robot_mode} |F| max {self.hb_max[0]:.1f} N  tau_ext max {100*self.hb_max[1]:.0f}% of limit  msgs {self.n}\n"); self.hb.flush()
            self.hb_max[:] = 0
        errs = errnames(m.last_motion_errors) + errnames(m.current_errors)
        trig = (m.robot_mode in (4, 5) and self.mode_prev not in (4, 5)) or (errs and not self.err_prev)
        self.mode_prev = m.robot_mode; self.err_prev = errs
        if trig and self.pending is None:
            self.pending = {"tick": self.n, "mode": int(m.robot_mode), "errs": sorted(set(errs)), "wall": time.strftime('%H:%M:%S', time.gmtime())}
        if self.pending is not None and self.n - self.pending["tick"] >= 500:
            self.dump(); self.pending = None

    def dump(self):
        p = self.pending; A = np.array(self.ring); self.nev += 1
        k = len(A) - 1 - (self.n - p["tick"]); pre = A[max(0, k - 1000):k + 1]
        F = pre[:, 2]; Wabs = np.abs(pre[:, 3:9]); te = np.abs(pre[:, 9:16]); dq = np.abs(pre[:, 16:23]); tjd = pre[:, 23:30]
        fr_w = (Wabs / FF).max(0); fr_t = (te / TQ).max(0)
        step = np.abs(np.diff(tjd, axis=0)).max(0) if len(tjd) > 1 else np.zeros(7)
        flags = (pre[:, 30:] > 0).any(0)
        names = [f"cart_lin_{c}" for c in "xyz"] + [f"cart_ang_{c}" for c in "xyz"] + [f"joint{i}" for i in range(1, 8)]
        fn = os.path.join(a.out, f"event_{self.nev}_mode{p['mode']}.npz"); np.savez_compressed(fn, data=A, trigger_index=k, cols=np.array(COLS + names))
        line = (f"{p['wall']}Z event {self.nev}: mode {p['mode']} errors {p['errs']}\n"
                f"  1 s before: peak |F| {F.max():.1f} N; wrench peak/limit {np.round(fr_w, 2).tolist()} (Fx Fy Fz Mx My Mz)\n"
                f"  tau_ext peak/limit j1-7 {np.round(fr_t, 2).tolist()}\n"
                f"  collision flags set: {[nm for nm, f in zip(names, flags) if f]}\n"
                f"  peak |dq| {np.round(dq.max(0), 2).tolist()} rad/s; peak tau_J_d step per tick {np.round(step, 2).tolist()} Nm\n"
                f"  file {fn}\n")
        self.ev.write(line); self.ev.flush(); print(line, flush=True)


rclpy.init(); n = R(); stop = {"v": False}
signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__("v", True)); signal.signal(signal.SIGINT, lambda *_: stop.__setitem__("v", True))
print(f"recording to {a.out} for {a.hours} h (passive, never commands)", flush=True)
while not stop["v"] and time.time() - n.t0 < a.hours * 3600:
    rclpy.spin_once(n, timeout_sec=0.05)
n.csv.flush(); print(f"done: {n.n} msgs, {n.nev} events", flush=True)
