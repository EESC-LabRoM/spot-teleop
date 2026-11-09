#!/usr/bin/env python3
"""Stream a planned calibration sweep (CSV t,q0..q6,hold at 50 Hz from calib_legs.py) to the lab forwarder: UDP {"q","v"} -> curobo_to_jtc
(max-step, max-accel limits) -> fr3_arm_controller. Same transport as home_direct.py. Needs the arm at the first sample (--q-now, tol).
  calib_sweep.py --csv path.csv --q-now=q0,..,q6 [--dry] [--stop-file /tmp/sweep_stop] [--hz 50]
A file at --stop-file stops the stream at once: the last sample is held (v=0). The e-stop stays the real safety."""
import argparse, json, os, socket, sys, time
import numpy as np
ap = argparse.ArgumentParser(); ap.add_argument("--csv", required=True); ap.add_argument("--q-now", required=True); ap.add_argument("--tol", type=float, default=0.03)
ap.add_argument("--hz", type=float, default=50.0); ap.add_argument("--host", default="10.0.0.2"); ap.add_argument("--port", type=int, default=47802)
ap.add_argument("--stop-file", default="/tmp/sweep_stop"); ap.add_argument("--dry", action="store_true"); ap.add_argument("--max-step", type=float, default=0.02)
a = ap.parse_args()
D = np.loadtxt(a.csv, delimiter=",", skiprows=1); T, Q, Hd = D[:, 0], D[:, 1:8], D[:, 8]; qn = np.array([float(v) for v in a.q_now.split(",")]); assert len(qn) == 7
V = np.gradient(Q, 1 / a.hz, axis=0); step = np.abs(np.diff(Q, axis=0)).max()
dev = np.abs(Q[0] - qn).max(); print(f"sweep: {len(T)} samples, {T[-1]:.0f} s, {int(((np.diff(Hd, prepend=0)) > 0).sum())} holds, max step {step:.4f} rad, peak speed {np.abs(V).max():.3f} rad/s, start deviation {dev:.4f} rad")
if not (np.isfinite(Q).all() and np.isfinite(V).all()): print("REFUSED: non-finite"); sys.exit(2)
if step > a.max_step: print(f"REFUSED: a step of {step:.4f} rad is above {a.max_step}"); sys.exit(2)
if dev > a.tol: print(f"REFUSED: the arm is {dev:.3f} rad from the sweep start (tol {a.tol})"); sys.exit(2)
if a.dry: print("DRY: nothing sent"); sys.exit(0)
if os.path.exists(a.stop_file): os.remove(a.stop_file)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); t0 = time.perf_counter(); last = 0; hold_i = -1
for i in range(len(T)):
    due = t0 + i / a.hz
    while time.perf_counter() < due: time.sleep(0.0005)
    if os.path.exists(a.stop_file):
        print(f"STOP FILE at sample {i}/{len(T)} (t={T[i]:.0f} s): holding the last sample"); 
        for _ in range(int(2 * a.hz)): sock.sendto(json.dumps({"q": Q[max(i - 1, 0)].tolist(), "v": [0.0] * 7}).encode(), (a.host, a.port)); time.sleep(1 / a.hz)
        sys.exit(4)
    sock.sendto(json.dumps({"q": Q[i].tolist(), "v": (V[i] * (1 - Hd[i])).tolist()}).encode(), (a.host, a.port))
    if Hd[i] and hold_i != i and (i == 0 or not Hd[i - 1]): hold_i = i; print(f"  hold at t={T[i]:5.0f} s  q={np.round(Q[i], 2).tolist()}", flush=True)
for _ in range(int(2 * a.hz)): sock.sendto(json.dumps({"q": Q[-1].tolist(), "v": [0.0] * 7}).encode(), (a.host, a.port)); time.sleep(1 / a.hz)
print("sweep done")
