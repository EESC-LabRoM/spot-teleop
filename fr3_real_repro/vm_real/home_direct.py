#!/usr/bin/env python3
"""Straight joint-space move from the live joints to the start pose: minimum-jerk, peak speed --vmax. Same UDP {"q","v"} output and gates as
home_replay.py (lab curobo_to_jtc limits, JTC, contact guard). NO collision knowledge of its own: home_run.sh MODE=direct checks the dumped path
with clearance_q.py against the real object clouds before anything is sent and refuses below --min-gap cm.
  home_direct.py --q-now=q0,..,q6 --q-target=q0,..,q6 [--vmax 0.3] [--dump path.csv] [--dry]"""
import argparse, json, socket, sys, time
import numpy as np
ap = argparse.ArgumentParser()
ap.add_argument("--q-now", required=True); ap.add_argument("--q-target", required=True)
ap.add_argument("--vmax", type=float, default=0.3); ap.add_argument("--hold", type=float, default=2.0); ap.add_argument("--hz", type=float, default=50.0)
ap.add_argument("--host", default="10.0.0.2"); ap.add_argument("--port", type=int, default=47802)
ap.add_argument("--dump", default=""); ap.add_argument("--dry", action="store_true")
a = ap.parse_args()
q0 = np.array([float(v) for v in a.q_now.split(",")]); q1 = np.array([float(v) for v in a.q_target.split(",")]); assert len(q0) == len(q1) == 7
dq = q1 - q0; D = max(1.0, np.abs(dq).max() * 1.875 / a.vmax)          # min-jerk peak speed = 1.875 * max|dq| / D
n = int(D * a.hz) + 1; tau = np.arange(n) / a.hz; x = np.clip(tau / D, 0, 1)
s = 10 * x**3 - 15 * x**4 + 6 * x**5; ds = (30 * x**2 - 60 * x**3 + 30 * x**4) / D
Qc = q0[None, :] + s[:, None] * dq[None, :]; V = ds[:, None] * dq[None, :]
print(f"direct move: max joint change {np.abs(dq).max():.3f} rad -> {D:.0f} s (peak {np.abs(V).max():.2f} rad/s, peak accel {5.77 * np.abs(dq).max() / D**2:.2f} rad/s2, max step {np.abs(np.diff(Qc, axis=0)).max():.4f} rad)")
print("target:", np.round(q1, 4).tolist())
if a.dump:
    with open(a.dump, "w") as fh:
        fh.write("t," + ",".join(f"q{i}" for i in range(7)) + "\n")
        for i in range(n): fh.write(f"{tau[i]:.3f}," + ",".join(f"{v:.5f}" for v in Qc[i]) + "\n")
if not (np.isfinite(Qc).all() and np.isfinite(V).all()): print("REFUSED: non-finite"); sys.exit(2)
if a.dry: print("DRY: nothing sent"); sys.exit(0)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); t0 = time.perf_counter()
for i in range(n + int(a.hold * a.hz)):
    j = min(i, n - 1); due = t0 + i / a.hz
    while time.perf_counter() < due: time.sleep(0.0005)
    sock.sendto(json.dumps({"q": Qc[j].tolist(), "v": (V[j] if i < n else np.zeros(7)).tolist()}).encode(), (a.host, a.port))
    if i % int(5 * a.hz) == 0: print(f"  move {i / a.hz:5.0f}/{D:.0f} s  q0 {Qc[j][0]:+.3f}", flush=True)
print("move done")
