#!/usr/bin/env python3
"""Bring the parked FR3 back to its start pose by replaying, in REVERSE and slowed down, the joint path the arm itself drove
(measured joints in a contact-guard CSV: start -> parked). That path was already driven once with >= 7.9 cm sphere clearance to
the bottle / drill / box (clearance_q.py), so its reverse is the one route home known to be clear. No planner, no map.

Output = the same UDP JSON {"q","v"} the planner's forwarder sends, to the lab half of curobo_to_jtc.py (127.0.0.1:47802 on the
lab host; from the VM that is 10.0.0.2:47802 through udp_fwd), so max-step / max-accel limits, the JTC and the contact guard
apply exactly as in avoid_run.sh. The first command must match the live joints (--q-now) within --tol, else it refuses.

  home_replay.py --csv guard_X.csv --t-from 41.0 --t-to 92.6 --q-now "q0,...,q6" [--dry] [--vmax 0.12]
Runs in vm-relay (/isaac-sim/python.sh, numpy)."""
import argparse, csv, json, socket, sys, time
import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("--csv", required=True)
ap.add_argument("--t-from", type=float, required=True, help="guard-CSV time at which the arm was still at the start pose")
ap.add_argument("--t-to", type=float, required=True, help="guard-CSV time at which the arm was parked (end pose)")
ap.add_argument("--q-now", required=True, help="live measured joints, 7 comma-separated radians")
ap.add_argument("--tol", type=float, default=0.02, help="max |live - first command| per joint (rad)")
ap.add_argument("--vmax", type=float, default=0.12, help="peak joint speed of the replay (rad/s); the path is slowed to stay under it")
ap.add_argument("--hold", type=float, default=2.0, help="seconds to keep sending the last point")
ap.add_argument("--hz", type=float, default=50.0)
ap.add_argument("--host", default="10.0.0.2"); ap.add_argument("--port", type=int, default=47802)
ap.add_argument("--dry", action="store_true", help="compute and check only, send nothing")
ap.add_argument("--dump", default="", help="write the commanded path as t,q0..q6 CSV (for clearance_q.py)")
a = ap.parse_args()

rows = list(csv.DictReader(open(a.csv)))
t = np.array([float(r["t"]) for r in rows]); Q = np.array([[float(r[f"q{i}"]) for i in range(7)] for r in rows])
m = (t >= a.t_from) & (t <= a.t_to); t, Q = t[m], Q[m]
assert len(t) > 1000, f"too few samples in [{a.t_from}, {a.t_to}]"
# uniform 100 Hz, light smoothing of the measurement (0.4 s box), ends padded with their own value
tu = np.arange(t[0], t[-1], 0.01); Qu = np.stack([np.interp(tu, t, Q[:, j]) for j in range(7)], 1)
w = 40; pad = np.pad(Qu, ((w // 2, w // 2), (0, 0)), mode="edge"); k = np.ones(w) / w
Qs = np.stack([np.convolve(pad[:, j], k, mode="valid")[: len(tu)] for j in range(7)], 1)
Qs[0], Qs[-1] = Qu[0], Qu[-1]
P = tu[-1] - tu[0]                                    # original duration of the path (s)
smooth = lambda x: x * x * (3 - 2 * x)                # zero velocity at both ends
def q_at(p):                                          # reverse path parameter p in [0, P] -> joints
    s = tu[-1] - p; return np.array([np.interp(s, tu, Qs[:, j]) for j in range(7)]).T
# speed of the reversed path with the smoothstep warp at 1x, then slow it down so the peak stays under --vmax
x = np.linspace(0, 1, 4001); pp = P * smooth(x); qq = q_at(pp); dq = np.gradient(qq, x, axis=0)  # dq/dx (rad per unit of normalised time)
peak1 = np.abs(dq).max()                              # rad per unit x; speed = dq/dx / D
D = max(P, peak1 / a.vmax)                            # replay duration (s)
n = int(D * a.hz) + 1; tau = np.arange(n) / a.hz; xs = np.clip(tau / D, 0, 1)
Qc = q_at(P * smooth(xs))
qn = np.array([float(v) for v in a.q_now.split(",")]); assert len(qn) == 7
off = qn - Qc[0]; Qc = Qc + off[None, :] * (1 - smooth(np.clip(tau / (0.15 * D), 0, 1)))[:, None]   # blend the tiny start offset away
V = np.gradient(Qc, tau, axis=0)
dev0 = np.abs(qn - q_at(0.0)).max()
print(f"path {a.t_from:.1f}..{a.t_to:.1f} s ({P:.1f} s) -> replay {D:.0f} s ({D / P:.2f}x slower); live vs first point: max {dev0:.4f} rad (tol {a.tol})")
print(f"peak speed {np.abs(V).max():.3f} rad/s (limit {a.vmax}), peak accel {np.abs(np.gradient(V, tau, axis=0)).max():.3f} rad/s2, max step per command {np.abs(np.diff(Qc, axis=0)).max():.4f} rad")
print("start pose (end of replay):", np.round(Qc[-1], 4).tolist()); print("parked pose (start of replay):", np.round(Qc[0], 4).tolist())
if dev0 > a.tol: print(f"REFUSED: the arm is {dev0:.3f} rad from where this path starts (> {a.tol}); it is not parked where the path ends"); sys.exit(2)
if not np.isfinite(Qc).all() or not np.isfinite(V).all(): print("REFUSED: non-finite command"); sys.exit(2)
if a.dump:
    with open(a.dump, "w") as fh:
        fh.write("t," + ",".join(f"q{i}" for i in range(7)) + "\n")
        for i in range(n): fh.write(f"{tau[i]:.3f}," + ",".join(f"{v:.5f}" for v in Qc[i]) + "\n")
if a.dry: print("DRY: nothing sent"); sys.exit(0)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); t0 = time.perf_counter(); sent = 0
for i in range(n + int(a.hold * a.hz)):
    j = min(i, n - 1); due = t0 + i / a.hz
    while time.perf_counter() < due: time.sleep(0.0005)
    sock.sendto(json.dumps({"q": Qc[j].tolist(), "v": (V[j] if i < n else np.zeros(7)).tolist()}).encode(), (a.host, a.port)); sent += 1
    if i % int(5 * a.hz) == 0: print(f"  replay {i / a.hz:5.0f}/{D:.0f} s  q0 {Qc[j][0]:+.3f}", flush=True)
print(f"replay done, {sent} commands")
