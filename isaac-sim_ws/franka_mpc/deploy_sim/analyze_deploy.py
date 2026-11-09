"""Deployment-path sim metrics from <run>/trace.npz (+ bridge.csv). usage: analyze_deploy.py <run dir> [label]"""
import csv, sys, numpy as np
run = sys.argv[1]; label = sys.argv[2] if len(sys.argv) > 2 else run
Z = np.load(f"{run}/trace.npz", allow_pickle=True); t = np.asarray(Z["t"], float); t = t - t[0]
q, qd, qc, hand, tgt = (np.asarray(Z[k], float) for k in ("q", "qd", "q_cmd", "hand", "target"))
dt = float(np.median(np.diff(t))); err = np.linalg.norm(hand - tgt, axis=1) * 1000
speed = np.linalg.norm(np.diff(hand, axis=0), axis=1) / dt * 1000; acc = np.diff(qd, axis=0) / dt; jerk = np.diff(acc, axis=0) / dt
dqc = np.abs(np.diff(qc, axis=0)).max(axis=1)
HOLDS = [(0, 6), (9, 15), (18, 24), (25, 31), (33, 40)]
print(f"== {label}: {len(t)} steps, dt {dt*1000:.1f} ms, {t[-1]:.1f} sim s")
for a, b in HOLDS:
    m = (t >= a) & (t < b)
    if m.sum() < 10: continue
    ss = (t >= a + 2.5) & (t < b)                      # after settling
    drift = np.linalg.norm(hand[ss] - hand[ss][0], axis=1).max() * 1000 if ss.sum() else float("nan")
    print(f"  hold {a:2d}-{b:2d}s: error mean {err[ss].mean():5.1f} mm  max {err[ss].max():5.1f} mm | hand drift after settle {drift:4.1f} mm")
mv = ~np.any([(t >= a) & (t < b) for a, b in HOLDS], axis=0)
print(f"  moves: tracking error mean {err[mv].mean():5.1f} mm  max {err[mv].max():5.1f} mm")
stall = ((speed < 2.0) & (np.linalg.norm(hand - tgt, axis=1)[1:] > 0.010) & mv[1:]).sum()
print(f"  stop-and-go: {stall} steps with hand speed < 2 mm/s while >10 mm from the goal during moves")
print(f"  joint accel |max| {np.abs(acc).max():6.1f} rad/s2  p99 {np.percentile(np.abs(acc),99):5.1f} | jerk p99 {np.percentile(np.abs(jerk),99):7.0f} rad/s3")
print(f"  commanded step |dq| max {dqc.max()*1000:5.1f} mrad  p99 {np.percentile(dqc,99)*1000:4.1f} mrad | repeated commands {(dqc < 1e-9).sum()}")
try:
    B = list(csv.DictReader(open(f"{run}/bridge.csv"))); print(f"  bridge: {len(B)} commands, clipped {sum(int(r['clipped']) for r in B)}, stalled {sum(int(r['stalled']) for r in B)}, accel-limited {sum(int(r.get('limited', 0)) for r in B)}")
except Exception as e: print("  bridge: n/a", e)
