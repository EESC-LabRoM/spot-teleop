"""Short summary of one run dir (rec.csv goal.csv preview.csv). python3 trial_summary.py RUNDIR"""
import csv, sys, math
D = sys.argv[1]
G = list(csv.DictReader(open(f"{D}/goal.csv"))); P = list(csv.DictReader(open(f"{D}/preview.csv")))
gx = [float(r["x"]) for r in G]; gy = [float(r["y"]) for r in G]; gz = [float(r["z"]) for r in G]
print(f"goal: {len(G)} msgs over {float(G[-1]['t'])-float(G[0]['t']):.1f} s; x [{min(gx):.3f},{max(gx):.3f}] y [{min(gy):.3f},{max(gy):.3f}] z [{min(gz):.3f},{max(gz):.3f}]; frozen flag set: {any(r['frozen']=='1' for r in G)}")
t = [float(r["t"]) for r in P]; v = max(abs(float(r[f"v{i}"])) for r in P for i in range(7))
print(f"preview (cuRobo output): {len(P)} msgs, {(len(P)-1)/(t[-1]-t[0]):.1f} Hz, max |v| {v:.3f} rad/s")
