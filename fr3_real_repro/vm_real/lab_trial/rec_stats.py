"""Quick stats of a rec_watch CSV: python3 rec_stats.py file.csv [t_from t_to]  (seconds since the first row)"""
import csv, math, sys
R = list(csv.DictReader(open(sys.argv[1])))
t0 = float(R[0]["t"]); lo = float(sys.argv[2]) if len(sys.argv) > 2 else 0; hi = float(sys.argv[3]) if len(sys.argv) > 3 else 1e9
R = [r for r in R if lo <= float(r["t"]) - t0 <= hi]
t = [float(r["t"]) for r in R]; f = lambda k: [float(r[k]) for r in R]
print(f"rows {len(R)}  span {t[-1]-t[0]:.1f}s  rate {(len(R)-1)/(t[-1]-t[0]):.1f} Hz  gaps>30ms {sum(1 for a,b in zip(t,t[1:]) if b-a>0.03)}")
print(f"mode {sorted(set(r['mode'] for r in R))}  err flags {sorted(set(r['err'] for r in R))}  tripped {sorted(set(r['tripped'] for r in R))}")
trk = f("trk_err"); print(f"cmd-vs-measured max {max(trk)*1e3:.2f} mrad  mean {sum(trk)/len(trk)*1e3:.2f} mrad  cmd_age max {max(f('cmd_age')):.3f}s")
print(f"max |qd| {max(abs(x) for i in range(7) for x in f(f'qd{i}')):.4f} rad/s")
h = [(float(r['hx']), float(r['hy']), float(r['hz'])) for r in R if r['hx'] != 'nan']
if h: print(f"hand start ({h[0][0]:.4f},{h[0][1]:.4f},{h[0][2]:.4f})  max excursion from start {max(math.dist(p,h[0]) for p in h)*1e3:.2f} mm  end ({h[-1][0]:.4f},{h[-1][1]:.4f},{h[-1][2]:.4f})")
q0 = [float(R[0][f"q{i}"]) for i in range(7)]; print("max joint drift from first row %.3f mrad" % (max(abs(float(r[f'q{i}']) - q0[i]) for r in R for i in range(7)) * 1e3))
