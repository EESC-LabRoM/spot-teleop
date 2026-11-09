"""Metrics of the lab trial. python3 trial_analysis.py DIR   (DIR has rec.csv goal.csv preview.csv fwd_*.log). Pure python."""
import csv, math, sys, statistics as st, bisect, re
D = sys.argv[1]
rd = lambda f: list(csv.DictReader(open(f"{D}/{f}")))
rec, goal, prev = rd("rec.csv"), rd("goal.csv"), rd("preview.csv")
G = [(float(r["t"]), float(r["x"]), float(r["y"]), float(r["z"])) for r in goal]; gt = [g[0] for g in G]
g0 = G[0][0]; HOLD, RAMP, DWELL = 8.0, 3.0, 6.0          # goal_run.py defaults used in the trial
t_out, t_back = g0 + HOLD, g0 + HOLD + RAMP + DWELL
def goal_at(t):
    i = min(max(bisect.bisect_left(gt, t), 0), len(G) - 1); return G[i][1:]
R = [(float(r["t"]), [float(r[f"q{i}"]) for i in range(7)], [float(r[f"qd{i}"]) for i in range(7)], [float(r[f"c{i}"]) for i in range(7)],
      (float(r["hx"]), float(r["hy"]), float(r["hz"])), float(r["trk_err"]), r["mode"], r["err"], r["tripped"]) for r in rec if r["hx"] != "nan" and float(r["t"]) >= g0 - 30 and float(r["t"]) <= g0 + 40]
def pct(v, p): v = sorted(v); return v[min(len(v) - 1, int(p / 100 * len(v)))]
def seg(a, b): return [r for r in R if a <= r[0] < b]
def herr(rows):  # hand position error to the goal, mm (3D), and x-only signed
    e3 = [math.dist(r[4], goal_at(r[0])) * 1e3 for r in rows]; ex = [(r[4][0] - goal_at(r[0])[0]) * 1e3 for r in rows]; return e3, ex
print(f"run: {len(R)} rec rows over {R[-1][0]-R[0][0]:.1f} s; modes {sorted(set(r[6] for r in R))}; error flags {sorted(set(r[7] for r in R))}; watchdog trips {sorted(set(r[8] for r in R))}")
# --- hold before the move
h = seg(g0 - 20, t_out - 0.5); e3, ex = herr(h)
print(f"\nHOLD (20 s before the move): hand error to goal mean {st.mean(e3):.2f} mm max {max(e3):.2f} mm; hand wander {max(math.dist(r[4], h[0][4]) for r in h)*1e3:.3f} mm; joint speed max {max(abs(x) for r in h for x in r[2])*1e3:.1f} mrad/s")
# --- moves
for name, a, b in (("OUT  (0 -> +5 cm, 3 s)", t_out, t_out + RAMP + 1.0), ("BACK (+5 cm -> 0, 3 s)", t_back, t_back + RAMP + 1.0)):
    s = seg(a, b); e3, ex = herr(s); sp = []
    for r0, r1 in zip(s, s[1:]):
        dt = r1[0] - r0[0]
        if dt > 0: sp.append(math.dist(r1[4], r0[4]) / dt)
    print(f"{name}: hand error to goal mean {st.mean(e3):.1f} mm max {max(e3):.1f} mm (x lag max {max(abs(x) for x in ex):.1f} mm); peak hand speed (0.1 s mean) "
          f"{max(st.mean(sp[i:i+10]) for i in range(0, len(sp)-10))*100:.1f} cm/s (goal peak {1.875*0.05/RAMP*100:.1f} cm/s); joint tracking err max {max(r[5] for r in s)*1e3:.1f} mrad; "
          f"peak joint speed {max(abs(x) for r in s for x in r[2]):.3f} rad/s")
for name, a, b in (("settle after OUT  (dwell, last 2 s)", t_out + RAMP + DWELL - 2, t_out + RAMP + DWELL), ("settle after BACK (tail, last 5 s)", g0 + 25, g0 + 30)):
    s = seg(a, b); e3, ex = herr(s); print(f"{name}: hand error mean {st.mean(e3):.2f} mm max {max(e3):.2f} mm; wander {max(math.dist(r[4], s[0][4]) for r in s)*1e3:.3f} mm")
# lag: time at which the hand reaches 50% of dx versus the goal
def cross(rows, level, up=True):
    for r in rows:
        if (r[4][0] - 0.309533 >= level) if up else (r[4][0] - 0.309533 <= level): return r[0]
tg = t_out + RAMP / 2; th = cross(seg(t_out, t_out + 8), 0.025)
print(f"lag: goal passes +2.5 cm at {tg-g0:.2f} s, hand at {th-g0:.2f} s -> {1e3*(th-tg):.0f} ms behind" if th else "lag: hand never reached +2.5 cm")
# measured joint accel (qd at 100 Hz, 50 ms window)
mv = seg(t_out, t_out + RAMP + 1); mv += seg(t_back, t_back + RAMP + 1); acc = []
for rows in (seg(t_out, t_out + RAMP + 1), seg(t_back, t_back + RAMP + 1)):
    for i in range(5, len(rows)):
        dt = rows[i][0] - rows[i-5][0]
        if dt > 0: acc.append(max(abs(rows[i][2][j] - rows[i-5][2][j]) for j in range(7)) / dt)
print(f"measured joint acceleration during moves (50 ms): p99 {pct(acc,99):.2f} max {max(acc):.2f} rad/s2 (limit 10)")
# --- command stream (preview = what cuRobo emitted, 1:1 what the forwarders carried)
P = [(float(r["t"]), [float(r[f"q{i}"]) for i in range(7)], [float(r[f"v{i}"]) for i in range(7)]) for r in prev]
ia = [b[0] - a[0] for a, b in zip(P, P[1:])]; dq = [max(abs(x - y) for x, y in zip(b[1], a[1])) for a, b in zip(P, P[1:])]
dv = [max(abs(x - y) for x, y in zip(b[2], a[2])) for a, b in zip(P, P[1:])]
print(f"\nCOMMANDS (cuRobo output, {len(P)} msgs in {P[-1][0]-P[0][0]:.1f} s): rate {(len(P)-1)/(P[-1][0]-P[0][0]):.1f} Hz; inter-arrival p50 {pct(ia,50)*1e3:.1f} p99 {pct(ia,99)*1e3:.1f} max {max(ia)*1e3:.1f} ms; gaps>100 ms: {sum(1 for x in ia if x>0.1)}")
print(f"  step between commands |dq| p99 {pct(dq,99)*1e3:.2f} max {max(dq)*1e3:.2f} mrad (clip backstop 30 mrad); commanded |v| max {max(abs(x) for p in P for x in p[2]):.3f} rad/s (cap 1.2); max change of commanded v per command {max(dv):.3f} rad/s")
print(f"  plan time vs wall time: MPC advances 20 ms per step -> at {(len(P)-1)/(P[-1][0]-P[0][0]):.1f} Hz the plan runs at {(len(P)-1)/(P[-1][0]-P[0][0])*0.02*100:.0f}% of real time")
for f in ("fwd_a.log", "fwd_lab.log"):
    txt = open(f"{D}/{f}").read(); last = re.findall(r"forwarded (\d+), refused (\d+), accel-limited (\d+)", txt)[-1]
    print(f"  {f}: forwarded {last[0]} refused {last[1]} accel-limited {last[2]}; 'step above max-step' warnings: {txt.count('step above max-step')}")
