"""Plots for one franka_mpc_isaac.py run: convergence, hand trajectory, joint tracking, MPC solve time.
   /usr/bin/python3 plot_run.py runs/simple_mpc      -> runs/simple_mpc/plots.png"""
import json, os, sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

run = os.path.abspath(sys.argv[1] if len(sys.argv) > 1 else "runs/simple_mpc")
T = np.load(os.path.join(run, "trace.npz")); R = json.load(open(os.path.join(run, "RESULT.json")))
SURF, INK, INK2, MUTED, GRID, AXIS = "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7"
S1, S2, S3 = "#2a78d6", "#eb6834", "#1baf7a"            # reference palette slots 1-3 (validated, light mode)
plt.rcParams.update({"figure.facecolor": SURF, "axes.facecolor": SURF, "axes.edgecolor": AXIS, "axes.labelcolor": INK2,
                     "xtick.color": MUTED, "ytick.color": MUTED, "grid.color": GRID, "grid.linewidth": 0.6,
                     "font.size": 10, "axes.titlesize": 11, "axes.titleweight": "bold", "axes.titlecolor": INK,
                     "axes.spines.top": False, "axes.spines.right": False, "lines.linewidth": 1.6})


def plot_path():
    t, ph, ref, ee = T["t"], T["ti"].astype(int), T["target"], T["ee_fk"]
    P = R["path"]; d_cm = np.linalg.norm(ee - ref, axis=1) * 100
    v_hand = np.linalg.norm(np.gradient(ee, t, axis=0), axis=1); v_ref = np.linalg.norm(np.gradient(ref, t, axis=0), axis=1)
    edges = [t[np.argmax(ph >= k)] for k in (1, 2, 3)]
    fig = plt.figure(figsize=(15, 9.5))
    gs = fig.add_gridspec(4, 2, width_ratios=[2.1, 1], hspace=0.5, wspace=0.18, left=0.06, right=0.96, top=0.88, bottom=0.07)
    fig.suptitle("Franka + cuRobo MPC in Isaac Sim 6.0: continuous path at constant speed", x=0.06, ha="left",
                 fontsize=14, fontweight="bold", color=INK)
    e = P["tracking_err_cm_at_full_speed"]
    fig.text(0.06, 0.915, f"circle r = {P['radius_m']} m with a {P['wave_m']} m vertical wave, {P['laps']:g} laps of {P['period_s']:g} s "
             f"· reference {P['reference_speed_mps']['min']}-{P['reference_speed_mps']['max']} m/s · hand-to-reference at full speed: "
             f"mean {e['mean']} cm, p95 {e['p95']} cm, max {e['max']} cm · all steps feasible: {R['all_steps_feasible']} · "
             f"MPC settings: {R['override'] or 'cuRobo defaults'}", ha="left", color=INK2, fontsize=10)
    def deco(ax, ylabel):
        ax.grid(True, axis="y"); ax.set_ylabel(ylabel); ax.set_xlim(0, t[-1])
        for x in edges: ax.axvline(x, color=AXIS, lw=0.8, zorder=0)
    ax = fig.add_subplot(gs[0, 0]); deco(ax, "speed (m/s)")
    ax.plot(t, v_ref, color=S2, lw=1.2, ls="--"); ax.plot(t, v_hand, color=S1)
    ax.text(t[-1], v_hand[-1], " hand", color=INK, va="bottom", fontsize=9, fontweight="bold", clip_on=False)
    ax.set_title("Speed: hand (solid) vs moving reference (dashed)", loc="left")
    for x0, x1, lab in ((0, edges[0], "ease onto path"), (edges[1], edges[2], "path, 2 laps"), (edges[2], t[-1], "stop")):
        ax.text((x0 + x1) / 2, ax.get_ylim()[1] * 0.97, lab, color=INK2, fontsize=9, ha="center", va="top")
    ax = fig.add_subplot(gs[1, 0], sharex=ax); deco(ax, "distance (cm)")
    ax.plot(t, d_cm, color=S1); ax.set_title("Tracking: distance from the hand to the moving reference", loc="left")
    ax = fig.add_subplot(gs[2, 0], sharex=ax); deco(ax, "joint speed (deg/s)")
    for j in range(T["qd"].shape[1]): ax.plot(t, np.degrees(T["qd"][:, j]), color=S1, lw=1.0, alpha=0.75)
    ax.set_title("All 7 measured joint speeds (PhysX): smooth, no steps", loc="left")
    ax = fig.add_subplot(gs[3, 0], sharex=ax); deco(ax, "solve time (ms)")
    ax.plot(t, T["mpc_ms"], color=S1, lw=1.0); ax.axhline(20, color=MUTED, lw=1, ls="--")
    ax.text(t[-1], 20, " 20 ms", color=MUTED, va="center", ha="left", fontsize=9, clip_on=False)
    ax.set_ylim(0, max(22, float(np.percentile(T["mpc_ms"], 99.5)) * 1.2)); ax.set_xlabel("simulated time (s)")
    ax.set_title("MPC solve time per 50 Hz step, inside Isaac (physics waits for each solve)", loc="left")
    on = ph >= 1
    for row, (k, name, title) in zip((slice(0, 2), slice(2, 4)), ((1, "y (m)", "Path, top view"), (2, "z (m)", "Path, side view"))):
        ax = fig.add_subplot(gs[row, 1]); ax.grid(True)
        ax.plot(ref[on, 0], ref[on, k], color=S2, lw=1.2, ls="--"); ax.plot(ee[:, 0], ee[:, k], color=S1)
        ax.plot(ee[0, 0], ee[0, k], "o", ms=8, mfc=SURF, mec=INK2, mew=1.5); ax.text(ee[0, 0], ee[0, k], "  start", color=INK2, va="center", fontsize=9)
        ax.set_xlabel("x (m)"); ax.set_ylabel(name); ax.set_aspect("equal", adjustable="datalim"); ax.set_title(title, loc="left")
        if k == 1:
            ax.text(0.02, 0.02, "solid: hand · dashed: reference", transform=ax.transAxes, color=INK2, fontsize=9)
    out = os.path.join(run, "plots.png"); fig.savefig(out, dpi=110); print("wrote", out); sys.exit(0)

if R.get("mode") == "path":
    plot_path()

t, ti = T["t"], T["ti"].astype(int); tg, ee = T["target"], T["ee_fk"]
d_cm = np.linalg.norm(ee - tg, axis=1) * 100
switch = t[np.flatnonzero(np.diff(ti)) + 1]
err_deg = np.degrees(np.abs(T["q_cmd"] - T["q"])).max(axis=1)
fk_mm = float(np.linalg.norm(T["ee_fk"] - T["ee_sim"], axis=1).max() * 1000)

fig = plt.figure(figsize=(15, 9.5))
gs = fig.add_gridspec(4, 2, width_ratios=[2.1, 1], hspace=0.45, wspace=0.18, left=0.06, right=0.97, top=0.88, bottom=0.07)
fig.suptitle("Franka + cuRobo MPC in Isaac Sim 6.0 (PhysX 200 Hz, MPC 50 Hz)", x=0.06, ha="left", fontsize=14,
             fontweight="bold", color=INK)
fig.text(0.06, 0.915, f"{len(R['targets'])} targets held {t[-1] / len(R['targets']) + 0.02:.0f} s each · MPC solve median "
         f"{R['mpc_solve_ms']['median']} ms (p95 {R['mpc_solve_ms']['p95']} ms) · all steps feasible: {R['all_steps_feasible']} · "
         f"cuRobo FK vs PhysX hand: max {fk_mm:.4f} mm · base moved {R['base_moved_mm']} mm",
         ha="left", color=INK2, fontsize=10)

def deco(ax, ylabel):
    ax.grid(True, axis="y"); ax.set_ylabel(ylabel); ax.set_xlim(0, t[-1])
    for s in switch: ax.axvline(s, color=AXIS, lw=0.8, zorder=0)

ax = fig.add_subplot(gs[0, 0]); deco(ax, "hand to target (cm)")
ax.semilogy(t, d_cm, color=S1); ax.axhline(1.0, color=MUTED, lw=1, ls="--")
ax.yaxis.set_major_formatter(matplotlib.ticker.FuncFormatter(lambda v, _: f"{v:g}"))
ax.text(t[-1], 1.0, " 1 cm", color=MUTED, va="center", ha="left", fontsize=9, clip_on=False)
ax.set_title("Convergence: distance from the hand to the current target", loc="left")
starts = np.concatenate([[0.0], switch])
for i, (s, p) in enumerate(zip(starts, R["targets"])):
    lab = f"T{i + 1}: 1 cm in {p['time_to_1cm_s']} s" if p["time_to_1cm_s"] is not None else f"T{i + 1}: not within 1 cm"
    ax.text(s + 0.35, d_cm.max() * 1.45, lab, color=INK2, fontsize=9, va="top")
ax.set_ylim(max(d_cm.min() * 0.5, 0.01), d_cm.max() * 1.6)

ax = fig.add_subplot(gs[1, 0], sharex=ax); deco(ax, "position (m)")
for k, c, n in ((0, S1, "x"), (1, S2, "y"), (2, S3, "z")):
    ax.plot(t, ee[:, k], color=c); ax.plot(t, tg[:, k], color=c, lw=1, ls="--")
    ax.text(t[-1], ee[-1, k], f" {n}", color=INK, va="center", ha="left", fontsize=10, fontweight="bold", clip_on=False)
ax.set_title("Hand position from the measured joints (solid) vs target (dashed)", loc="left")

ax = fig.add_subplot(gs[2, 0], sharex=ax); deco(ax, "max |cmd - measured| (deg)")
ax.plot(t, err_deg, color=S1)
ax.set_title("Joint tracking under physics: largest gap between MPC command and PhysX joint angle", loc="left")

ax = fig.add_subplot(gs[3, 0], sharex=ax); deco(ax, "solve time (ms)")
ax.plot(t, T["mpc_ms"], color=S1, lw=1.0); ax.axhline(20, color=MUTED, lw=1, ls="--")
ax.text(t[-1], 20, " 20 ms budget", color=MUTED, va="center", ha="left", fontsize=9, clip_on=False)
ax.set_ylim(0, max(22, float(np.percentile(T["mpc_ms"], 99.5)) * 1.2)); ax.set_xlabel("simulated time (s)")
ax.set_title("MPC solve time per 50 Hz step, inside Isaac (physics waits for each solve, so motion is unaffected)", loc="left")

for row, (k, name) in zip((slice(0, 2), slice(2, 4)), ((1, "y (m)"), (2, "z (m)"))):
    ax = fig.add_subplot(gs[row, 1]); ax.grid(True)
    ax.plot(ee[:, 0], ee[:, k], color=S1)
    ax.plot(ee[0, 0], ee[0, k], "o", ms=8, mfc=SURF, mec=INK2, mew=1.5)
    ax.text(ee[0, 0], ee[0, k], "  start", color=INK2, va="center", fontsize=9)
    spots = {}
    for i, p in enumerate(R["targets"]):                       # targets that coincide in this view share one label
        spots.setdefault((round(p["target_m"][0], 3), round(p["target_m"][k], 3)), []).append(f"T{i + 1}")
    for (x, v), labs in spots.items():
        ax.plot(x, v, "s", ms=8, mfc=INK, mec=SURF, mew=1.5)
        ax.text(x, v, "  " + "/".join(labs), color=INK, va="bottom", fontsize=10, fontweight="bold")
    ax.set_xlabel("x (m)"); ax.set_ylabel(name); ax.set_aspect("equal", adjustable="datalim")
    ax.set_title("Hand path, top view" if k == 1 else "Hand path, side view", loc="left")

out = os.path.join(run, "plots.png"); fig.savefig(out, dpi=110); print("wrote", out)
