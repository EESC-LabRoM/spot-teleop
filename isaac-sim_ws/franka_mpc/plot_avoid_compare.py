"""Same grasp commands with and without the obstacle map: fingertip paths past the blue bottle and the arm's clearance to it over time.
   /usr/bin/python3 plot_avoid_compare.py runs/<with map> runs/<no map> [--out file.png]"""
import argparse, json, os
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
ap = argparse.ArgumentParser(); ap.add_argument("with_map"); ap.add_argument("no_map"); ap.add_argument("--out"); a = ap.parse_args()
INK, INK2, MUTED, GRID, BG = "#1d232b", "#4a525c", "#8a919a", "#e4e1da", "#fbfaf7"; BLUE, RED = "#2f5fb3", "#c0392b"
plt.rcParams.update({"figure.facecolor": BG, "axes.facecolor": BG, "axes.edgecolor": MUTED, "axes.labelcolor": INK2, "xtick.color": MUTED, "ytick.color": MUTED,
                     "font.size": 9.5, "axes.titlesize": 10.5, "axes.titleweight": "bold", "axes.titlecolor": INK, "axes.grid": True, "grid.color": GRID})
A, B = np.load(os.path.join(a.with_map, "avoid_check.npz")), np.load(os.path.join(a.no_map, "avoid_check.npz"))
MV = json.load(open(os.path.join(a.with_map, "objects.json")))["centre_moved_mm"]; MN = json.load(open(os.path.join(a.no_map, "objects.json")))["centre_moved_mm"]
M = dict(np.load(os.path.join(a.with_map, "objects_start.npz"))); root = np.array(json.load(open(os.path.join(a.with_map, "RESULT_isaac.json")))["root"])
V = M["veja"].reshape(-1, 3) - root
fig, ax = plt.subplots(1, 3, figsize=(14, 4.6), gridspec_kw={"width_ratios": [1, 1, 1.35]})
fig.suptitle("Same drill-grasp commands, with and without the nvblox obstacle map: the blue bottle is in the way", x=0.01, ha="left", fontsize=12.5, fontweight="bold", color=INK)
for k, (z, c, lab) in enumerate(((0, "tcp", "top view, x forward / y left (cm)"), (2, "tcp", "side view, x forward / z up (cm)"))):
    for D, col, nm in ((B, RED, "no map"), (A, INK, "with map")):
        ax[k].plot(D[c][:, 0] * 100, D[c][:, 1 if z == 0 else 2] * 100, color=col, lw=2.0, label=nm)
    ax[k].scatter(V[::60, 0] * 100, (V[::60, 1] if z == 0 else V[::60, 2]) * 100, s=0.6, color=BLUE, alpha=0.5)
    ax[k].set_aspect("equal"); ax[k].set_title(lab, loc="left"); ax[k].set_xlabel("x, cm"); ax[k].set_xlim(5, 70)
    ax[k].set_ylim((-25, 30) if z == 0 else (0, 60))
ax[0].legend(frameon=False, loc="lower left"); ax[0].text(56, 2, "blue\nbottle", color=BLUE, fontsize=8, ha="center", va="center")
ax[0].set_ylabel("fingertip path (y, cm)"); ax[1].set_ylabel("fingertip path (z, cm)")
ax[2].plot(B["t"], B["clear_veja"] * 100, color=RED, lw=2, label=f"no map: bottle knocked {MN['veja']:.0f} mm")
ax[2].plot(A["t"], A["clear_veja"] * 100, color=INK, lw=2, label=f"with map: bottle moved {MV['veja']:.0f} mm")
ax[2].axhline(0, color=RED, lw=0.8); ax[2].axhline(3, color=MUTED, lw=0.8, ls=":"); ax[2].text(0.3, 3.5, "3 cm margin the node keeps", fontsize=8, color=INK2)
ax[2].set_ylim(-4, 40); ax[2].set_xlim(0, 12); ax[2].set_xlabel("sim time since the first goal, s"); ax[2].set_ylabel("closest arm surface to the bottle, cm\n(below 0 = inside it)")
ax[2].set_title("clearance of the whole arm to the blue bottle", loc="left"); ax[2].legend(frameon=False, loc="upper right")
fig.tight_layout(rect=(0, 0, 1, 0.93)); out = a.out or os.path.join(a.with_map, "avoid_compare.png"); fig.savefig(out, dpi=110); print(out)
