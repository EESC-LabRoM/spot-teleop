"""Plot a run_grasp.sh run: the arm's tracking of the grasp sequence, the fingers, the drill's height, the node's collision cost.
   /usr/bin/python3 plot_grasp.py runs/<run> [--out file.png]"""
import argparse, os
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
ap = argparse.ArgumentParser(); ap.add_argument("run"); ap.add_argument("--out"); a = ap.parse_args()
INK, INK2, MUTED, GRID, BG = "#1d232b", "#4a525c", "#8a919a", "#e4e1da", "#fbfaf7"
plt.rcParams.update({"figure.facecolor": BG, "axes.facecolor": BG, "axes.edgecolor": MUTED, "axes.labelcolor": INK2, "xtick.color": MUTED,
                     "ytick.color": MUTED, "font.size": 9, "axes.titlesize": 10, "axes.titleweight": "bold", "axes.titlecolor": INK, "axes.grid": True, "grid.color": GRID})
T = np.load(os.path.join(a.run, "trace.npz")); t = T["t"] - T["t"][0]
hand, tgt, qf, ob = T["hand"], T["target"], T["q_fingers"], T["obj"]; err = np.linalg.norm(hand - tgt, axis=1) * 1e3
D = np.genfromtxt(os.path.join(a.run, "mpc_node.csv"), delimiter=",", names=True, dtype=None, encoding=None, invalid_raise=False)
cc = np.array([float(x) if str(x).strip() not in ("", "nan") else np.nan for x in D["coll_cost"]]); cc = cc[-len(t):]; tc = np.linspace(0, t[-1], len(cc))
marks = [(0, "to pregrasp"), (4.0, "settle"), (4.5, "approach"), (7.0, "close"), (8.0, "grip"), (8.8, "lift"), (11.8, "hold")]
fig, ax = plt.subplots(4, 1, figsize=(9.5, 9), sharex=True)
fig.suptitle("Drill grasp in Isaac Sim: segmentation cloud, GraspGen-X, MPC on the nvblox map", x=0.01, ha="left", fontsize=12, fontweight="bold", color=INK)
ax[0].plot(t, hand[:, 2] * 100, color=INK, label="hand height z"); ax[0].plot(t, tgt[:, 2] * 100, color="#c0392b", lw=1, ls="--", label="goal")
ax[0].plot(t, ob[:, 2] * 100 + 0, color="#2a9d5c", label="drill (rise from its start)"); ax[0].set_ylabel("cm (base frame)"); ax[0].legend(frameon=False, ncol=3)
ax[1].plot(t, err, color=INK); ax[1].set_ylabel("hand vs goal, mm")
ax[2].plot(t, qf[:, 0] * 2e3, color="#3b6fb6"); ax[2].set_ylabel("finger gap, mm")
ax[3].plot(tc, cc, color="#8e5ea2"); ax[3].set_ylabel("node collision cost"); ax[3].set_xlabel("sim time since the first goal, s")
for x, lab in marks:
    for k, ax_ in enumerate(ax):
        ax_.axvline(x, color=MUTED, lw=0.6)
        if k == 0: ax_.text(x + 0.05, ax_.get_ylim()[1], lab, rotation=90, va="top", ha="left", fontsize=7.5, color=INK2)
fig.tight_layout(rect=(0, 0, 1, 0.96)); out = a.out or os.path.join(a.run, "grasp_timeline.png"); fig.savefig(out, dpi=110); print(out)
