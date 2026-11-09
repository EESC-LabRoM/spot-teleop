"""Plot grasp_filter.py's result for one or more objects: the target's segmentation cloud, the neighbours' clouds, the top candidates as
Franka Hand wireframes coloured by the filter that rejected them, and the selected grasp (bold) with its pregrasp (dashed).
   /usr/bin/python3 grasp_plot.py runs/<run> obj1 [obj2 ...] [--top 25] [--out file.png]
"""
import argparse, json, os
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

ap = argparse.ArgumentParser(); ap.add_argument("run"); ap.add_argument("objects", nargs="+"); ap.add_argument("--top", type=int, default=25)
ap.add_argument("--out"); a = ap.parse_args()
INK, INK2, MUTED, GRID, BG = "#1d232b", "#4a525c", "#8a919a", "#e4e1da", "#fbfaf7"
COL = {0: "#2a9d5c", 1: "#c0392b", 2: "#c0392b", 3: "#d9a21b", 4: "#8e5ea2", 5: "#3b6fb6", 6: "#3b6fb6"}
LAB = {0: "passes all filters", 1: "hand/fingers hit the target", 3: "nothing between the fingers", 4: "hits table or neighbour",
       5: "no reach (IK)", 6: "no reach (IK)"}
CP = np.array([[0, 0, 0], [0, 0, 0.0584], [0, 0.04, 0.0584], [0, 0.04, 0.1034], [0, -0.04, 0.0584], [0, -0.04, 0.1034]])
SEG = [(0, 1), (2, 4), (2, 3), (4, 5)]
nice = {"veja": "blue bottle", "drill": "drill", "SM_Mug_B1": "mug", "SM_BottlePlasticC_01": "small bottle"}
plt.rcParams.update({"figure.facecolor": BG, "axes.facecolor": BG, "font.size": 9, "axes.titlesize": 10, "axes.titleweight": "bold", "axes.titlecolor": INK})
n = len(a.objects); fig = plt.figure(figsize=(5.2 * n, 9.6))
fig.suptitle("Grasp selection from the segmentation cloud: GraspGen-X candidates, collision and reach filters", x=0.01, ha="left", fontsize=13, fontweight="bold", color=INK)
def hand(ax, T, col, lw, ls="-", back=0.0):
    W = (T[:3, :3] @ CP.T).T + T[:3, 3] - back * T[:3, 2]
    for i, j in SEG: ax.plot(*W[[i, j]].T, color=col, lw=lw, ls=ls)
for c, o in enumerate(a.objects):
    G = np.load(os.path.join(a.run, "grasps", f"{o}.npz")); S = np.load(os.path.join(a.run, "grasps", f"{o}_status.npz"))
    sel = json.load(open(os.path.join(a.run, "grasps", f"{o}_selected.json"))); pc = G["pc"]
    nb = [np.load(os.path.join(a.run, "clouds", f)) ["pc"][::12] for f in os.listdir(os.path.join(a.run, "clouds")) if f.endswith(".npz") and f[:-4] != o]
    nb = np.concatenate(nb) if nb else np.zeros((0, 3))
    for row, (el, az) in enumerate(((22, -60), (88, -90))):
        ax = fig.add_subplot(2, n, row * n + c + 1, projection="3d"); ax.set_facecolor(BG)
        ctr = pc.mean(0); half = max(0.16, 0.8 * np.ptp(pc, 0).max())
        near = nb[np.all(np.abs(nb - ctr) < half * 1.3, axis=1)]
        ax.scatter(*near.T, s=0.4, color=GRID, alpha=0.5, depthshade=False)
        ax.scatter(*pc.T, s=0.8, color=INK2, alpha=0.8, depthshade=False)
        for k in range(min(a.top, len(S["grasps"])))[::-1]:
            hand(ax, S["grasps"][k], COL[int(S["status"][k])], 1.0)
        if "grasp" in sel:
            T = np.eye(4); T[:3, 3] = sel["grasp"][:3]; T[:3, :3] = Rotation.from_quat(sel["grasp"][3:]).as_matrix()
            hand(ax, T, "#1b6e3e", 3.4); hand(ax, T, "#1b6e3e", 1.6, "--", back=sel["pregrasp_m"])
            P0, P1 = T[:3, 3] - sel["pregrasp_m"] * T[:3, 2], T[:3, 3]; ax.plot(*np.stack([P0, P1]).T, color="#1b6e3e", lw=1.2, ls=":")
        ax.set_xlim(ctr[0] - half, ctr[0] + half); ax.set_ylim(ctr[1] - half, ctr[1] + half); ax.set_zlim(max(-0.01, ctr[2] - half), ctr[2] + half)
        ax.set_box_aspect((1, 1, 1)); ax.view_init(el, az); ax.set_xticklabels([]); ax.set_yticklabels([]); ax.set_zticklabels([])
        if row == 0:
            f = sel["funnel"]; v = f["valid"]
            ax.set_title(f"{nice.get(o, o)}: {f['candidates']} candidates, {v} valid\n"
                         + (f"picked #{sel['index']}: conf {sel['confidence']:.2f}, approach down {sel['approach_down']:.2f}" if v else "no valid grasp"), loc="left")
from matplotlib.lines import Line2D
fig.legend(handles=[Line2D([0], [0], color=COL[k], lw=2, label=LAB[k]) for k in (0, 1, 3, 4, 5)] + [Line2D([0], [0], color="#1b6e3e", lw=3.4, label="selected (dashed = pregrasp)")],
           loc="lower left", ncol=6, frameon=False, fontsize=8.5, bbox_to_anchor=(0.01, 0.0))
fig.tight_layout(rect=(0, 0.04, 1, 0.94)); out = a.out or os.path.join(a.run, "grasps", "selection.png"); fig.savefig(out, dpi=105); print(out)
