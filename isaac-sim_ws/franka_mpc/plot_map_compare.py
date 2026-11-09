"""Map completeness, 1 vs 2 ZEDs: share of each object's true surface the map misses by more than 2 cm (map_error.json
from map_coverage.py), front half vs robot-facing half.   /usr/bin/python3 plot_map_compare.py runs/A runs/B -> runs/map_1v2.png"""
import json, os, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
A, B = sys.argv[1], sys.argv[2]
ea, eb = (json.load(open(os.path.join(r, "map_error.json")))["objects"] for r in (A, B))
NAMES = {"veja": "blue bottle (obstacle)", "drill": "drill", "beaker_500ml": "beaker", "SM_Mug_B1": "mug",
         "small_KLT_visual_collision": "bin", "SM_BottlePlasticC_01": "bin bottle 1", "SM_BottlePlasticC_02": "bin bottle 2"}
objs = list(NAMES)
SURF, INK, INK2, MUTED, GRID, AXIS = "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7"
S1, S2 = "#2a78d6", "#eb6834"
plt.rcParams.update({"figure.facecolor": SURF, "axes.facecolor": SURF, "axes.edgecolor": AXIS, "axes.labelcolor": INK2,
                     "xtick.color": MUTED, "ytick.color": INK2, "grid.color": GRID, "grid.linewidth": 0.6, "font.size": 10,
                     "axes.titlesize": 11, "axes.titleweight": "bold", "axes.titlecolor": INK, "axes.spines.top": False,
                     "axes.spines.right": False, "legend.frameon": False, "legend.labelcolor": INK2})
fig, ax = plt.subplots(1, 2, figsize=(12, 4.6), sharey=True)
fig.suptitle("Second ZED: how much of each object the map misses (by more than 2 cm)", x=0.01, ha="left", fontsize=14,
             fontweight="bold", color=INK)
fig.text(0.01, 0.885, "Map ESDF read on Isaac's true object surfaces; same scene and sweep, self-filter 0.15 m, 2 cm voxels",
         ha="left", color=INK2)
y = np.arange(len(objs)); h = 0.36
for a, half, title in ((ax[0], "front_half", "Half facing the front ZED"), (ax[1], "robot_side_half", "Half facing the robot")):
    va = [ea[o][half]["missed_over_2cm_pct"] for o in objs]; vb = [eb[o][half]["missed_over_2cm_pct"] for o in objs]
    a.barh(y - h / 2 - 0.01, va, h, color=S1, label="1 ZED (front)")
    a.barh(y + h / 2 + 0.01, vb, h, color=S2, label="2 ZEDs (front + behind-left)")
    for yi, v in zip(y - h / 2, va): a.text(v + 1, yi, f"{v:.0f}", va="center", fontsize=8.5, color=INK2)
    for yi, v in zip(y + h / 2, vb): a.text(v + 1, yi, f"{v:.0f}", va="center", fontsize=8.5, color=INK2)
    a.set_title(title, loc="left"); a.set_xlim(0, 100); a.set_xlabel("% of the surface missed"); a.grid(axis="x")
ax[0].set_yticks(y); ax[0].set_yticklabels([NAMES[o] for o in objs]); ax[0].invert_yaxis()
ax[1].legend(loc="lower right", fontsize=9)
fig.tight_layout(rect=(0, 0, 1, 0.86))
out = os.path.join(os.path.dirname(os.path.abspath(A)), "map_1v2.png"); fig.savefig(out, dpi=110); print(out)
