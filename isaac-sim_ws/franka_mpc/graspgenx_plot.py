"""Plot graspgenx_test.py's output: each object's ZED point cloud with its top grasps drawn as Franka Hand wireframes
(hand-flange frame, fr3_hand = panda_hand: finger base at z 0.0584, fingers +-4 cm along y, fingertips at z 0.1034), two views per object.
   /usr/bin/python3 graspgenx_plot.py runs/graspgenx_franka [--top 5]
"""
import argparse, json, os

import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ap = argparse.ArgumentParser(); ap.add_argument("dir"); ap.add_argument("--top", type=int, default=5); a = ap.parse_args()
R = json.load(open(os.path.join(a.dir, "result.json")))
INK, INK2, MUTED, GRID, BG = "#1d232b", "#4a525c", "#8a919a", "#e4e1da", "#fbfaf7"
plt.rcParams.update({"figure.facecolor": BG, "axes.facecolor": BG, "font.size": 9, "axes.titlesize": 10, "axes.titleweight": "bold",
                     "axes.titlecolor": INK, "axes.labelcolor": INK2, "xtick.color": MUTED, "ytick.color": MUTED})
CP = np.array([[0, 0, 0], [0, 0, 0.0584], [0, 0.04, 0.0584], [0, 0.04, 0.1034], [0, -0.04, 0.0584], [0, -0.04, 0.1034]])   # hand flange: fingers slide along y
SEG = [(0, 1), (2, 4), (2, 3), (4, 5)]
names = {"veja": "blue bottle", "drill": "drill", "SM_Mug_B1": "mug", "beaker_500ml": "beaker", "small_KLT_visual_collision": "KLT bin",
         "SM_BottlePlasticC_01": "bottle 1", "SM_BottlePlasticC_02": "bottle 2"}
objs = list(R["objects"]); cmap = plt.get_cmap("viridis")
fig = plt.figure(figsize=(5.2 * len(objs), 9.4))
fig.suptitle(f"GraspGen-X, zero-shot on our sim scene: {R['gripper']} grasps from one {R['cam'].upper()} depth frame\n"
             f"top {a.top} of 100 by confidence; not yet filtered for the table, neighbours or reach", x=0.01, ha="left", fontsize=13, fontweight="bold", color=INK)
for c, name in enumerate(objs):
    Z = np.load(os.path.join(a.dir, f"{name}.npz")); pc, G, conf, S = Z["pc"], Z["grasps"], Z["conf"], Z["surface"]; r = R["objects"][name]
    for row, (el, az) in enumerate(((25, -60), (85, -90))):
        ax = fig.add_subplot(2, len(objs), row * len(objs) + c + 1, projection="3d"); ax.set_facecolor(BG)
        ax.scatter(*S.T, s=0.3, color=GRID, alpha=0.5, depthshade=False)
        ax.scatter(*pc.T, s=0.8, color=INK2, alpha=0.8, depthshade=False)
        for k in range(min(a.top, len(G)))[::-1]:
            W = (G[k][:3, :3] @ CP.T).T + G[k][:3, 3]; col = cmap(0.15 + 0.7 * (1 - k / max(1, a.top - 1)))
            for i, j in SEG: ax.plot(*W[[i, j]].T, color=col, lw=2.2 if k == 0 else 1.3)
        ctr = pc.mean(0); half = max(0.12, 0.75 * np.ptp(pc, 0).max())
        ax.set_xlim(ctr[0] - half, ctr[0] + half); ax.set_ylim(ctr[1] - half, ctr[1] + half); ax.set_zlim(max(-0.01, ctr[2] - half), ctr[2] + half)
        ax.set_box_aspect((1, 1, 1)); ax.view_init(el, az); ax.set_xticklabels([]); ax.set_yticklabels([]); ax.set_zticklabels([])
        if row == 0:
            ax.set_title(f"{names.get(name, name)}: {r['points']} points, top conf {r['conf_top'][0]:.2f}\n"
                         f"inference {r['infer_s']:.1f} s, peak {r['peak_alloc_mib']} MiB", loc="left")
fig.text(0.01, 0.01, "dark grey: the ZED's points on the object (cut out with the exact sim mesh, standing in for SAM2); light: the true "
         "surface; brightest wireframe = top grasp. Frame: fr3_link0.", color=MUTED, fontsize=8.5)
fig.tight_layout(rect=(0, 0.03, 1, 0.93)); out = os.path.join(a.dir, "grasps.png"); fig.savefig(out, dpi=105); print(out)
