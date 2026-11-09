"""ESDF slices (robot base frame) of the grasp region from map_check.py dumps, with the target's true outline: is the target in the map?
   /usr/bin/python3 plot_mask_slice.py runs/<run>[:map] ... [--target drill] [--out file.png]   (each arg: run dir, optional :pre|:post)"""
import argparse, json, os
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
ap = argparse.ArgumentParser(); ap.add_argument("runs", nargs="+"); ap.add_argument("--target", default="drill"); ap.add_argument("--out", required=True); a = ap.parse_args()
HERE = os.path.dirname(os.path.abspath(__file__)); root = np.array(json.load(open(os.path.join(HERE, "runs", "workcell", "scene.json")))["franka_base"])
INK, INK2, MUTED, BG = "#1d232b", "#4a525c", "#8a919a", "#fbfaf7"
plt.rcParams.update({"figure.facecolor": BG, "axes.facecolor": BG, "font.size": 9, "axes.titlesize": 10, "axes.titleweight": "bold", "axes.titlecolor": INK})
fig, ax = plt.subplots(2, len(a.runs), figsize=(5.2 * len(a.runs), 8), squeeze=False)
for c, spec in enumerate(a.runs):
    run, _, tag = spec.partition(":"); tag = tag or "pre"
    tri = np.load(os.path.join(run, "objects_start.npz"))[a.target].astype(np.float64).reshape(-1, 3) - root; lo, hi = tri.min(0), tri.max(0); ctr = (lo + hi) / 2
    Z = np.load(os.path.join(run, f"map_{tag}", "esdf.npz")); E, org, vs = Z["esdf"], Z["origin"], float(Z["voxel"])
    E = np.where(E <= -999, np.nan, E)
    for r, (ax_i, name) in enumerate(((1, "y"), (2, "z"))):
        # r=0: x-z slice at the drill's y centre; r=1: x-y slice at z 0.12
        if r == 0:
            j = int(np.clip((ctr[1] - org[1]) / vs, 0, E.shape[1] - 1)); S = E[:, j, :].T; ext = [org[0], org[0] + E.shape[0] * vs, org[2], org[2] + E.shape[2] * vs]; xl, yl = "x", "z"
            P = tri[:, [0, 2]]
        else:
            k = int(np.clip((0.12 - org[2]) / vs, 0, E.shape[2] - 1)); S = E[:, :, k].T; ext = [org[0], org[0] + E.shape[0] * vs, org[1], org[1] + E.shape[1] * vs]; xl, yl = "x", "y"
            P = tri[:, [0, 1]]
        a_ = ax[r, c]; im = a_.imshow(S, origin="lower", extent=ext, cmap="RdBu", vmin=-0.15, vmax=0.15)
        a_.scatter(P[::8, 0], P[::8, 1], s=0.5, color="#2a9d5c", alpha=0.5); a_.set_xlabel(xl + ", m"); a_.set_ylabel(yl + ", m")
        a_.set_title(f"{os.path.basename(run.rstrip('/'))} ({tag}): ESDF, {'x-z at the drill y' if r == 0 else 'x-y at z 12 cm'}" if r == 0 else "", loc="left")
fig.colorbar(im, ax=ax, shrink=0.6, label="ESDF, m (red = inside / occupied, blue = free; grey = unobserved)")
fig.suptitle("Is the drill (green outline: its true mesh) in the map the MPC plans on?", x=0.01, ha="left", fontsize=12, fontweight="bold", color=INK)
fig.savefig(a.out, dpi=100); print(a.out)
