"""Picture of the nvblox map (map_check.py output) in the Franka base frame: top view of the mapped surfaces coloured by
height with the objects' known positions, a side view, and the ESDF slice 15 cm above the table.
    /usr/bin/python3 map_plot.py out ../../isaac-sim_ws/franka_mpc/runs/workcell
"""
import json, os, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt, numpy as np
out, wc = sys.argv[1], sys.argv[2]
E = np.load(os.path.join(out, "esdf.npz")); esdf, org, vs = E["esdf"], E["origin"], float(E["voxel"])
scene = json.load(open(os.path.join(wc, "scene.json"))); chk = json.load(open(os.path.join(out, "map_check.json")))
ext = json.load(open(os.path.join(out, "extrinsics.json")))
SURF, INK, INK2, MUTED, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9"
plt.rcParams.update({"figure.facecolor": SURF, "axes.facecolor": SURF, "axes.edgecolor": "#c3c2b7", "axes.labelcolor": INK2,
                     "xtick.color": MUTED, "ytick.color": MUTED, "font.size": 10, "axes.titlesize": 11, "axes.titleweight": "bold",
                     "axes.titlecolor": INK, "axes.spines.top": False, "axes.spines.right": False})
known = np.isfinite(esdf) & (esdf > -999)
ix = np.argwhere(known & (np.abs(esdf) <= vs)); P = org + (ix + 0.5) * vs
NAMES = {"drill": "drill", "SM_Mug_B1": "mug", "veja": "veja", "beaker_500ml": "beaker", "small_KLT_visual_collision": "bin",
         "SM_BottlePlasticC_01": "bottle 1", "SM_BottlePlasticC_02": "bottle 2"}

fig = plt.figure(figsize=(18, 6.6))
gs = fig.add_gridspec(1, 3, width_ratios=[1.0, 1.15, 1.0], wspace=0.35, left=0.045, right=0.955, top=0.83, bottom=0.1)
tc = chk["tf_check"]; t = chk.get("table_top_z_m", {})
fig.suptitle("nvblox map of the Franka workcell from the simulated ZED X (stereo depth), robot base frame fr3_link0",
             x=0.05, ha="left", fontsize=14, fontweight="bold", color=INK)
fig.text(0.05, 0.9, f"2 cm voxels · ESDF service {chk['esdf']['call_ms']} ms for {' x '.join(str(s) for _, s in chk['esdf']['layout'])} voxels · "
         f"camera at {tc['camera_in_base_m']} m, TF vs Isaac camera {tc['position_err_mm']} mm / {tc['rotation_err_deg']} deg · "
         f"table top mapped at z = {t.get('peak')} m (true 0)", ha="left", color=INK2, fontsize=10)

ax = fig.add_subplot(gs[0, 0])                                     # top view: highest surface voxel per column
H = np.full(esdf.shape[:2], np.nan)
for (i, j, k), p in zip(ix, P):
    if np.isnan(H[i, j]) or p[2] > H[i, j]: H[i, j] = p[2]
im = ax.imshow(H.T, origin="lower", cmap="viridis", vmin=-0.05, vmax=0.45, interpolation="nearest",
               extent=[org[0], org[0] + esdf.shape[0] * vs, org[1], org[1] + esdf.shape[1] * vs])
for n, o in scene["objects"].items():
    x, y = o["forward_left_of_base_m"]
    ax.plot(x, y, "o", ms=9, mfc="none", mec=INK, mew=1.4)
    ax.text(x + 0.03, y + 0.02, NAMES.get(n, n), color=INK, fontsize=9, fontweight="bold")
ax.plot(0, 0, "s", ms=9, mfc=INK, mec=SURF); ax.text(0.02, -0.06, "Franka base", color=INK, fontsize=9)
ax.set_xlabel("x forward (m)"); ax.set_ylabel("y left (m)"); ax.set_title("Top view: height of the mapped surface", loc="left")
fig.colorbar(im, ax=ax, fraction=0.045, pad=0.02).set_label("height above the table top (m)")
ax.text(0.01, 0.01, "circles: true object positions", transform=ax.transAxes, color=INK2, fontsize=9)

ax = fig.add_subplot(gs[0, 1])                                     # side view
ax.scatter(P[:, 0], P[:, 2], s=3, c=P[:, 1], cmap="viridis", alpha=0.5, linewidths=0)
ax.axhline(0, color=MUTED, lw=1, ls="--"); ax.text(org[0] + 0.01, 0.006, "true table top", color=MUTED, fontsize=9)
for n, o in scene["objects"].items():
    x = o["forward_left_of_base_m"][0]; top = 2 * o["centre_world"][2] - o["bottom_z"] - scene["table_top_z"]
    ax.plot([x - 0.04, x + 0.04], [top, top], color=INK, lw=1.4)
ax.set_xlabel("x forward (m)"); ax.set_ylabel("z (m)"); ax.set_aspect("equal", adjustable="datalim"); ax.grid(True, color=GRID, lw=0.6)
ax.set_title("Side view: surface voxels (black ticks: true object tops)", loc="left")

ax = fig.add_subplot(gs[0, 2])                                     # ESDF slice
kz = int(round((0.15 - org[2]) / vs - 0.5)); S = np.where(known[:, :, kz], esdf[:, :, kz], np.nan)
im = ax.imshow(S.T, origin="lower", cmap="magma", vmin=0, vmax=0.4, interpolation="nearest",
               extent=[org[0], org[0] + esdf.shape[0] * vs, org[1], org[1] + esdf.shape[1] * vs])
ax.set_xlabel("x forward (m)"); ax.set_ylabel("y left (m)")
ax.set_title(f"ESDF slice at z = {org[2] + (kz + 0.5) * vs:.2f} m", loc="left")
fig.colorbar(im, ax=ax, fraction=0.045, pad=0.03).set_label("distance to nearest obstacle (m); blank = unobserved")
p = os.path.join(out, "map_check.png"); fig.savefig(p, dpi=105); print("wrote", p, "| surface voxels", len(P))
