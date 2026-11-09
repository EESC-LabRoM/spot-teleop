"""Plot the ZED placement search (zed_placement.py) and, when present, the Isaac map runs that check it
(map_coverage.py on runs/map_pair_current and runs/map_pair_opt).
   curobo-isaac6/.venv/bin/python -I plot_placement.py [runs/zed_placement]      -> <dir>/placement.png
"""
import json, os, sys

import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import torch

HERE = os.path.dirname(os.path.abspath(__file__)); D = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "runs", "zed_placement")
R = json.load(open(os.path.join(D, "placement.json"))); Z = np.load(os.path.join(D, "visibility.npz"), allow_pickle=True)
C, n = Z["C"], int(Z["n"]); V = np.unpackbits(Z["V"], axis=1)[:, :n].astype(bool); OWN, names = Z["OWN"], list(Z["names"])
root, centre = Z["root"], Z["centre"]
INK, INK2, MUTED, GRID, BG = "#1d232b", "#4a525c", "#8a919a", "#e4e1da", "#fbfaf7"
CUR, OPT = "#d9622b", "#2a6fdb"
plt.rcParams.update({"figure.facecolor": BG, "axes.facecolor": BG, "axes.edgecolor": MUTED, "axes.labelcolor": INK2, "xtick.color": MUTED,
                     "ytick.color": MUTED, "grid.color": GRID, "font.size": 10, "axes.titlesize": 11, "axes.titleweight": "bold",
                     "axes.titlecolor": INK, "axes.spines.top": False, "axes.spines.right": False, "axes.grid": True, "legend.frameon": False})
pretty = {"veja": "blue bottle", "drill": "drill", "SM_Mug_B1": "mug", "beaker_500ml": "beaker", "SM_BottlePlasticC_01": "bottle 1",
          "SM_BottlePlasticC_02": "bottle 2", "small_KLT_visual_collision": "KLT bin"}

# best pair score reachable from each stand (its best partner), max over heights: where a ZED should go
w = np.where(OWN >= 0, 0.75 / len(names) / np.bincount(OWN[OWN >= 0])[np.maximum(OWN, 0)], 0.25 / (OWN == -1).sum())
Vt = torch.tensor(V, device="cuda", dtype=torch.float32); wt = torch.tensor(w, device="cuda", dtype=torch.float32)
s1 = Vt @ wt; pair = s1[:, None] + s1[None, :] - (Vt * wt) @ Vt.T; best_with = pair.max(1).values.cpu().numpy()
xy = np.round(C[:, :2], 3); keys, inv = np.unique(xy, axis=0, return_inverse=True); cell = np.full(len(keys), -1.0)
np.maximum.at(cell, inv.ravel(), best_with)

fig = plt.figure(figsize=(17, 6.6)); gs = fig.add_gridspec(1, 3, width_ratios=[1.35, 1, 0.8])
ax0, ax1, ax2 = fig.add_subplot(gs[0]), fig.add_subplot(gs[1]), fig.add_subplot(gs[2])
cp, op = R["current"]["pair"], R["best_pair"]
TITLE = (f"ZED placement: the searched pair leaves {100 - op['objects_mean'] * 100:.0f}% of the objects' surface unmapped vs "
         f"{100 - cp['objects_mean'] * 100:.0f}% for the hand-placed pair (predicted)")
fig.text(0.01, 0.885, f"Ray casting on the true scene: both stereo eyes must see a point (FOV {R['settings']['fov_deg'][0]} x {R['settings']['fov_deg'][1]} deg, "
         f"<= {R['settings']['range_m']} m, incidence <= {R['settings']['incidence_deg']:.0f} deg). A surface counts as mapped within 2 cm of a seen point,\n"
         f"as map_coverage.py scores the real map. {R['settings']['candidates']} floor stands ({len(R['settings']['heights_m'])} heights, outside the arm's reach, "
         f"clear of the furniture), arm at retract.", ha="left", color=INK2, va="bottom")

S = np.load(os.path.join(HERE, "runs", "static_scene", "static_scene.npz"))     # furniture footprint (0.05-2 m high)
for k in S.files:
    if "Warehouse" in k: continue
    T = S[k]; m = (T[..., 2].max(1) > 0.05) & (T[..., 2].min(1) < 2.0); T = T[m]
    if len(T):
        x0, x1, y0, y1 = T[..., 0].min(), T[..., 0].max(), T[..., 1].min(), T[..., 1].max()
        ax0.add_patch(plt.Rectangle((x0, y0), x1 - x0, y1 - y0, color="#d8d3c8", zorder=0))
sc = ax0.scatter(keys[:, 0], keys[:, 1], c=cell, cmap="Blues", s=26, marker="s", vmin=np.percentile(cell, 5), vmax=cell.max(), zorder=1)
cb = fig.colorbar(sc, ax=ax0, fraction=0.04, pad=0.01); cb.set_label("best pair score with a ZED here", color=INK2); cb.outline.set_visible(False)
P = Z["P"]; ob = OWN >= 0; ax0.scatter(P[ob, 0][::6], P[ob, 1][::6], s=1, color=INK2, zorder=2)
ax0.add_patch(plt.Circle(root[:2], 0.06, color=INK, zorder=3)); ax0.text(root[0] - 0.08, root[1] - 0.08, "Franka", ha="right", va="top", fontsize=9, color=INK)
ax0.add_patch(plt.Circle(root[:2], 0.855, fill=False, ls=":", color=MUTED, zorder=2))


def cam(ax, eye, at, color, label, filled, dx=0.08, dy=0.08, ha="left"):
    eye, at = np.asarray(eye), np.asarray(at); d = (at - eye)[:2]; d = d / np.linalg.norm(d) * 0.35
    ax.annotate("", eye[:2] + d, eye[:2], arrowprops=dict(arrowstyle="-|>", color=color, lw=1.6), zorder=5)
    ax.scatter(*eye[:2], s=110, marker="o", facecolor=color if filled else BG, edgecolor=color, linewidth=2, zorder=6)
    ax.text(eye[0] + dx, eye[1] + dy, f"{label}, {eye[2]:.1f} m high", fontsize=9, color=color, ha=ha, va="center", zorder=7,
            bbox=dict(boxstyle="round,pad=0.15", fc=BG, ec="none", alpha=0.85))


cur = R["current"]; cam(ax0, cur["zed"]["eye"], centre, CUR, "ZED 1 now", False, 0.12, 0.12)
cam(ax0, cur["zed2"]["eye"], centre, CUR, "ZED 2 now", False, -0.12, -0.12, "right")
cam(ax0, op["b"], centre, OPT, "ZED 1 searched", True, 0.12, -0.14); cam(ax0, op["a"], centre, OPT, "ZED 2 searched", True, -0.12, 0.16, "right")
ax0.set_aspect("equal"); ax0.set_xlim(-3.05, 2.0); ax0.set_ylim(-2.65, 1.65); ax0.set_xlabel("world x (m)"); ax0.set_ylabel("world y (m)")
ax0.set_title("Seen from above: where the stands can go", loc="left"); ax0.grid(False)

# per object: unmapped share, predicted (and measured in Isaac when the map runs exist)
meas = {}
for tag, run in (("current", "map_pair_current"), ("searched", "map_pair_opt")):
    f = os.path.join(HERE, "runs", run, "map_error.json")
    if os.path.exists(f):
        M = json.load(open(f))["objects"]; meas[tag] = {k: (M[k]["front_half"]["missed_over_2cm_pct"] + M[k]["robot_side_half"]["missed_over_2cm_pct"]) / 2 for k in M}
if len(meas) == 2:
    mc, mo = (np.mean(list(meas[t].values())) for t in ("current", "searched"))
    TITLE = (f"ZED placement, measured in Isaac: {mo:.0f}% of the objects' surface unmapped with the searched pair vs {mc:.0f}% "
             f"hand-placed (predicted {100 - op['objects_mean'] * 100:.0f}% vs {100 - cp['objects_mean'] * 100:.0f}%)")
fig.suptitle(TITLE, x=0.01, ha="left", fontsize=14, fontweight="bold", color=INK)
rows = list(pretty); yy = np.arange(len(rows)); bars = [("current", "hand-placed, predicted", CUR, cp), ("searched", "searched, predicted", OPT, op)]
hgt = 0.2 if meas else 0.36
for b, (tag, lab, col, sc_) in enumerate(bars):
    v = [100 * (1 - sc_["per_object"][k]) for k in rows]; off = (b - 0.5) * (2 * hgt if meas else hgt)
    ax1.barh(yy + off - (hgt / 2 if meas else 0), v, hgt, color=col, alpha=0.45 if meas else 1, label=lab)
    if tag in meas:
        mv = [meas[tag][k] for k in rows]; ax1.barh(yy + off + hgt / 2, mv, hgt, color=col, label=lab.replace("predicted", "measured in Isaac"))
        for i, x in enumerate(mv): ax1.text(x + 0.5, i + off + hgt / 2, f"{x:.0f}", va="center", fontsize=7.5, color=INK2)
ax1.set_yticks(yy, [pretty[k] for k in rows]); ax1.invert_yaxis(); ax1.set_xlabel("object surface unmapped by > 2 cm (%)")
ax1.set_title("Per object", loc="left"); ax1.grid(axis="y", visible=False); ax1.legend(loc="lower right", fontsize=8.5)

ks = [("1 ZED", R["current"]["zed"]["objects_mean"], R["best_single"]["objects_mean"]), ("2 ZEDs", cp["objects_mean"], op["objects_mean"]),
      ("3 ZEDs", None, R["best_third_after_best_pair"]["objects_mean"])]
x = np.arange(len(ks))
ax2.bar(x - 0.19, [100 * (1 - k[1]) if k[1] is not None else 0 for k in ks], 0.36, color=CUR, label="hand-placed")
ax2.bar(x + 0.19, [100 * (1 - k[2]) for k in ks], 0.36, color=OPT, label="searched")
for i, k in enumerate(ks):
    if k[1] is not None: ax2.text(i - 0.19, 100 * (1 - k[1]) + 0.6, f"{100 * (1 - k[1]):.0f}%", ha="center", fontsize=8.5, color=INK2)
    ax2.text(i + 0.19, 100 * (1 - k[2]) + 0.6, f"{100 * (1 - k[2]):.0f}%", ha="center", fontsize=8.5, color=INK2)
ax2.set_xticks(x, [k[0] for k in ks]); ax2.set_ylabel("object surface unmapped (%), predicted"); ax2.grid(axis="x", visible=False)
ax2.set_title("How many cameras", loc="left"); ax2.legend(fontsize=8.5)
v_ = R.get("validation")
if v_: fig.text(0.99, 0.015, f"Model check against the measured maps (28 object halves): correlation {v_['corr']:.2f}, it predicts {100 * v_['bias']:+.0f} points "
                f"more coverage than measured", ha="right", fontsize=8.5, color=MUTED)
fig.tight_layout(rect=(0, 0.03, 1, 0.87)); out = os.path.join(D, "placement.png"); fig.savefig(out, dpi=110); print(out)
