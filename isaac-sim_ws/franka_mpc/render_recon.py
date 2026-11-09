"""Inspection images of the nvblox reconstruction around the grasp target, one column per map variant, plus the mask overlays.
Reads <run>/mesh_<tag>.npz (mesh_recorder.py: vertices, colors, triangles in the robot base frame) and the true mesh of the target
(objects_start.npz). Writes <run>/inspect/recon_<target>.png (3 views per variant: oblique, top, side; the target's true surface as green
dots, so a reconstruction that still contains the target shows a coloured surface where the dots are), recon_<target>_stats.json and, with
mask_inspect/ images from target_mask.py, <run>/inspect/mask_<cam>.png (RGB with the mask outlines, depth with the mask).
   /usr/bin/python3 render_recon.py runs/<run> [--target drill] [--variants main:"mask +3 px" b:"mask +12 px" c:"no mask"] [--pad 0.10]
"""
import argparse, glob, json, os
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np

ap = argparse.ArgumentParser(); ap.add_argument("run"); ap.add_argument("--target", default="drill")
ap.add_argument("--variants", nargs="+", default=["main:mask +3 px", "b:mask +12 px", "c:no mask"]); ap.add_argument("--pad", type=float, default=0.10); a = ap.parse_args()
HERE = os.path.dirname(os.path.abspath(__file__)); root = np.array(json.load(open(os.path.join(HERE, "runs", "workcell", "scene.json")))["franka_base"])
out = os.path.join(a.run, "inspect"); os.makedirs(out, exist_ok=True)
S = np.load(os.path.join(a.run, "objects_start.npz"))[a.target].astype(np.float64).reshape(-1, 3) - root
lo, hi = S.min(0) - a.pad, S.max(0) + a.pad; ctr = (S.min(0) + S.max(0)) / 2
INK, MUTED, BG = "#1d232b", "#8a919a", "#fbfaf7"
plt.rcParams.update({"figure.facecolor": BG, "axes.facecolor": BG, "font.size": 9, "axes.titlesize": 10, "axes.titleweight": "bold", "axes.titlecolor": INK})
vs = [v.partition(":") for v in a.variants]; vs = [(t, lab) for t, _, lab in vs if os.path.exists(os.path.join(a.run, f"mesh_{t}.npz"))]
fig = plt.figure(figsize=(5.0 * len(vs), 12.5)); stats = {}
fig.suptitle(f"nvblox reconstruction around the {a.target} (robot base frame; green dots = the {a.target}'s true surface)", x=0.01, ha="left", fontsize=12.5, fontweight="bold", color=INK)
rng = np.random.default_rng(0); Sd = S[rng.choice(len(S), min(len(S), 2500), replace=False)]
for c, (tag, lab) in enumerate(vs):
    M = np.load(os.path.join(a.run, f"mesh_{tag}.npz")); V, C, T = M["vertices"], M["colors"], M["triangles"]
    cen = V[T].mean(1); keep = np.all((cen > lo) & (cen < hi), 1); T, Cc = T[keep], C[T[keep]].mean(1) / 255.0
    # triangles inside the target's true box (1 cm shrunk) that are above the table: the reconstructed surface of the target
    inb = np.all((cen[keep] > S.min(0) + 0.01) & (cen[keep] < S.max(0) - 0.01), 1) & (cen[keep][:, 2] > 0.02)
    stats[tag] = {"label": lab, "triangles_in_view": int(len(T)), "triangles_inside_target_box_above_table": int(inb.sum())}
    for r, (el, az) in enumerate(((24, -62), (90, -90), (0, -90))):
        ax = fig.add_subplot(3, len(vs), r * len(vs) + c + 1, projection="3d"); ax.set_facecolor(BG)
        pc = Poly3DCollection(V[T], facecolors=Cc, edgecolor="none", alpha=1.0); ax.add_collection3d(pc)
        ax.scatter(*Sd.T, s=0.6, color="#2a9d5c", alpha=0.7, depthshade=False)
        ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1]); ax.set_zlim(max(lo[2], -0.02), hi[2]); ax.set_box_aspect((hi[0] - lo[0], hi[1] - lo[1], hi[2] - max(lo[2], -0.02)))
        ax.view_init(el, az); ax.set_xticklabels([]); ax.set_yticklabels([]); ax.set_zticklabels([])
        if r == 0: ax.set_title(f"{lab}\n{stats[tag]['triangles_inside_target_box_above_table']} mesh triangles inside the {a.target}'s box", loc="left")
json.dump(stats, open(os.path.join(out, f"recon_{a.target}_stats.json"), "w"), indent=1)
fig.tight_layout(rect=(0, 0, 1, 0.96)); fig.savefig(os.path.join(out, f"recon_{a.target}.png"), dpi=95); print(os.path.join(out, f"recon_{a.target}.png"), json.dumps(stats))
# mask images: the RGB with the outlines (first snapshot per camera) and the depth overlay, side by side
for cam in ("zed", "zed2"):
    rgb = sorted(glob.glob(os.path.join(a.run, "mask_inspect", f"{cam}_t*_rgb_contours.jpg"))); dep = sorted(glob.glob(os.path.join(a.run, "mask_inspect", f"{cam}_t*_depth_overlay.jpg")))
    if not rgb: continue
    import cv2
    im, ov = cv2.imread(rgb[0]), cv2.imread(dep[0]) if dep else None
    ys, xs = np.nonzero((im[..., 2] > 200) & (im[..., 0] < 60) & (im[..., 1] < 60)); pad = 140
    if len(xs): x0, x1, y0, y1 = max(xs.min() - pad, 0), xs.max() + pad, max(ys.min() - pad, 0), ys.max() + pad
    else: x0, x1, y0, y1 = 0, im.shape[1], 0, im.shape[0]
    f2 = plt.figure(figsize=(12, 5.2)); f2.suptitle(f"{cam}: the {a.target} mask (outlines: red = +3 px, orange = larger growth) on the RGB, and on the depth the map is built from",
                                                        x=0.01, ha="left", fontsize=11, fontweight="bold", color=INK)
    for k, (img, ttl) in enumerate(((im, "RGB + mask outlines"), (ov, "depth + mask (red)"))):
        if img is None: continue
        a2 = f2.add_subplot(1, 2, k + 1); a2.imshow(cv2.cvtColor(img[y0:y1, x0:x1], cv2.COLOR_BGR2RGB)); a2.set_title(ttl, loc="left"); a2.axis("off")
    f2.tight_layout(rect=(0, 0, 1, 0.93)); f2.savefig(os.path.join(out, f"mask_{cam}.png"), dpi=95); print(os.path.join(out, f"mask_{cam}.png"))
