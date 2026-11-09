"""ZED wrapper output (check_topics.py) next to Isaac's ground-truth depth for the same camera (franka_scene.py run).
   /usr/bin/python3 preview_check.py check ../isaac-sim_ws/franka_mpc/runs/workcell"""
import json, os, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt, numpy as np
from PIL import Image
chk, gt_run = sys.argv[1], sys.argv[2]
rgb, d = np.load(os.path.join(chk, "rgb.npy")), np.load(os.path.join(chk, "depth_m.npy"))
gt = np.array(Image.fromarray(np.load(os.path.join(gt_run, "zed_left_depth_m.npy"))).resize(d.shape[::-1], Image.NEAREST))
ok = np.isfinite(d) & (d > 0) & np.isfinite(gt) & (gt > 0) & (gt < 4)
err = np.where(ok, np.abs(d - gt) * 100, np.nan)
fig, ax = plt.subplots(1, 3, figsize=(19, 4.3), facecolor="#fcfcfb")
ax[0].imshow(rgb); ax[0].set_title("ROS /zed/zed_node/rgb/color/rect/image (1920x1080)", loc="left", fontsize=10, fontweight="bold")
im1 = ax[1].imshow(np.clip(np.nan_to_num(d, nan=0, posinf=0), 0, 4), cmap="viridis_r")
ax[1].set_title("ROS depth_registered: ZED SDK stereo (NEURAL_LIGHT), m", loc="left", fontsize=10, fontweight="bold")
im2 = ax[2].imshow(err, cmap="magma", vmin=0, vmax=10)
ax[2].set_title(f"|ZED stereo - Isaac exact| (cm), < 4 m: median {np.nanmedian(err):.1f}, p90 {np.nanpercentile(err, 90):.1f}",
                loc="left", fontsize=10, fontweight="bold")
for a_ in ax: a_.set_xticks([]); a_.set_yticks([])
fig.colorbar(im1, ax=ax[1], fraction=0.03, pad=0.01).set_label("m"); fig.colorbar(im2, ax=ax[2], fraction=0.03, pad=0.01).set_label("cm")
fig.tight_layout(); out = os.path.join(chk, "zed_ros_preview.png"); fig.savefig(out, dpi=95)
print("wrote", out, "| err cm median", round(float(np.nanmedian(err)), 2), "p90", round(float(np.nanpercentile(err, 90)), 2), "p99", round(float(np.nanpercentile(err, 99)), 2))
