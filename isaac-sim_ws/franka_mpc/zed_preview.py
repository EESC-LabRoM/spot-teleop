"""RGB + depth side by side for a franka_scene.py run:  /usr/bin/python3 zed_preview.py runs/workcell"""
import os, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt, numpy as np
from PIL import Image
run = os.path.abspath(sys.argv[1] if len(sys.argv) > 1 else "runs/workcell")
d = np.load(os.path.join(run, "zed_left_depth_m.npy")); rgb = np.asarray(Image.open(os.path.join(run, "zed_left_rgb.jpg")))
fig, ax = plt.subplots(1, 2, figsize=(16, 4.8), facecolor="#fcfcfb")
ax[0].imshow(rgb); ax[0].set_title(f"ZED X left: RGB ({rgb.shape[1]}x{rgb.shape[0]})", loc="left", fontsize=11, fontweight="bold")
im = ax[1].imshow(np.clip(d, 0, 4), cmap="viridis_r"); ax[1].set_title("ZED X left: depth to image plane (m), clipped at 4 m", loc="left", fontsize=11, fontweight="bold")
for a_ in ax: a_.set_xticks([]); a_.set_yticks([])
fig.colorbar(im, ax=ax[1], fraction=0.03, pad=0.01).set_label("m")
fig.tight_layout(); out = os.path.join(run, "zed_rgb_depth.png"); fig.savefig(out, dpi=100); print("wrote", out)
