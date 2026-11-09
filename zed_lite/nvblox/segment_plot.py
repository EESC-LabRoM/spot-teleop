"""Overlay of segment_test.py masks on the ZED frame:  /usr/bin/python3 segment_plot.py out/segtest ../check"""
import json, os, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt, numpy as np
st, chk = sys.argv[1], sys.argv[2]
rgb = np.load(os.path.join(chk, "rgb.npy")); d = np.load(os.path.join(chk, "depth_m.npy"))
m02, m05, m20 = (np.load(os.path.join(st, f"robot_mask_thr{t}.npy")) for t in ("02", "05", "20"))
wd = np.load(os.path.join(st, "world_depth_thr05_m.npy")); R = json.load(open(os.path.join(st, "segment_test.json")))
fig, ax = plt.subplots(1, 3, figsize=(19, 4.6), facecolor="#fcfcfb")
ov = rgb.astype(float) / 255
ov[m05] = 0.35 * ov[m05] + 0.65 * np.array([0.92, 0.41, 0.20])                       # removed at 5 cm
extra = m20 & ~m05; ov[extra] = 0.35 * ov[extra] + 0.65 * np.array([0.16, 0.47, 0.84])  # removed only at 20 cm
ax[0].imshow(ov); ax[0].set_title("Removed as robot: orange at 5 cm threshold, blue = extra at 20 cm", loc="left", fontsize=10, fontweight="bold")
ax[1].imshow(np.clip(np.nan_to_num(d, nan=0, posinf=0), 0, 3), cmap="viridis_r"); ax[1].set_title("ZED depth in (stereo)", loc="left", fontsize=10, fontweight="bold")
ax[2].imshow(np.clip(np.nan_to_num(wd, nan=0, posinf=0), 0, 3), cmap="viridis_r"); ax[2].set_title("world_depth out (5 cm): what nvblox would integrate", loc="left", fontsize=10, fontweight="bold")
for a_ in ax: a_.set_xticks([]); a_.set_yticks([])
fig.suptitle(f"cuRobo RobotSegmenter on a recorded ZED frame: Franka at retract, {R['spheres']} spheres, masked px 2 cm {R['thr_0.02']['masked_px']}, "
             f"5 cm {R['thr_0.05']['masked_px']}, 20 cm {R['thr_0.2']['masked_px']}, ~8 ms/frame", x=0.01, ha="left", fontsize=11, fontweight="bold")
fig.tight_layout(); p = os.path.join(st, "segment_test.png"); fig.savefig(p, dpi=95); print("wrote", p)
