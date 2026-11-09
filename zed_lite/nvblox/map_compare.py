"""Self-filter check: obstacle voxels inside the volume the Franka swept (zed_stream.py --arm sweep) in the nvblox map
with the cuRobo segmenter (out/filtered) vs the raw ZED depth (out/raw). Swept volume = cuRobo franka.yml spheres along
the sweep + 3 cm, above the table plane. Runs with the curobo-isaac6 venv on the host:
    ~/spot-teleop/isaac-sim_ws/curobo-isaac6/.venv/bin/python map_compare.py out ../../isaac-sim_ws/franka_mpc/runs/workcell
"""
import json, os, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt, numpy as np, torch
from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel
from curobo.types.base import TensorDeviceType
from curobo.types.robot import RobotConfig
from curobo.util_file import get_robot_configs_path, join_path, load_yaml

out, wc = sys.argv[1], sys.argv[2]
MARGIN, TABLE_EXCL = 0.03, 0.03                              # m: around the spheres; ignore the table plane (z < 3 cm)
ta = TensorDeviceType()
km = CudaRobotModel(RobotConfig.from_dict(load_yaml(join_path(get_robot_configs_path(), "fr3.yml"))["robot_cfg"], ta).kinematics)
R = np.array([0.0, -1.3, 0.0, -2.5, 0.0, 1.0, 0.0])           # same sweep as zed_stream.py
AMP, PER = np.array([0.8, 0.25, 0.0, 0.35, 0.0, 0.0, 0.0]), np.array([14.0, 9.0, 1.0, 11.0, 1.0, 1.0, 1.0])
ts = np.arange(0, 160, 0.2); Q = R + AMP * np.sin(2 * np.pi * ts[:, None] / PER)
sph = km.get_state(ta.to_device(Q)).link_spheres_tensor.reshape(-1, 4)
sph = sph[sph[:, 3] > 0]


def load(tag):
    E = np.load(os.path.join(out, tag, "esdf.npz")); esdf, org, vs = E["esdf"], E["origin"], float(E["voxel"])
    known = np.isfinite(esdf) & (esdf > -999); occ = known & (esdf <= vs)
    ix = np.argwhere(occ); P = org + (ix + 0.5) * vs
    Pt = ta.to_device(P.astype(np.float32)); dmin = torch.full((len(P),), 9.0, device=Pt.device)
    for c in torch.split(sph, 4096):                         # distance to the nearest swept sphere surface
        dmin = torch.minimum(dmin, (torch.cdist(Pt, c[:, :3]) - c[:, 3]).min(dim=1).values)
    inside = (dmin.cpu().numpy() <= MARGIN) & (P[:, 2] > TABLE_EXCL)
    return P, inside, vs, json.load(open(os.path.join(out, tag, "map_check.json")))


res = {}
data = {t: load(t) for t in ("raw", "filtered")}
for t, (P, inside, vs, chk) in data.items():
    res[t] = {"obstacle_voxels_total": int(len(P)), "obstacle_voxels_in_arm_swept_volume": int(inside.sum()),
              "objects_top_z_m": {k: v["top_z_m"] for k, v in chk["objects"].items()}}
r, f = res["raw"]["obstacle_voxels_in_arm_swept_volume"], res["filtered"]["obstacle_voxels_in_arm_swept_volume"]
res["arm_voxels_removed_frac"] = round(1 - f / max(r, 1), 3)
json.dump(res, open(os.path.join(out, "map_compare.json"), "w"), indent=1); print(json.dumps(res, indent=1))

SURF, INK, INK2, MUTED = "#fcfcfb", "#0b0b0b", "#52514e", "#898781"
plt.rcParams.update({"figure.facecolor": SURF, "axes.facecolor": SURF, "axes.edgecolor": "#c3c2b7", "axes.labelcolor": INK2,
                     "xtick.color": MUTED, "ytick.color": MUTED, "font.size": 10, "axes.titlesize": 11, "axes.titleweight": "bold",
                     "axes.titlecolor": INK, "axes.spines.top": False, "axes.spines.right": False})
fig, axs = plt.subplots(1, 2, figsize=(15, 6.2), sharey=True)
S = sph.cpu().numpy()
for ax, t in zip(axs, ("raw", "filtered")):
    P, inside, vs, _ = data[t]
    ax.scatter(S[:, 0], S[:, 2], s=1, color="#e1e0d9", zorder=0)
    ax.scatter(P[~inside, 0], P[~inside, 2], s=4, color="#2a78d6", linewidths=0, label="map: rest of the scene")
    ax.scatter(P[inside, 0], P[inside, 2], s=6, color="#eb6834", linewidths=0, label="map: inside the arm's swept volume")
    ax.set_xlabel("x forward (m)"); ax.set_aspect("equal", adjustable="datalim"); ax.grid(True, color="#e1e0d9", lw=0.6)
    name = "raw ZED depth (no self-filter)" if t == "raw" else "cuRobo robot segmenter in front of nvblox"
    ax.set_title(f"{name}: {inside.sum()} obstacle voxels in the arm's volume", loc="left")
axs[0].set_ylabel("z above the table top (m)"); axs[0].legend(loc="upper right", fontsize=9, frameon=False)
axs[1].text(0.01, 0.01, "light grey: cuRobo spheres along the sweep", transform=axs[1].transAxes, color=INK2, fontsize=9)
fig.suptitle(f"nvblox side view (all y) while the Franka swings: self-filter removes {res['arm_voxels_removed_frac'] * 100:.0f}% of the arm voxels",
             x=0.01, ha="left", fontsize=13, fontweight="bold", color=INK)
fig.tight_layout(); p = os.path.join(out, "map_compare.png"); fig.savefig(p, dpi=105); print("wrote", p)
