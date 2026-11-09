"""How the arm got past the obstacles in a run_grasp.sh run with a transit waypoint (VIA): the commanded hand path (what the goal source
asked for, straight legs) against the executed one, and the clearance of the whole arm (cuRobo collision spheres on the measured joints,
no buffer) to each real object's mesh (Isaac's exact meshes, not the map), the grasped object excluded.
   curobo-isaac6/.venv/bin/python -I avoid_check.py runs/<run> [--target drill]  -> avoid_check.png, avoid_check.json"""
import argparse, json, os, re
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np, torch
from scipy.spatial import cKDTree
from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel, CudaRobotModelConfig
from curobo.types.base import TensorDeviceType
from curobo.util_file import get_robot_configs_path, join_path, load_yaml

ap = argparse.ArgumentParser(); ap.add_argument("run"); ap.add_argument("--target", default="drill"); a = ap.parse_args()
T = np.load(os.path.join(a.run, "trace.npz")); M = dict(np.load(os.path.join(a.run, "objects_start.npz")))
ROOT = np.array(json.load(open(os.path.join(a.run, "RESULT_isaac.json")))["root"])
t0 = float(re.search(r"start at sim t=([0-9.]+)", open(os.path.join(a.run, "goal.log")).read()).group(1)); t = T["t"] - t0
TCP = 0.1034; rng = np.random.default_rng(0); S = {}
for n, tri in M.items():
    if n == a.target: continue
    tri = tri.astype(np.float64) - ROOT; cr = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]); ar = 0.5 * np.linalg.norm(cr, axis=1); ok = ar > 1e-12
    k = int(min(300000, max(20000, ar.sum() / 1.5e-3 ** 2))); f = rng.choice(np.nonzero(ok)[0], k, p=ar[ok] / ar[ok].sum())
    u, v = rng.random(k), rng.random(k); s = u + v > 1; u[s], v[s] = 1 - u[s], 1 - v[s]
    S[n] = tri[f, 0] + u[:, None] * (tri[f, 1] - tri[f, 0]) + v[:, None] * (tri[f, 2] - tri[f, 0])
cfg = load_yaml(join_path(get_robot_configs_path(), "fr3.yml"))["robot_cfg"]; cfg["kinematics"]["collision_sphere_buffer"] = 0.0
kin = CudaRobotModel(CudaRobotModelConfig.from_data_dict(cfg["kinematics"], tensor_args=TensorDeviceType()))
q = np.concatenate([T["q"], np.full((len(T["q"]), 2), 0.04)], 1)
sph = kin.get_state(torch.tensor(q, device="cuda", dtype=torch.float32)).link_spheres_tensor.cpu().numpy()   # (n, S, 4)
sph = sph[:, sph[0, :, 3] > 0]                                                                                # drop unused (radius 0)
clear = {}
for n, P in S.items():
    tr = cKDTree(P); d, _ = tr.query(sph[..., :3].reshape(-1, 3)); clear[n] = (d.reshape(len(sph), -1) - sph[..., 3]).min(1)   # sphere surface to mesh
res = {n: {"min_clearance_cm": round(float(c.min()) * 100, 2), "at_sim_s": round(float(t[c.argmin()]), 2)} for n, c in clear.items()}
trans = t < 8.0   # transit window of this sequence (to waypoint + to pregrasp)
res_tr = {n: round(float(c[trans].min()) * 100, 2) for n, c in clear.items()}
tgt_tip = T["target"] + TCP * np.stack([2 * (T["target_quat"][:, 0] * T["target_quat"][:, 2] + T["target_quat"][:, 3] * T["target_quat"][:, 1]),
                                         2 * (T["target_quat"][:, 1] * T["target_quat"][:, 2] - T["target_quat"][:, 3] * T["target_quat"][:, 0]),
                                         1 - 2 * (T["target_quat"][:, 0] ** 2 + T["target_quat"][:, 1] ** 2)], 1)
# how far the executed fingertips leave the commanded straight legs during the transit
dev = np.linalg.norm(T["tcp"][trans] - tgt_tip[trans], axis=1)
out = {"min_clearance_all_run": res, "min_clearance_during_transit_cm": res_tr, "fingertip_vs_command_in_transit_cm": {"max": round(float(dev.max()) * 100, 1), "median": round(float(np.median(dev)) * 100, 1)},
       "objects_moved_mm": json.load(open(os.path.join(a.run, "objects.json")))["centre_moved_mm"]}
np.savez(os.path.join(a.run, "avoid_check.npz"), t=t, tcp=T["tcp"], hand=T["hand"], **{"clear_" + n: c for n, c in clear.items()})
json.dump(out, open(os.path.join(a.run, "avoid_check.json"), "w"), indent=1); print(json.dumps(out, indent=1))
INK, INK2, MUTED, GRID, BG = "#1d232b", "#4a525c", "#8a919a", "#e4e1da", "#fbfaf7"
plt.rcParams.update({"figure.facecolor": BG, "axes.facecolor": BG, "axes.edgecolor": MUTED, "axes.labelcolor": INK2, "xtick.color": MUTED, "ytick.color": MUTED,
                     "font.size": 9, "axes.titlesize": 10, "axes.titleweight": "bold", "axes.titlecolor": INK, "axes.grid": True, "grid.color": GRID})
nice = {"veja": "blue bottle", "SM_Mug_B1": "mug", "beaker_500ml": "beaker", "small_KLT_visual_collision": "bin", "SM_BottlePlasticC_01": "small bottle 1", "SM_BottlePlasticC_02": "small bottle 2"}
col = {"veja": "#2f5fb3", "SM_Mug_B1": "#8e5ea2", "beaker_500ml": "#8a919a", "small_KLT_visual_collision": "#c98a1b", "SM_BottlePlasticC_01": "#3b9fb6", "SM_BottlePlasticC_02": "#3b9fb6"}
fig = plt.figure(figsize=(12, 8.4)); gs = fig.add_gridspec(2, 2, height_ratios=[1.25, 1])
fig.suptitle("Drill grasp with obstacles in the way: the arm detours on the nvblox map, then grasps and lifts", x=0.01, ha="left", fontsize=12.5, fontweight="bold", color=INK)
for k, (ix, iy, lab) in enumerate(((0, 1, "top view (x forward, y left; base frame, cm)"), (0, 2, "side view (x forward, z up)"))):
    ax = fig.add_subplot(gs[0, k]); sel = slice(None)
    for n, P in S.items():
        if n in ("small_KLT_visual_collision",) and iy == 2: pass
        ax.scatter(P[::40, ix] * 100 if iy == 1 else P[::40, 0] * 100, (P[::40, 1] if iy == 1 else P[::40, 2]) * 100, s=0.6, color=col.get(n, MUTED), alpha=0.5)
    c = (T["target"][:, [0, iy]] * 100); h = (T["hand"][:, [0, iy]] * 100); tp = (T["tcp"][:, [0, iy]] * 100)
    ax.plot(*tgt_tip[:, [0, iy]].T * 100, color="#c0392b", lw=1.2, ls="--", label="commanded fingertip (straight legs)")
    ax.plot(*tp.T, color=INK, lw=2, label="executed fingertip")
    ax.plot(*h.T, color="#2a9d5c", lw=1.2, label="hand flange")
    ax.set_aspect("equal"); ax.set_title(lab, loc="left"); ax.set_xlabel("x, cm"); ax.set_ylabel("y, cm" if iy == 1 else "z, cm")
    if k == 0: ax.legend(frameon=False, fontsize=8, loc="lower left")
ax = fig.add_subplot(gs[1, :])
for n, c in clear.items(): ax.plot(t, c * 100, color=col.get(n, MUTED), label=nice.get(n, n), lw=1.6 if n == "veja" else 1)
ax.axhline(0, color="#c0392b", lw=0.8); ax.set_ylabel("closest arm sphere surface to the object, cm"); ax.set_xlabel("sim time since the first goal, s")
ax.set_ylim(-1, 40); ax.legend(frameon=False, ncol=3, fontsize=8)
for x, lab in ((4.0, "waypoint reached"), (8.0, "pregrasp"), (11.0, "closing"), (12.8, "lift")): ax.axvline(x, color=MUTED, lw=0.6); ax.text(x + 0.05, 38, lab, fontsize=7.5, color=INK2, va="top")
fig.tight_layout(rect=(0, 0, 1, 0.95)); fig.savefig(os.path.join(a.run, "avoid_check.png"), dpi=105); print(os.path.join(a.run, "avoid_check.png"))
