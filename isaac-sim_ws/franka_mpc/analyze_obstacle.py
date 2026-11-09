"""Checkpoint-2 results for one run_obstacle.sh run: signed distance to the real objects (Isaac's meshes, not the map) of
the commanded fingertip reference, the executed fingertips and wrist (hand flange), and the whole arm (cuRobo's collision
spheres posed by forward kinematics on the measured joints), as in the paper's obstacle figure; plus whether any object
moved, and the node's own view (its ESDF distance and timing).
   curobo-isaac6/.venv/bin/python -I analyze_obstacle.py runs/obstacle_2zed     -> RESULT.json, obstacle.png
"""
import csv, json, os, re, sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import ConvexHull, Delaunay, cKDTree

run = os.path.abspath(sys.argv[1] if len(sys.argv) > 1 else "runs/obstacle_2zed")
T = np.load(os.path.join(run, "trace.npz")); M = dict(np.load(os.path.join(run, "objects_start.npz")))
OB = json.load(open(os.path.join(run, "objects.json"))); RI = json.load(open(os.path.join(run, "RESULT_isaac.json")))
ROOT = np.array(RI["root"])                                     # Franka base (fr3_link0) in world, no rotation
TCP = 0.1034
m = re.search(r"start at sim t=([0-9.]+)", open(os.path.join(run, "goal.log")).read())
t0 = float(m.group(1)) if m else float(T["target_stamp"].min())
tp = T["t"] - t0

# ---- ground-truth signed distance to the objects: dense surface samples with face normals -----------------------------
rng = np.random.default_rng(0); pts, owner, hulls = [], [], []
for k, (name, tri) in enumerate(M.items()):
    tri = tri.astype(np.float64) - ROOT                        # world -> base frame
    e1, e2 = tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]; cr = np.cross(e1, e2); area = 0.5 * np.linalg.norm(cr, axis=1)
    keep = area > 1e-12; tri, cr, area = tri[keep], cr[keep], area[keep]
    n = int(min(400_000, max(20_000, area.sum() / (1.5e-3) ** 2)))   # ~1.5 mm spacing
    f = rng.choice(len(tri), n, p=area / area.sum()); u, v = rng.random(n), rng.random(n); s_ = u + v > 1
    u[s_], v[s_] = 1 - u[s_], 1 - v[s_]
    P = tri[f, 0] + u[:, None] * (tri[f, 1] - tri[f, 0]) + v[:, None] * (tri[f, 2] - tri[f, 0])
    V = tri.reshape(-1, 3); hulls.append(Delaunay(V[ConvexHull(V).vertices]))   # inside test: the object's convex hull
    pts.append(P); owner += [k] * n                            # (the meshes have several shells and mixed winding, so
pts, owner = np.concatenate(pts), np.array(owner); names = list(M)   # normals and ray parity are unreliable; the bottle
tree = cKDTree(pts)                                            # the reference sweeps through is close to convex)
def sdist(X):                                                  # signed distance (m), negative inside; nearest object
    d, i = tree.query(X); o = owner[i]; s = np.ones(len(X))
    for k in np.unique(o):
        m = o == k; s[m] = np.where(hulls[k].find_simplex(X[m]) >= 0, -1.0, 1.0)
    return d * s, o

def tcp_of(p, quat_xyzw):
    x, y, z, w = quat_xyzw.T
    ax = np.stack([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)], 1)   # hand z axis
    return p + TCP * ax
ref_tip = tcp_of(T["target"], T["target_quat"])
d_ref, o_ref = sdist(ref_tip); d_tip, o_tip = sdist(T["tcp"]); d_wr, o_wr = sdist(T["hand"])

# ---- whole arm: cuRobo spheres (no buffer) on the measured joints ----------------------------------------------------------
import torch
from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel, CudaRobotModelConfig
from curobo.types.base import TensorDeviceType
from curobo.util_file import get_robot_configs_path, join_path, load_yaml
cfg = load_yaml(join_path(get_robot_configs_path(), "fr3.yml"))["robot_cfg"]; cfg["kinematics"]["collision_sphere_buffer"] = 0.0   # FR3 (Panda runs: same FK and spheres)
kin = CudaRobotModel(CudaRobotModelConfig.from_data_dict(cfg["kinematics"], tensor_args=TensorDeviceType()))
q = np.concatenate([T["q"], np.full((len(T["q"]), 2), 0.04)], 1)
sph = kin.get_state(torch.tensor(q, device="cuda", dtype=torch.float32)).link_spheres_tensor.cpu().numpy()   # (n, S, 4)
link_of = np.array(sum(([ln] * len(v) for ln, v in kin.kinematics_config.link_spheres_names.items()), [])) \
    if hasattr(kin.kinematics_config, "link_spheres_names") else None
valid = sph[0, :, 3] > 0
c_ = sph[:, valid, :3].reshape(-1, 3); r_ = sph[:, valid, 3].reshape(-1)
dc, oc = sdist(c_); clear = (dc - r_).reshape(len(q), -1); arm_min = clear.min(1); arm_obj = oc.reshape(len(q), -1)[np.arange(len(q)), clear.argmin(1)]
base_spheres = sph[0, valid, 2] < 0.12                         # link0/link1 spheres: the base stands on the table, so the
arm_min_moving = clear[:, ~base_spheres].min(1)                # 'moving arm' excludes spheres that never move

rows = [r for r in csv.DictReader(open(os.path.join(run, "mpc_node.csv")))]
ctl = [r for r in rows if r["event"] == "control_step" and r["mpc_step_ms"]]
occ = [int(m_.group(1)) for r in rows if r["event"] == "esdf" for m_ in [re.match(r"ESDF: (\d+)/", r["message"])] if m_]
esdf_ms = [float(m_.group(1)) for r in rows if r["event"] == "esdf" for m_ in [re.match(r"ESDF service responded in (\d+)ms", r["message"])] if m_]
q_ = lambda x, p: round(float(np.percentile(x, p)), 2)
sw = tp >= 0
res = {"run": os.path.basename(run), "cameras": RI.get("zeds"), "real_time_factor": RI["real_time_factor"],
       "commands_per_sim_s": round(RI["commands_received"] / RI["record_sim_s"], 1), "lockstep_timeouts": RI.get("lockstep_timeouts"),
       "min_signed_distance_cm": {"commanded_fingertips": round(float(d_ref[sw].min()) * 100, 1),
                                  "executed_fingertips": round(float(d_tip[sw].min()) * 100, 1),
                                  "executed_wrist_hand": round(float(d_wr[sw].min()) * 100, 1),
                                  "arm_spheres_surface_moving_links": round(float(arm_min_moving[sw].min()) * 100, 1),
                                  "arm_spheres_surface_all": round(float(arm_min[sw].min()) * 100, 1)},
       "nearest_object_at_min": {"commanded": names[o_ref[sw][d_ref[sw].argmin()]], "executed_fingertips": names[o_tip[sw][d_tip[sw].argmin()]],
                                 "arm": names[arm_obj[sw][arm_min_moving[sw].argmin()]]},
       "object_centre_moved_mm": OB["centre_moved_mm"],
       "esdf_occupied_voxels_last": occ[-1] if occ else None, "esdf_service_ms": {"median": q_(esdf_ms, 50), "max": max(esdf_ms)} if esdf_ms else None,
       "node_mpc_step_ms_wall": {"median": q_([float(r["mpc_step_ms"]) for r in ctl], 50), "p95": q_([float(r["mpc_step_ms"]) for r in ctl], 95)},
       "node_min_dist_esdf_m_max": round(max(float(r["min_dist"]) for r in ctl if r["min_dist"]), 3),
       "final_fingertip_err_cm": round(float(np.linalg.norm(T["tcp"][-1] - ref_tip[-1])) * 100, 1)}
json.dump(res, open(os.path.join(run, "RESULT.json"), "w"), indent=1)
print(json.dumps(res, indent=1))

# ---- figure: like the paper's obstacle figure (path + signed distance over time) --------------------------------------------
SURF, INK, INK2, MUTED, GRID, AXIS = "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7"
S1, S2, S3, S4 = "#2a78d6", "#eb6834", "#1baf7a", "#6250d6"
plt.rcParams.update({"figure.facecolor": SURF, "axes.facecolor": SURF, "axes.edgecolor": AXIS, "axes.labelcolor": INK2,
                     "xtick.color": MUTED, "ytick.color": MUTED, "grid.color": GRID, "grid.linewidth": 0.6, "font.size": 10,
                     "axes.titlesize": 11, "axes.titleweight": "bold", "axes.titlecolor": INK, "axes.spines.top": False,
                     "axes.spines.right": False, "axes.grid": True, "lines.linewidth": 2.0, "legend.frameon": False,
                     "legend.labelcolor": INK2})
fig, ax = plt.subplots(1, 3, figsize=(16, 5.2), gridspec_kw={"width_ratios": [1, 1, 1.6]})
fig.suptitle("Obstacle avoidance on the nvblox map: the reference sweeps through the bottle", x=0.01, ha="left",
             fontsize=14, fontweight="bold", color=INK)
fig.text(0.01, 0.905, f"{res['run']}: cameras {res['cameras']}; distances to Isaac's object meshes (ground truth), "
         f"negative = inside; objects moved (mm): {max(OB['centre_moved_mm'].values()):.1f} max", ha="left", color=INK2)
veja = pts[owner == names.index("veja")] if "veja" in names else pts
for a_, (i, j, lab) in zip(ax[:2], ((1, 0, "x, forward (m)"), (1, 2, "z, up (m)"))):
    near = (np.abs(pts[:, 0] - 0.55) < 0.25) & (np.abs(pts[:, 1]) < 0.3) & (pts[:, 2] < 0.45)
    a_.scatter(pts[near, i][::40], pts[near, j][::40], s=1, color=AXIS, zorder=0)
    a_.plot(ref_tip[sw, i], ref_tip[sw, j], color=S1, lw=1.6, label="commanded fingertips")
    a_.plot(T["tcp"][sw, i], T["tcp"][sw, j], color=S2, lw=1.6, label="executed fingertips")
    a_.plot(T["hand"][sw, i], T["hand"][sw, j], color=S3, lw=1.6, label="executed wrist (hand flange)")
    a_.set_xlabel("y, left (m)"); a_.set_ylabel(lab); a_.invert_xaxis(); a_.set_aspect("equal")
ax[0].set_title("Seen from above (grey: objects)", loc="left"); ax[1].set_title("Seen from the front", loc="left")
ax[1].legend(loc="upper left", fontsize=8.5)
a_ = ax[2]
a_.axhline(0, color=MUTED, lw=1)
a_.plot(tp[sw], d_ref[sw] * 100, color=S1, label="commanded fingertips")
a_.plot(tp[sw], d_tip[sw] * 100, color=S2, label="executed fingertips")
a_.plot(tp[sw], d_wr[sw] * 100, color=S3, label="executed wrist (hand flange)")
a_.plot(tp[sw], arm_min_moving[sw] * 100, color=S4, lw=1.4, label="whole arm (collision-sphere surfaces)")
a_.set_title("Signed distance to the nearest object", loc="left"); a_.set_xlabel("sweep time (sim s)"); a_.set_ylabel("cm")
a_.legend(loc="upper right", fontsize=8.5)
fig.tight_layout(rect=(0, 0, 1, 0.89))
fig.savefig(os.path.join(run, "obstacle.png"), dpi=110)
print("plot:", os.path.join(run, "obstacle.png"))
