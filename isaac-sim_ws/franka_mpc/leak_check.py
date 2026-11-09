"""Does the robot leak into the nvblox map? For each episode of a valve_isaac session: the ESDF dump (map_pre = before the goal starts, arm at its start pose;
map_post = after the episode, arm at its final pose), the arm's collision spheres at that q (cuRobo FK of fr3.yml, the same spheres the MPC and the self-filter use),
and the scene's true geometry (table plane, hub/panel/lever meshes with the lever at its logged angle, the other objects' start triangles).
Every OCCUPIED voxel (-1000 < esdf <= 0) is classified:
  explained : within TOL of a true scene surface (table z = 0, hub, panel, lever, objects)
  robot_core: inside a robot sphere (d to the sphere surface < 0) and NOT explained  = the robot itself got into the map
  robot_halo: 0-5 cm outside the sphere surfaces, not explained                       = ghost around the robot (a thin self-filter margin leaves these)
  other     : not explained, not near the robot (noise / unmodelled)
Also: per sphere, the map's distance (trilinear ESDF at the centre minus radius) vs the true scene distance = what the MPC collision cost sees and whether it is real.
   ../curobo-isaac6/.venv/bin/python -I leak_check.py runs/<run> [--ep 0 1 2] [--tol 0.04] [--json out.json] [--png out.png]"""
import argparse, json, os, sys
import numpy as np, torch, trimesh
from scipy.spatial import cKDTree
from scipy.interpolate import RegularGridInterpolator
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); import valve_geometry as vg
from curobo.types.base import TensorDeviceType
from curobo.types.robot import RobotConfig
from curobo.util_file import load_yaml
from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel
ap = argparse.ArgumentParser(); ap.add_argument("run"); ap.add_argument("--ep", type=int, nargs="*"); ap.add_argument("--tol", type=float, default=0.04)
ap.add_argument("--json", default=""); ap.add_argument("--png", default=""); a = ap.parse_args()
ta = TensorDeviceType(); rc = RobotConfig.from_dict(load_yaml(os.path.join(HERE, "node_config", "fr3.yml"))["robot_cfg"]); kin = CudaRobotModel(rc.kinematics)
link_names = rc.kinematics.kinematics_config.link_name_to_idx_map; inv = {v: k for k, v in link_names.items()}; sph_link = rc.kinematics.kinematics_config.link_sphere_idx_map.cpu().numpy()
BUF = float(rc.kinematics.kinematics_config.collision_sphere_buffer) if hasattr(rc.kinematics.kinematics_config, "collision_sphere_buffer") else 0.0
root = np.array(json.load(open(os.path.join(HERE, "runs", "workcell", "scene.json")))["franka_base"])
def surf(V, F, n, seed=0):
    return trimesh.sample.sample_surface(trimesh.Trimesh(vertices=V, faces=F, process=False), n, seed=seed)[0]
Vb, Fb = vg.link_mesh("Mesh.obj", vg.BODY_VIS); Vp, Fp = vg.panel_mesh(); Vl, Fl = vg.link_mesh("Mesh_1.obj", vg.LEVER_VIS)
STATIC = np.vstack([surf(vg.to_base(Vb), Fb, 40000), surf(vg.to_base(Vp), Fp, 80000)])
LEVER0 = surf(vg.to_base(Vl), Fl, 20000)
def scene_points(run, lever_rad):
    c, Rm = vg.rot_about_hinge(lever_rad); lev = (LEVER0 - c) @ Rm.T + c
    O = np.load(os.path.join(run, "objects_start.npz")); pts = [STATIC, lev]
    for k in O.files:
        if k == "lever": continue
        T = O[k].astype(np.float64) - root; pts.append(T.reshape(-1, 3)); pts.append(T.mean(1))
    return np.vstack(pts)
def spheres(q):
    S = kin.get_state(torch.tensor(np.asarray(q, np.float32)[None], device=ta.device)).link_spheres_tensor.cpu().numpy()[0]; act = S[:, 3] > 0
    return np.where(act)[0], S[act, :3].astype(np.float64), S[act, 3].astype(np.float64)
def analyse(ep_dir, tag, q, lever_rad, run):
    f = os.path.join(ep_dir, f"map_{tag}", "esdf.npz")
    if not os.path.exists(f): return None
    Z = np.load(f); E, org, vs = Z["esdf"], Z["origin"], float(Z["voxel"])
    idx = np.argwhere((E > -999.0) & (E <= 0.0)); P = org + (idx + 0.5) * vs
    sc = scene_points(run, lever_rad); tree = cKDTree(sc); ids, C, Rr = spheres(q)
    d_scene = tree.query(P)[0]; dist_s = np.linalg.norm(P[:, None, :] - C[None], axis=2) - Rr[None]; d_rob = dist_s.min(1); near = dist_s.argmin(1)
    cls = np.full(len(P), "other", object); expl = d_scene <= a.tol
    cls[expl] = "explained"; cls[(~expl) & (d_rob < 0)] = "robot_core"; cls[(~expl) & (d_rob >= 0) & (d_rob < 0.05)] = "robot_halo"
    counts = {k: int((cls == k).sum()) for k in ("explained", "robot_core", "robot_halo", "other")}
    core_links = {}
    for j in np.where(cls == "robot_core")[0]: core_links[inv.get(int(sph_link[ids[near[j]]]), "?")] = core_links.get(inv.get(int(sph_link[ids[near[j]]]), "?"), 0) + 1
    # per sphere: map distance vs true scene distance
    gx = [org[i] + (np.arange(E.shape[i]) + 0.5) * vs for i in range(3)]; interp = RegularGridInterpolator(gx, np.where(E <= -999.0, 1.0, E), bounds_error=False, fill_value=1.0)
    dm = interp(C) - Rr; dt = tree.query(C)[0] - Rr
    worst = np.argsort(dm)[:5]
    sph = [{"sphere": int(ids[j]), "link": inv.get(int(sph_link[ids[j]]), "?"), "map_dist_mm": round(float(dm[j]) * 1e3, 1), "true_scene_dist_mm": round(float(dt[j]) * 1e3, 1)} for j in worst]
    return {"occupied_voxels": int(len(P)), "classes": counts, "robot_core_by_link": core_links, "closest_spheres_in_map": sph, "voxel": vs,
            "_P": P, "_cls": cls, "_C": C, "_R": Rr, "_d_scene": d_scene}
res, figs = {}, []
eps = a.ep if a.ep else sorted(int(d[2:]) for d in os.listdir(a.run) if d.startswith("ep") and d[2:].isdigit())
for ep in eps:
    d = os.path.join(a.run, f"ep{ep:02d}"); tf = os.path.join(d, "trace.npz")
    if os.path.exists(tf):
        T = np.load(tf); q0, q1 = T["q"][0], T["q"][-1]; lq = T["lever_q"].reshape(len(T["q"]), -1)[:, 0] if "lever_q" in T.files else np.zeros(len(T["q"]))
        todo = (("pre", q0, float(lq[0])), ("post", q1, float(lq[-1])))
    else:                                                    # episode still running: map_pre only, the arm is at the sim's retract pose, lever closed
        todo = (("pre", np.array([0.0, -1.3, 0.0, -2.5, 0.0, 1.0, 0.0]), 0.0),)
    for tag, q, lev in todo:
        r = analyse(d, tag, q, lev, a.run)
        if r is None: continue
        figs.append((ep, tag, r)); res[f"ep{ep:02d}/{tag}"] = {k: v for k, v in r.items() if not k.startswith("_")}
        print(f"ep{ep:02d} map_{tag}: occupied {r['occupied_voxels']} -> {r['classes']} | robot_core by link {r['robot_core_by_link']} | lever {np.degrees(lev):+.0f} deg", flush=True)
        for s in r["closest_spheres_in_map"][:3]: print("     sphere", s, flush=True)
if a.json: json.dump(res, open(a.json, "w"), indent=1)
if a.png and figs:
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    col = {"explained": "#9aa0a6", "robot_core": "#d62728", "robot_halo": "#ff7f0e", "other": "#1f77b4"}
    fig, ax = plt.subplots(len(figs), 2, figsize=(11, 3.6 * len(figs)), squeeze=False)
    for i, (ep, tag, r) in enumerate(figs):
        P, cls, C, R = r["_P"], r["_cls"], r["_C"], r["_R"]
        for j, (u, v, nm) in enumerate(((0, 1, "top view: x (forward) / y"), (0, 2, "side view: x (forward) / z (up)"))):
            for k in ("explained", "other", "robot_halo", "robot_core"):
                m = cls == k; ax[i, j].scatter(P[m, u], P[m, v], s=9, c=col[k], label=f"{k} ({int(m.sum())})")
            for c_, r_ in zip(C, R): ax[i, j].add_patch(plt.Circle((c_[u], c_[v]), r_, fill=False, ec="k", lw=0.5, alpha=0.5))
            ax[i, j].set_aspect("equal"); ax[i, j].set_title(f"ep{ep:02d} map_{tag}: {nm}", fontsize=9); ax[i, j].grid(alpha=.3)
        ax[i, 0].legend(fontsize=7, loc="upper left")
    fig.tight_layout(); fig.savefig(a.png, dpi=110); print("figure", a.png)
