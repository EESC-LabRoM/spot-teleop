"""Timeline of the MPC's OWN voxel grid during an episode (node_dump/curobo_mpc_node.py with ESDF_DUMP=1: runs/<run>/epNN/esdf_dump/e_*.npz + sphere_dists.jsonl).
Per dump (sim time t, joint state q at that time, ESDF crop of the valve region in the base frame):
  hub_panel : occupied voxels within TOL of the TRUE hub body / honeycomb panel  (are the real obstacles in the MPC's map?)
  lever     : occupied voxels within TOL of the true lever at its logged angle    (the target: it should be masked out)
  robot_core: occupied, inside a robot collision sphere, not explained by any scene surface = the robot leaked into the map
  robot_halo: occupied, 0-5 cm outside the spheres, not explained
  other     : occupied, unexplained, away from the robot
  true_clear: exact distance of the nearest robot sphere surface to the true hub/panel (geometry, no map)
and from sphere_dists.jsonl the nearest-obstacle distance the MPC's collision checker returned per sphere (-100 = outside the grid, spheres 59-62 are padding).
   ../curobo-isaac6/.venv/bin/python -I approach_check.py runs/<run> --ep 0 [--png out.png] [--json out.json]"""
import argparse, glob, json, os, sys
import numpy as np, torch, trimesh
from scipy.spatial import cKDTree
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); import valve_geometry as vg
from curobo.types.base import TensorDeviceType
from curobo.types.robot import RobotConfig
from curobo.util_file import load_yaml
from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel
ap = argparse.ArgumentParser(); ap.add_argument("run"); ap.add_argument("--ep", type=int, nargs="+", default=[0]); ap.add_argument("--tol", type=float, default=0.04)
ap.add_argument("--png", default=""); ap.add_argument("--json", default=""); a = ap.parse_args()
ta = TensorDeviceType(); rc = RobotConfig.from_dict(load_yaml(os.path.join(HERE, "node_config", "fr3.yml"))["robot_cfg"]); kin = CudaRobotModel(rc.kinematics)
root = np.array(json.load(open(os.path.join(HERE, "runs", "workcell", "scene.json")))["franka_base"])
def surf(V, F, n): return trimesh.sample.sample_surface(trimesh.Trimesh(vertices=V, faces=F, process=False), n, seed=0)[0]
Vb, Fb = vg.link_mesh("Mesh.obj", vg.BODY_VIS); Vp, Fp = vg.panel_mesh(); Vl, Fl = vg.link_mesh("Mesh_1.obj", vg.LEVER_VIS)
HUB = np.vstack([surf(vg.to_base(Vb), Fb, 40000), surf(vg.to_base(Vp), Fp, 80000)]); LEV0 = surf(vg.to_base(Vl), Fl, 20000); hub_tree = cKDTree(HUB)
POST = np.array([[0.65 + dx, 0.06 + dy, z] for dx in np.linspace(-0.02, 0.02, 5) for dy in np.linspace(-0.02, 0.02, 5) for z in np.arange(0.0, 0.4165, 0.01)])   # hiveboard_post (hiveboard_scene.py)
OUT = {}
for ep in a.ep:
    d = os.path.join(a.run, f"ep{ep:02d}"); T = np.load(os.path.join(d, "trace.npz")); t0 = float(T["t"][0]); lever_t = (T["t"], T["lever_q"].reshape(len(T["t"]), -1)[:, 0])
    O = np.load(os.path.join(a.run, "objects_start.npz")); others = np.vstack([O[k].astype(np.float64).reshape(-1, 3) - root for k in O.files if k != "lever"] + [O[k].astype(np.float64).mean(1) - root for k in O.files if k != "lever"])
    rows = []
    for f in sorted(glob.glob(os.path.join(d, "esdf_dump", "e_*.npz"))):
        Z = np.load(f); E, org, vs = Z["esdf"], Z["origin"], float(Z["voxel"]); q = Z["q"][:7]
        if len(q) < 7: continue
        names = list(Z["names"]); q = np.array([Z["q"][names.index(f"fr3_joint{j}")] for j in range(1, 8)])
        S = kin.get_state(torch.tensor(q[None].astype(np.float32), device=ta.device)).link_spheres_tensor.cpu().numpy()[0]; act = S[:, 3] > 0; C, Rr = S[act, :3].astype(np.float64), S[act, 3].astype(np.float64)
        ts = float(Z["t_sim"]); lq = float(np.interp(ts - 0.0, lever_t[0] - t0 + t0, lever_t[1]))
        c, Rm = vg.rot_about_hinge(lq); lev = (LEV0 - c) @ Rm.T + c
        idx = np.argwhere((E > -999.0) & (E <= 0.0)); P = org + (idx + 0.5) * vs
        dh = hub_tree.query(P)[0] if len(P) else np.zeros(0); dl = cKDTree(lev).query(P)[0] if len(P) else np.zeros(0)
        do = cKDTree(np.vstack([others, POST])).query(P)[0] if len(P) else np.zeros(0)
        ds = (np.linalg.norm(P[:, None, :] - C[None], axis=2) - Rr[None]).min(1) if len(P) else np.zeros(0)
        hubm, levm, othm = dh <= a.tol, dl <= a.tol, do <= a.tol; expl = hubm | levm | othm
        core = (~expl) & (ds < 0); halo = (~expl) & (ds >= 0) & (ds < 0.05)
        true_clear = float((hub_tree.query(C)[0] - Rr).min())
        rows.append({"t": ts, "occupied": int(len(P)), "hub_panel": int(hubm.sum()), "lever": int((levm & ~hubm).sum()), "static_other": int((othm & ~hubm & ~levm).sum()),
                     "robot_core": int(core.sum()), "robot_halo": int(halo.sum()), "other": int(((~expl) & ~core & ~halo).sum()), "true_clear_mm": round(true_clear * 1e3, 1), "lever_deg": round(np.degrees(lq), 1)})
    sd = [json.loads(l) for l in open(os.path.join(d, "sphere_dists.jsonl"))]; ST = np.array([r["t"] for r in sd]); SD = np.array([r["d"] for r in sd])[:, :59]; SD = np.where(SD < -50, np.nan, SD)
    OUT[f"ep{ep:02d}"] = {"dumps": rows, "sphere_t": ST.tolist(), "sphere_closest_m": np.nanmax(SD, 1).tolist() if SD.size else []}
    print(f"ep{ep:02d}: {len(rows)} grids; hub_panel voxels in the MPC grid first/last {rows[0]['hub_panel']}/{rows[-1]['hub_panel']}, max robot_core {max(r['robot_core'] for r in rows)}, max robot_halo {max(r['robot_halo'] for r in rows)}, "
          f"min true sphere->hub/panel clearance {min(r['true_clear_mm'] for r in rows)} mm; MPC's closest sphere-to-map distance over the episode {np.nanmax(SD):+.3f} (more negative = further; the collision cost starts near -0.025)", flush=True)
if a.json: json.dump(OUT, open(a.json, "w"), indent=1)
if a.png:
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    fig, ax = plt.subplots(3, len(a.ep), figsize=(6 * len(a.ep), 8), squeeze=False)
    for j, ep in enumerate(a.ep):
        R_ = OUT[f"ep{ep:02d}"]; r = R_["dumps"]; t = np.array([x["t"] for x in r]); t = t - t[0]
        ax[0, j].plot(t, [x["hub_panel"] for x in r], "o-", label="hub + panel voxels in the MPC grid"); ax[0, j].plot(t, [x["lever"] for x in r], "s-", label="lever voxels (target)")
        ax[0, j].plot(t, [x["robot_core"] for x in r], "r^-", label="robot_core (leak)"); ax[0, j].plot(t, [x["robot_halo"] for x in r], "v-", c="orange", label="robot_halo"); ax[0, j].legend(fontsize=7); ax[0, j].set_ylabel("occupied voxels"); ax[0, j].set_title(f"ep{ep:02d}: what is in the MPC's grid")
        ax[1, j].plot(t, [x["true_clear_mm"] for x in r], "o-", c="k"); ax[1, j].axhline(29, c="r", ls=":"); ax[1, j].text(0.2, 31, "MPC repulsion starts ~29 mm", fontsize=7, c="r"); ax[1, j].set_ylabel("true sphere -> hub/panel (mm)"); ax[1, j].set_title("geometry (no map)")
        st = np.array(R_["sphere_t"]); ax[2, j].plot(st - st[0], np.array(R_["sphere_closest_m"]) * 1e3, c="g"); ax[2, j].axhline(-29, c="r", ls=":"); ax[2, j].set_ylabel("closest sphere -> map (mm, <0 = outside)"); ax[2, j].set_xlabel("node sim time (s)"); ax[2, j].set_title("what the MPC's collision checker returned")
    for x in ax.ravel(): x.grid(alpha=.3)
    fig.tight_layout(); fig.savefig(a.png, dpi=110); print("figure", a.png)
