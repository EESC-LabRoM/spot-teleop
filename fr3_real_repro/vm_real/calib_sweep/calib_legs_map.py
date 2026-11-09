#!/usr/bin/env python3
"""EasyHeC calibration poses, step 2: plan every leg (home -> p1 -> ... -> pN -> home) with cuRobo MotionGen against the SAME inflated world,
re-time the geometric path (min-jerk over the whole leg, peak joint speed --vmax), add dwell holds, validate densely, write one stream file.
  calib_legs.py <poses.json> <out_prefix> [--vmax 0.25] [--dwell 3.0]            (runs in the sim image)
Outputs <out_prefix>.csv (t,q0..q6,hold) at 50 Hz and <out_prefix>_summary.txt"""
import sys, json, argparse, time, numpy as np, torch
from curobo.types.base import TensorDeviceType
from curobo.types.state import JointState
from curobo.util_file import load_yaml
from curobo.geom.types import WorldConfig, Cuboid
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig
from curobo.cuda_robot_model.cuda_robot_model import CudaRobotModel, CudaRobotModelConfig
ap = argparse.ArgumentParser(); ap.add_argument("poses"); ap.add_argument("esdf"); ap.add_argument("cloud"); ap.add_argument("prefix"); ap.add_argument("--margin", type=float, default=0.08); ap.add_argument("--cloud-margin", type=float, default=0.06); ap.add_argument("--vmax", type=float, default=0.25); ap.add_argument("--dwell", type=float, default=3.0)
ap.add_argument("--hz", type=float, default=50.0); ap.add_argument("--slab-margin", type=float, default=0.03); ap.add_argument("--cfg", default="/workspace/franka_mpc/node_config/fr3_vcap1.2_acc10_real.yml")
a = ap.parse_args(); J = json.load(open(a.poses)); QH = np.array(J["q_home"]); P = [np.array(q) for q in J["poses"]]
tdev = TensorDeviceType(); robot_cfg = load_yaml(a.cfg)["robot_cfg"]
from scipy.ndimage import map_coordinates
from scipy.spatial import cKDTree
from curobo.geom.types import VoxelGrid as CuVoxelGrid
from curobo.geom.sdf.world import CollisionCheckerType
Zm = np.load(a.esdf); E = Zm["esdf"].astype(float); ORG = Zm["origin"]; VOX = float(Zm["voxel"]); Ef = np.where(E <= -999, 1.0, E)
dims = [float(round((n - 1) * VOX, 6)) for n in E.shape]; center = (ORG + np.array(dims) / 2).tolist()      # cuRobo grid shape = dims/voxel + 1 (same as the MPC node: grid_size_m = (shape-1)*voxel)
feat = torch.as_tensor((-np.where(E <= -999, 1000.0, E) + 0.5 * VOX).astype(np.float32)).reshape(-1, 1)      # same conversion as curobo_mpc_node._parse_esdf_response
grid = CuVoxelGrid(name="world_voxel", dims=dims, pose=[*center, 1.0, 0.0, 0.0, 0.0], voxel_size=VOX, feature_dtype=torch.float32, feature_tensor=feat)
top = 0.046 + a.slab_margin; table = Cuboid(name="table", pose=[1.2, 0.0, top - 0.1, 1, 0, 0, 0], dims=[2.0, 3.0, 0.2])   # starts at x = 0.2: the robot base itself sits on the table
world = WorldConfig(cuboid=[table], voxel=[grid]); robot_cfg["kinematics"]["collision_sphere_buffer"] += a.margin
S = np.load(a.cloud); CP = np.vstack([S[k] for k in S.files]); CP = CP[(CP[:, 2] > 0.09) & (CP[:, 0] > 0.10) & (CP[:, 0] < 1.10) & (np.abs(CP[:, 1]) < 0.80) & (CP[:, 2] < 1.0)]
CP = CP[np.unique(np.floor(CP / 0.015).astype(int), axis=0, return_index=True)[1]]; CT = cKDTree(CP)
cfg = MotionGenConfig.load_from_robot_config(robot_cfg, world, tdev, interpolation_dt=1 / a.hz, use_cuda_graph=False, num_trajopt_seeds=8, num_graph_seeds=8, collision_activation_distance=0.005, trajopt_tsteps=40, collision_checker_type=CollisionCheckerType.VOXEL, maximum_trajectory_dt=1.0, maximum_trajectory_time=200.0, minimum_trajectory_dt=0.02)
mg = MotionGen(cfg); mg.world_coll_checker.update_voxel_data(grid); t0 = time.time(); mg.warmup(enable_graph=True, warmup_js_trajopt=True); print(f"MotionGen warm ({time.time()-t0:.0f} s), dof {mg.kinematics.get_dof()}, joints {mg.kinematics.joint_names}")
dof = mg.kinematics.get_dof()
def full(q): q = np.asarray(q, float); return np.concatenate([q, np.full(dof - 7, 0.04)]) if dof > 7 else q
def js(q): return JointState.from_position(torch.tensor(full(q)[None], device=tdev.device, dtype=tdev.dtype))
def plan(q0, q1, tries=((True, 8), (False, 16))):
    for fine, nattempt in tries:
        res = mg.plan_single_js(js(q0), js(q1), MotionGenPlanConfig(max_attempts=nattempt, enable_graph=True, enable_finetune_trajopt=fine, timeout=25.0))
        if bool(res.success.item()): return res.get_interpolated_plan().position.cpu().numpy()[:, :7]
    return None
# 1) every pose must connect to HOME both ways (home is the hub); the others are dropped (dead ends of the free space around the objects)
conn = []; hub = {}
for i, q in enumerate(P):
    out, back = plan(QH, q), plan(q, QH)
    if out is not None and back is not None: conn.append(i); hub[i] = (out, back)
    print(f"  pose {i:2d}: connects to home both ways: {out is not None and back is not None}", flush=True)
print(f"{len(conn)} of {len(P)} poses connect to home"); P = [P[i] for i in conn]; hubs = [hub[i] for i in conn]
# 2) chain them: direct leg if cuRobo finds one, otherwise via home
legs = []; cur = QH; curi = None; ndirect = nvia = 0
for k, q in enumerate(P):
    d = plan(cur, q) if curi is not None else None
    if d is not None: legs.append(d); ndirect += 1
    else:
        if curi is not None: legs.append(hubs[curi][1])                  # back to home first
        legs.append(hubs[k][0]); nvia += 1
    cur, curi = q, k
legs.append(hubs[curi][1]); print(f"legs: {len(legs)} ({ndirect} direct pose-to-pose, {nvia} via home)")
# ---- re-time: min-jerk over the cumulative path length of each leg, then dwell
rows = []; t = 0.0; hz = a.hz
for i, Qp in enumerate(legs):
    ds = np.abs(np.diff(Qp, axis=0)).max(1); S = np.r_[0, np.cumsum(ds)]; Ls = S[-1]
    if Ls < 1e-4: continue
    Tl = max(2.0, 1.875 * Ls / a.vmax); n = int(Tl * hz) + 1; x = np.arange(n) / (n - 1); s = (10 * x**3 - 15 * x**4 + 6 * x**5) * Ls
    Q = np.stack([np.interp(s, S, Qp[:, j]) for j in range(7)], 1)
    for k in range(n): rows.append((t + k / hz, Q[k], 0))
    t += n / hz
    if np.abs(Qp[-1] - QH).max() > 0.02:        # dwell only at the calibration poses, not on the way through home
        for k in range(int(a.dwell * hz)): rows.append((t + k / hz, Q[-1], 1))
        t += a.dwell
T = np.array([r[0] for r in rows]); Q = np.array([r[1] for r in rows]); H = np.array([r[2] for r in rows])
V = np.gradient(Q, 1 / hz, axis=0); A = np.gradient(V, 1 / hz, axis=0)
LO = np.array([-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159]); HI = np.array([2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159])
# ---- dense validation against the inflated boxes + table (independent of cuRobo's own check)
kin = load_yaml(a.cfg)["robot_cfg"]["kinematics"]; kin["collision_sphere_buffer"] = 0.0; model = CudaRobotModel(CudaRobotModelConfig.from_data_dict(kin, tdev))
qq = torch.tensor(Q[::5], device=tdev.device, dtype=tdev.dtype); qq = torch.cat([qq, torch.full((len(qq), 2), 0.04, device=tdev.device, dtype=tdev.dtype)], 1)
S9 = model.get_state(qq).link_spheres_tensor.cpu().numpy(); S9 = S9[:, S9[0, :, 3] > 0]; C, r = S9[..., :3], S9[0, :, 3]; static = C.std(0).max(-1) < 0.005; C, r = C[:, ~static], r[~static]
def esdf_at(Cc):
    idx = (Cc - ORG) / VOX - 0.5; d = map_coordinates(Ef, idx.reshape(-1, 3).T, order=1, mode="nearest").reshape(Cc.shape[:-1])
    out = ~np.all((idx >= 0) & (idx <= np.array(E.shape) - 1), axis=-1); return np.where(out, 1.0, d) - 0.5 * VOX
gmap = (esdf_at(C) - r).min(1)                       # sphere surface (buffer 0) to the map surface
dc, _ = CT.query(C.reshape(-1, 3), distance_upper_bound=0.5); gcloud = (dc.reshape(C.shape[:2]) - r).min(1)   # ... to the raw robot-free depth points
g = np.minimum(gmap - a.margin, gcloud - a.cloud_margin)
gt = (C[..., 2] - r).min(1) - 0.046
txt = [f"legs {len(legs)}, poses {len(P)} ({ndirect} direct, {nvia} via home), duration {T[-1]:.0f} s ({T[-1]/60:.1f} min), holds {int(H.sum()/ (a.dwell*hz))} x {a.dwell:.1f} s",
       f"peak joint speed {np.abs(V).max():.3f} rad/s (cap {a.vmax}), peak accel {np.abs(A).max():.2f} rad/s2, max step {np.abs(np.diff(Q,axis=0)).max():.4f} rad",
       f"min distance of any sphere surface to the INFLATED boxes {100*g.min():.1f} cm (0 = at the margin), min height of any sphere surface above the table {100*gt.min():.1f} cm",
       f"real gap (buffer 0): to the map surface min {100*gmap.min():.1f} cm, to the raw depth points min {100*gcloud.min():.1f} cm (margins used: map {a.margin*100:.0f} cm, raw {a.cloud_margin*100:.0f} cm)",
       f"joint limit margin: {np.minimum(Q-LO, HI-Q).min():.2f} rad; start/end = home within {np.abs(Q[0]-QH).max():.4f}/{np.abs(Q[-1]-QH).max():.4f} rad"]
open(a.prefix + "_summary.txt", "w").write("\n".join(txt) + "\n"); print("\n".join(txt))
with open(a.prefix + ".csv", "w") as fh:
    fh.write("t," + ",".join(f"q{i}" for i in range(7)) + ",hold\n")
    for k in range(len(T)): fh.write(f"{T[k]:.3f}," + ",".join(f"{v:.5f}" for v in Q[k]) + f",{H[k]}\n")
print("wrote", a.prefix + ".csv")
