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
ap = argparse.ArgumentParser(); ap.add_argument("poses"); ap.add_argument("prefix"); ap.add_argument("--vmax", type=float, default=0.25); ap.add_argument("--dwell", type=float, default=3.0)
ap.add_argument("--hz", type=float, default=50.0); ap.add_argument("--slab-margin", type=float, default=0.03); ap.add_argument("--cfg", default="/workspace/franka_mpc/node_config/fr3_vcap1.2_acc10_real.yml")
a = ap.parse_args(); J = json.load(open(a.poses)); QH = np.array(J["q_home"]); P = [np.array(q) for q in J["poses"]]
tdev = TensorDeviceType(); robot_cfg = load_yaml(a.cfg)["robot_cfg"]
cub = []
for k, lo, hi in J["boxes"]: lo, hi = np.array(lo), np.array(hi); cub.append(Cuboid(name=f"b_{k}", pose=[*((lo + hi) / 2).tolist(), 1, 0, 0, 0], dims=(hi - lo).tolist()))
top = 0.046 + a.slab_margin; cub.append(Cuboid(name="table", pose=[1.2, 0.0, top - 0.1, 1, 0, 0, 0], dims=[2.0, 3.0, 0.2]))   # starts at x = 0.2: the robot base itself sits on the table
world = WorldConfig(cuboid=cub)
cfg = MotionGenConfig.load_from_robot_config(robot_cfg, world, tdev, interpolation_dt=1 / a.hz, use_cuda_graph=False, num_trajopt_seeds=8, num_graph_seeds=8, collision_activation_distance=0.02, trajopt_tsteps=40, maximum_trajectory_dt=1.0, maximum_trajectory_time=200.0, minimum_trajectory_dt=0.02)
mg = MotionGen(cfg); t0 = time.time(); mg.warmup(enable_graph=True, warmup_js_trajopt=True); print(f"MotionGen warm ({time.time()-t0:.0f} s), dof {mg.kinematics.get_dof()}, joints {mg.kinematics.joint_names}")
dof = mg.kinematics.get_dof()
def full(q): q = np.asarray(q, float); return np.concatenate([q, np.full(dof - 7, 0.04)]) if dof > 7 else q
def js(q): return JointState.from_position(torch.tensor(full(q)[None], device=tdev.device, dtype=tdev.dtype))
seq = [QH] + P + [QH]; legs = []; fails = []
for i in range(len(seq) - 1):
    res = None
    for attempt in range(3):
        res = mg.plan_single_js(js(seq[i]), js(seq[i + 1]), MotionGenPlanConfig(max_attempts=8, enable_graph=True, enable_finetune_trajopt=True, timeout=20.0))
        if bool(res.success.item()): break
    if not bool(res.success.item()): fails.append(i); print(f"  leg {i}: FAILED ({res.status})"); legs.append(None); continue
    Qp = res.get_interpolated_plan().position.cpu().numpy()[:, :7]; legs.append(Qp); print(f"  leg {i:2d}: {len(Qp)} pts, max joint change {np.abs(Qp[-1]-Qp[0]).max():.2f} rad, path length {np.abs(np.diff(Qp,axis=0)).max(1).sum():.2f} rad")
if fails: print("LEGS FAILED:", fails); sys.exit(3)
# ---- re-time: min-jerk over the cumulative path length of each leg, then dwell
rows = []; t = 0.0; hz = a.hz
for i, Qp in enumerate(legs):
    ds = np.abs(np.diff(Qp, axis=0)).max(1); S = np.r_[0, np.cumsum(ds)]; Ls = S[-1]
    if Ls < 1e-4: continue
    Tl = max(2.0, 1.875 * Ls / a.vmax); n = int(Tl * hz) + 1; x = np.arange(n) / (n - 1); s = (10 * x**3 - 15 * x**4 + 6 * x**5) * Ls
    Q = np.stack([np.interp(s, S, Qp[:, j]) for j in range(7)], 1)
    for k in range(n): rows.append((t + k / hz, Q[k], 0))
    t += n / hz
    for k in range(int(a.dwell * hz)): rows.append((t + k / hz, Q[-1], 1))
    t += a.dwell
T = np.array([r[0] for r in rows]); Q = np.array([r[1] for r in rows]); H = np.array([r[2] for r in rows])
V = np.gradient(Q, 1 / hz, axis=0); A = np.gradient(V, 1 / hz, axis=0)
LO = np.array([-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159]); HI = np.array([2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159])
# ---- dense validation against the inflated boxes + table (independent of cuRobo's own check)
kin = load_yaml(a.cfg)["robot_cfg"]["kinematics"]; kin["collision_sphere_buffer"] = 0.0; model = CudaRobotModel(CudaRobotModelConfig.from_data_dict(kin, tdev))
qq = torch.tensor(Q[::5], device=tdev.device, dtype=tdev.dtype); qq = torch.cat([qq, torch.full((len(qq), 2), 0.04, device=tdev.device, dtype=tdev.dtype)], 1)
S9 = model.get_state(qq).link_spheres_tensor.cpu().numpy(); S9 = S9[:, S9[0, :, 3] > 0]; C, r = S9[..., :3], S9[0, :, 3]; static = C.std(0).max(-1) < 0.005; C, r = C[:, ~static], r[~static]
g = np.full(len(C), 9.0)
for k, lo, hi in J["boxes"]:
    lo, hi = np.array(lo), np.array(hi); d = np.maximum(np.maximum(lo - C, C - hi), 0); dist = np.linalg.norm(d, axis=-1) - r; dist = np.where(np.all((C > lo) & (C < hi), axis=-1), -0.1, dist); g = np.minimum(g, dist.min(1))
gt = (C[..., 2] - r).min(1) - 0.046
txt = [f"legs {len(legs)}, poses {len(P)}, duration {T[-1]:.0f} s ({T[-1]/60:.1f} min), holds {int(H.sum()/ (a.dwell*hz))} x {a.dwell:.1f} s",
       f"peak joint speed {np.abs(V).max():.3f} rad/s (cap {a.vmax}), peak accel {np.abs(A).max():.2f} rad/s2, max step {np.abs(np.diff(Q,axis=0)).max():.4f} rad",
       f"min distance of any sphere surface to the INFLATED boxes {100*g.min():.1f} cm (0 = at the margin), min height of any sphere surface above the table {100*gt.min():.1f} cm",
       f"joint limit margin: {np.minimum(Q-LO, HI-Q).min():.2f} rad; start/end = home within {np.abs(Q[0]-QH).max():.4f}/{np.abs(Q[-1]-QH).max():.4f} rad"]
open(a.prefix + "_summary.txt", "w").write("\n".join(txt) + "\n"); print("\n".join(txt))
with open(a.prefix + ".csv", "w") as fh:
    fh.write("t," + ",".join(f"q{i}" for i in range(7)) + ",hold\n")
    for k in range(len(T)): fh.write(f"{T[k]:.3f}," + ",".join(f"{v:.5f}" for v in Q[k]) + f",{H[k]}\n")
print("wrote", a.prefix + ".csv")
