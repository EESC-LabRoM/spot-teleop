"""Grasp selection: GraspGen-X candidates (grasp_generate.py) -> collision and reach filters -> one grasp + pregrasp.
Runs in the cuRobo venv (host GPU):
   ../curobo-isaac6/.venv/bin/python -I grasp_filter.py runs/<run> --object drill [--pregrasp 0.12] [--top 100]
Everything comes from perception: the target's segmentation cloud and the other objects' clouds (clouds/*.npz). Frame: fr3_link0.
Filters, in order (the funnel is printed and saved):
  0 frame       : G (GraspGen-X) -> hand flange frame, both 180-deg-symmetric versions of each grasp
  1 palm        : the hand body (box) must not contain target points, at the grasp or anywhere on the straight approach
  2 fingers     : the OPEN fingers (inner faces 8 cm apart) must not contain target points on the way in
  3 contact     : points of the target must lie between the fingers at the grasp (something to grasp)
  4 scene       : the hand boxes must not touch the table (z < 0) or a neighbour's cloud, at the grasp or on the approach
  5 reach       : cuRobo IK of the grasp pose (self collision + table + neighbours' boxes, the target excluded, 4 mm sphere buffer)
  6 pregrasp    : IK of the pregrasp pose, `--pregrasp` m back along the approach axis, with the target's box in the world too
Pick: highest score = confidence + 0.15 * (how much the approach points down) among the survivors.
Writes <run>/grasps/<object>_selected.json (poses as [x y z qx qy qz qw] of the hand flange, finger width, target box, funnel) and
<object>_status.npz (a status code per candidate, for plotting).
"""
import argparse, json, os, time

import numpy as np
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
ap = argparse.ArgumentParser()
ap.add_argument("run"); ap.add_argument("--object", required=True)
ap.add_argument("--pregrasp", type=float, default=0.12, help="pregrasp distance back along the approach axis (m)")
ap.add_argument("--top", type=int, default=100, help="candidates considered (by confidence)")
ap.add_argument("--robot", default=os.path.join(HERE, "node_config", "fr3.yml"))
ap.add_argument("--no-ik-cuboids", action="store_true", help="neighbours only in the point-box test (step 4), not as AABB cuboids in the IK world (a hinge/panel AABB swallows the target)")
ap.add_argument("--margin", type=float, default=0.006, help="inflation of the hand boxes (m)")
a = ap.parse_args()

G = np.load(os.path.join(a.run, "grasps", f"{a.object}.npz")); gg, gconf = G["grasps"][:a.top].astype(np.float64), G["conf"][:a.top]
target = G["pc"].astype(np.float64)
# GraspGen-X's franka_panda grasp frame is its URDF's 'world': the hand is rotated +90 deg about z from it (gripper.urdf world_joint rpy
# 0 0 1.5708), so the fingers slide along the grasp frame's x. The hand flange frame (fr3_hand: fingers along y) is G @ Rz(90). The hand is
# symmetric: the same grasp with the hand turned 180 deg about its approach axis is as good, so both are candidates and IK picks by reach.
def Rz(deg):
    c, s = np.cos(np.radians(deg)), np.sin(np.radians(deg)); T = np.eye(4); T[:2, :2] = [[c, -s], [s, c]]; return T
grasps = np.concatenate([np.stack([g @ Rz(90) for g in gg]), np.stack([g @ Rz(270) for g in gg])]); conf = np.concatenate([gconf, gconf])
src = np.concatenate([np.arange(len(gg))] * 2); flip = np.concatenate([np.zeros(len(gg), int), np.ones(len(gg), int)]); N = len(grasps)
names = [n[:-4] for n in os.listdir(os.path.join(a.run, "clouds")) if n.endswith(".npz")]
def vox(P, v=0.005):
    _, i = np.unique(np.floor(P / v).astype(np.int64), axis=0, return_index=True); return P[i]
others = {n: vox(np.load(os.path.join(a.run, "clouds", f"{n}.npz"))["pc"].astype(np.float64)) for n in names if n != a.object}
others = {n: P for n, P in others.items() if len(P) > 20}

# Franka Hand in the grasp (hand flange) frame: approach +z, fingers slide along y, fingertips z 0.1034, open inner faces at |y| 0.04
m = a.margin
BOX = {"palm": ((0.0, 0.0, 0.0292), (0.035 + m, 0.100 + m, 0.0292 + m)),
       "finger_l": ((0.0, 0.05, 0.0809), (0.0105 + m, 0.010 + m, 0.0225 + m)),
       "finger_r": ((0.0, -0.05, 0.0809), (0.0105 + m, 0.010 + m, 0.0225 + m))}
GAP = ((0.0, 0.0, 0.0809), (0.0105, 0.039, 0.0225))                       # between the open fingers
def inside(P, R, t, box, back=0.0):
    """Boolean per point: inside `box` of the hand placed at (R, t), moved `back` m back along its approach axis."""
    c, h = np.array(box[0]), np.array(box[1]); pl = (P - (t - back * R[:, 2])) @ R
    return np.all(np.abs(pl - c) <= h, axis=1)
offsets = np.linspace(a.pregrasp, 0.0, 7)
status = np.zeros(N, int)                                                 # 0 = survives; else the first filter that rejected it
CODE = {0: "selected pool", 1: "palm hits target", 2: "fingers hit target", 3: "nothing between fingers", 4: "hand hits table or neighbour",
        5: "no IK at the grasp", 6: "no IK at the pregrasp"}
width = np.zeros(N)
for i in range(N):
    R, t = grasps[i][:3, :3], grasps[i][:3, 3]
    if any(inside(target, R, t, BOX["palm"], s).any() for s in offsets): status[i] = 1; continue
    if any(inside(target, R, t, BOX[f], s).any() for f in ("finger_l", "finger_r") for s in offsets): status[i] = 2; continue
    g = inside(target, R, t, GAP)
    if g.sum() < 10: status[i] = 3; continue
    pl = (target[g] - t) @ R; width[i] = float(pl[:, 1].max() - pl[:, 1].min())
    bad = False
    for s in offsets:                                                      # table: the fingertips and the palm stay above z = 0
        for f in ("palm", "finger_l", "finger_r"):
            c, h = np.array(BOX[f][0]), np.array(BOX[f][1])
            corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * h + c
            if ((corners @ R.T + (t - s * R[:, 2]))[:, 2] < 0.0).any(): bad = True
        if not bad:
            bad = any(inside(P, R, t, BOX[f], s).any() for P in others.values() for f in BOX)
        if bad: break
    if bad: status[i] = 4
funnel = {"candidates": int(N), "GraspGen-X grasps": int(len(gg))}
for k in (1, 2, 3, 4): funnel[CODE[k]] = int((status == k).sum())
pool = np.nonzero(status == 0)[0]; funnel["pass geometry"] = int(len(pool)); print(f"[filter] {a.object}: {funnel}", flush=True)

def quat_wxyz(R):
    from scipy.spatial.transform import Rotation
    x, y, z, w = Rotation.from_matrix(R).as_quat(); return np.array([w, x, y, z])
def cuboid(name, P, pad):
    lo, hi = P.min(0) - pad, P.max(0) + pad; c = (lo + hi) / 2; d = hi - lo
    return {name: {"dims": d.tolist(), "pose": [*c.tolist(), 1, 0, 0, 0]}}

if len(pool):
    from curobo.geom.types import WorldConfig
    from curobo.types.base import TensorDeviceType
    from curobo.types.math import Pose
    from curobo.types.robot import RobotConfig
    from curobo.util_file import load_yaml
    from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig
    ta = TensorDeviceType()
    cub = {"table": {"dims": [3.0, 3.0, 0.2], "pose": [0.0, 0.0, -0.1, 1, 0, 0, 0]}}
    for n, P in ({} if a.no_ik_cuboids else others).items(): cub.update(cuboid(n, P, 0.01))
    world_nt = WorldConfig.from_dict({"cuboid": cub})
    cub_t = dict(cub); cub_t.update(cuboid("target", target, 0.0)); world_t = WorldConfig.from_dict({"cuboid": cub_t})
    robot_cfg = RobotConfig.from_dict(load_yaml(a.robot)["robot_cfg"])
    ik = IKSolver(IKSolverConfig.load_from_robot_config(robot_cfg, world_nt, rotation_threshold=0.02, position_threshold=0.002,
                                                       num_seeds=48, self_collision_check=True, self_collision_opt=True, tensor_args=ta,
                                                       use_cuda_graph=False))
    def solve(idx, back):
        pos = np.stack([grasps[i][:3, 3] - back * grasps[i][:3, 2] for i in idx]); q = np.stack([quat_wxyz(grasps[i][:3, :3]) for i in idx])
        t0 = time.time(); r = ik.solve_batch(Pose(ta.to_device(pos), ta.to_device(q))); torch.cuda.synchronize()
        return r.success.cpu().numpy().reshape(-1), r.solution.cpu().numpy().reshape(len(idx), -1), time.time() - t0
    ok_g, q_g, t_g = solve(pool, 0.0)
    status[pool[~ok_g]] = 5; pool = pool[ok_g]; q_g = q_g[ok_g]; funnel[CODE[5]] = int((~ok_g).sum())
    t_p = 0.0
    if len(pool):
        ik.update_world(world_t); ok_p, q_p, t_p = solve(pool, a.pregrasp)
        status[pool[~ok_p]] = 6; pool = pool[ok_p]; q_g = q_g[ok_p]; q_p = q_p[ok_p]; funnel[CODE[6]] = int((~ok_p).sum())
    funnel["IK solve s (grasp, pregrasp)"] = [round(t_g, 2), round(t_p, 2)]
funnel["valid"] = int(len(pool)); print(f"[filter] funnel: {funnel}", flush=True)
np.savez_compressed(os.path.join(a.run, "grasps", f"{a.object}_status.npz"), status=status, conf=conf, width=width, grasps=grasps, src=src, flip=flip)
res = {"object": a.object, "funnel": funnel, "codes": CODE, "pregrasp_m": a.pregrasp}
if len(pool):
    down = -grasps[pool][:, 2, 2]                                          # hand approach axis (+z of the hand) pointing down
    score = conf[pool] + 0.15 * np.clip(down, 0, 1); k = int(np.argmax(score)); i = int(pool[k])
    R, t = grasps[i][:3, :3], grasps[i][:3, 3]
    def pose7(p, R_): w, x, y, z = quat_wxyz(R_); return [float(p[0]), float(p[1]), float(p[2]), float(x), float(y), float(z), float(w)]
    lo, hi = target.min(0), target.max(0)
    res.update(index=int(src[i]), flipped=bool(flip[i]), confidence=float(conf[i]), score=float(score[k]), approach_down=float(down[k]), finger_gap_m=float(width[i]),
               grasp=pose7(t, R), pregrasp=pose7(t - a.pregrasp * R[:, 2], R), q_grasp=q_g[k].tolist(), q_pregrasp=q_p[k].tolist(),
               tcp_grasp=(t + 0.1034 * R[:, 2]).tolist(), target_center=((lo + hi) / 2).tolist(), target_half=((hi - lo) / 2).tolist(),
               valid_indices=[int(j) for j in pool])
    print(f"[filter] selected GraspGen-X #{src[i]}{" (hand turned 180 deg)" if flip[i] else ""}: conf {conf[i]:.3f}, approach down {down[k]:.2f}, gap {width[i] * 1e3:.0f} mm, fingertip point "
          f"{np.round(res['tcp_grasp'], 3).tolist()}", flush=True)
else:
    print("[filter] NO valid grasp", flush=True)
json.dump(res, open(os.path.join(a.run, "grasps", f"{a.object}_selected.json"), "w"), indent=1)
