"""GraspGen-X on our sim scene, offline: an object's partial point cloud from one saved ZED depth frame (a map-only run's
depth.npz) -> 6-DoF grasps for a gripper (default the Franka Hand, GraspGen-X's 'franka_panda': the FR3's hand too), with timing and GPU memory.
The object is cut out of the ZED depth with the sim's exact meshes (points within --cut m of the true surface), standing
in for the SAM2 mask. Frame: the robot base, fr3_link0 (Z up), which GraspGen-X's top-down branch needs.
Runs in GraspGen-X's own venv:
   ../graspgenx/.venv/bin/python -I graspgenx_test.py [--run runs/map_pair_current] [--cam zed] [--objects drill veja SM_Mug_B1]
       [--gripper franka_panda] [--out runs/graspgenx_franka]
"""
import argparse, json, os, sys, time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__)); GGX = os.path.normpath(os.path.join(HERE, "..", "graspgenx"))
os.environ.setdefault("GRASPGENX_CHECKPOINT_DIR", os.path.join(GGX, "ext", "graspgenx_checkpoints"))
os.environ.setdefault("GRASPGENX_GRIPPER_CFG_DIR", os.path.join(GGX, "ext", "gripper_descriptions"))
ap = argparse.ArgumentParser()
ap.add_argument("--run", default=os.path.join(HERE, "runs", "map_pair_current"))
ap.add_argument("--cam", default="zed", help="which ZED's depth frame (zed | zed2)")
ap.add_argument("--objects", nargs="+", default=["drill", "veja", "SM_Mug_B1"])
ap.add_argument("--gripper", default="franka_panda")
ap.add_argument("--num-grasps", type=int, default=200)
ap.add_argument("--cut", type=float, default=0.015)
ap.add_argument("--out", default=os.path.join(HERE, "runs", "graspgenx_franka"))
a = ap.parse_args(); os.makedirs(a.out, exist_ok=True)

import torch
from scipy.spatial import cKDTree
from graspgenx import get_checkpoints_version_dir, get_gripper_descriptions_assets
from graspgenx.grasp_server import GraspGenXSampler
from graspgenx.samplers import run_planner_on_object
from graspgenx.utils.checkpoint_io import load_model_cfg

# ---- the object's partial cloud from one ZED depth frame ---------------------------------------------------------------
D = np.load(os.path.join(a.run, "depth.npz"), allow_pickle=True)
depth, K = D[f"{a.cam}_raw_depth"].astype(np.float64), D[f"{a.cam}_raw_K"]
Tw = np.array(json.load(open(os.path.join(a.run, f"camera_{a.cam}.json")))["T_world_camera_usd"])
root = np.array(json.load(open(os.path.join(HERE, "runs", "workcell", "scene.json")))["franka_base"])
v, u = np.nonzero(np.isfinite(depth) & (depth > 0.2) & (depth < 3.0)); z = depth[v, u]
Pc = np.stack([(u - K[0, 2]) * z / K[0, 0], -(v - K[1, 2]) * z / K[1, 1], -z], 1)        # optical -> USD camera (-Z forward)
P = (Tw[:3, :3] @ Pc.T).T + Tw[:3, 3] - root                                               # robot base (fr3_link0)
O = dict(np.load(os.path.join(a.run, "objects_start.npz"))); rng = np.random.default_rng(0)

ckpt = str(get_checkpoints_version_dir())
cfg = load_model_cfg(os.path.join(ckpt, "gen"), os.path.join(ckpt, "dis"), None, None)
t0 = time.time(); sampler = GraspGenXSampler(cfg, a.gripper, assets_dir=str(get_gripper_descriptions_assets().parent))
torch.cuda.synchronize(); t_load = time.time() - t0; mem_load = torch.cuda.memory_allocated() / 2**20
res = {"run": os.path.basename(a.run), "cam": a.cam, "gripper": a.gripper, "load_s": round(t_load, 1), "objects": {}}
print(f"[ggx] {a.gripper} loaded in {t_load:.1f} s, {mem_load:.0f} MiB allocated", flush=True)
for name in a.objects:
    tri = O[name].astype(np.float64) - root
    cr = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]); ar = 0.5 * np.linalg.norm(cr, axis=1); ok = ar > 1e-12
    f = rng.choice(np.nonzero(ok)[0], 60000, p=ar[ok] / ar[ok].sum()); w1, w2 = rng.random(60000), rng.random(60000); s = w1 + w2 > 1
    w1[s], w2[s] = 1 - w1[s], 1 - w2[s]; S = tri[f, 0] + w1[:, None] * (tri[f, 1] - tri[f, 0]) + w2[:, None] * (tri[f, 2] - tri[f, 0])
    lo, hi = S.min(0) - 0.03, S.max(0) + 0.03; box = np.all((P > lo) & (P < hi), 1)
    d, _ = cKDTree(S).query(P[box]); pc = P[box][d < a.cut]; pc = pc[pc[:, 2] > 0.004]      # off the table top
    if len(pc) > 4000: pc = pc[rng.choice(len(pc), 4000, replace=False)]
    if len(pc) < 100: print(f"[ggx] {name}: only {len(pc)} points in view, skipped"); continue
    torch.cuda.reset_peak_memory_stats(); torch.cuda.synchronize(); t0 = time.time()
    G, conf, tags, _ = run_planner_on_object(pc, sampler, planner="graspmoe", grasp_threshold=-1.0, num_grasps=a.num_grasps,
                                             topk_num_grasps=100)
    torch.cuda.synchronize(); dt = time.time() - t0; peak = torch.cuda.max_memory_allocated() / 2**20
    o = np.argsort(-conf); G, conf = G[o], conf[o]; tags = [tags[i] for i in o]
    np.savez_compressed(os.path.join(a.out, f"{name}.npz"), pc=pc, grasps=G, conf=conf, tags=np.array(tags), surface=S[::20])
    res["objects"][name] = {"points": int(len(pc)), "grasps": int(len(G)), "conf_top": [round(float(c), 3) for c in conf[:5]],
                            "conf_median": round(float(np.median(conf)), 3) if len(conf) else None,
                            "from_diffusion": int(sum(t == "diff" for t in tags)), "infer_s": round(dt, 2), "peak_alloc_mib": round(peak)}
    print(f"[ggx] {name}: {len(pc)} pts -> {len(G)} grasps, top conf {conf[:3].round(3).tolist()}, {dt:.2f} s, peak {peak:.0f} MiB", flush=True)
res["gpu_reserved_mib"] = round(torch.cuda.max_memory_reserved() / 2**20)
json.dump(res, open(os.path.join(a.out, "result.json"), "w"), indent=1); print(json.dumps(res, indent=1))
