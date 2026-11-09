"""Grasp perception step 2: GraspGen-X on the object's segmentation cloud (clouds/<object>.npz from franka_ros_arm.py
--capture-cams) -> 6-DoF grasp candidates for the Franka Hand (GraspGen-X 'franka_panda' = the FR3's hand), in the robot
base frame fr3_link0. Each grasp pose G is the hand flange frame: approach +z, fingers slide along y, fingertips at z 0.1034.
Runs in GraspGen-X's own venv:
   ../graspgenx/.venv/bin/python -I grasp_generate.py runs/<run> --object drill [--num-grasps 200]
Writes runs/<run>/grasps/<object>.npz (cloud used, grasps, conf, tags) and prints the timing.
"""
import argparse, json, os, time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__)); GGX = os.path.normpath(os.path.join(HERE, "..", "graspgenx"))
os.environ.setdefault("GRASPGENX_CHECKPOINT_DIR", os.path.join(GGX, "ext", "graspgenx_checkpoints"))
os.environ.setdefault("GRASPGENX_GRIPPER_CFG_DIR", os.path.join(GGX, "ext", "gripper_descriptions"))
ap = argparse.ArgumentParser()
ap.add_argument("run"); ap.add_argument("--object", required=True)
ap.add_argument("--gripper", default="franka_panda"); ap.add_argument("--num-grasps", type=int, default=200)
ap.add_argument("--max-points", type=int, default=4000)
a = ap.parse_args()

import torch
from scipy.spatial import cKDTree
from graspgenx import get_checkpoints_version_dir, get_gripper_descriptions_assets
from graspgenx.grasp_server import GraspGenXSampler
from graspgenx.samplers import run_planner_on_object
from graspgenx.utils.checkpoint_io import load_model_cfg

Z = np.load(os.path.join(a.run, "clouds", f"{a.object}.npz")); pc = Z["pc"].astype(np.float64)
n_raw = len(pc)
# depth edges leave flying points: keep points with enough neighbours within 1 cm, and drop what sits on the table top
d, _ = cKDTree(pc).query(pc, k=9); pc = pc[(d[:, -1] < 0.01) & (pc[:, 2] > 0.004)]
rng = np.random.default_rng(0)
if len(pc) > a.max_points: pc = pc[rng.choice(len(pc), a.max_points, replace=False)]
print(f"[ggx] {a.object}: {n_raw} segmentation points -> {len(pc)} after the outlier filter", flush=True)
assert len(pc) >= 100, "too few points on the object"

ckpt = str(get_checkpoints_version_dir())
cfg = load_model_cfg(os.path.join(ckpt, "gen"), os.path.join(ckpt, "dis"), None, None)
t0 = time.time(); sampler = GraspGenXSampler(cfg, a.gripper, assets_dir=str(get_gripper_descriptions_assets().parent))
torch.cuda.synchronize(); t_load = time.time() - t0
torch.cuda.reset_peak_memory_stats(); t0 = time.time()
G, conf, tags, _ = run_planner_on_object(pc, sampler, planner="graspmoe", grasp_threshold=-1.0, num_grasps=a.num_grasps,
                                         topk_num_grasps=100)
torch.cuda.synchronize(); dt = time.time() - t0
o = np.argsort(-conf); G, conf = G[o], conf[o]; tags = [tags[i] for i in o]
os.makedirs(os.path.join(a.run, "grasps"), exist_ok=True)
np.savez_compressed(os.path.join(a.run, "grasps", f"{a.object}.npz"), pc=pc, grasps=G, conf=conf, tags=np.array(tags))
print(f"[ggx] {a.object}: {len(G)} grasps, top conf {conf[:3].round(3).tolist()}, load {t_load:.1f} s, infer {dt:.2f} s, "
      f"peak {torch.cuda.max_memory_allocated() / 2**20:.0f} MiB", flush=True)
json.dump({"object": a.object, "points": int(len(pc)), "grasps": int(len(G)), "conf_top": conf[:5].round(3).tolist(),
           "load_s": round(t_load, 1), "infer_s": round(dt, 2)}, open(os.path.join(a.run, "grasps", f"{a.object}.json"), "w"), indent=1)
