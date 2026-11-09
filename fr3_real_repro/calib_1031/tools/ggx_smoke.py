import os, time, numpy as np, torch
from graspgenx import get_checkpoints_version_dir, get_gripper_descriptions_assets
from graspgenx.grasp_server import GraspGenXSampler
from graspgenx.samplers import run_planner_on_object
from graspgenx.utils.checkpoint_io import load_model_cfg
ckpt = str(get_checkpoints_version_dir()); cfg = load_model_cfg(os.path.join(ckpt, "gen"), os.path.join(ckpt, "dis"), None, None)
t0 = time.time(); sampler = GraspGenXSampler(cfg, "franka_panda", assets_dir=str(get_gripper_descriptions_assets().parent)); torch.cuda.synchronize()
print(f"loaded franka_panda in {time.time()-t0:.1f} s, {torch.cuda.memory_allocated()/2**20:.0f} MiB allocated", flush=True)
# synthetic cylinder r=3 cm, h=12 cm standing on the table at (0.5, 0.0) in fr3_link0 (table z ~ 0.02), seen from the front-left (partial cloud)
rng = np.random.default_rng(0); n = 6000; th = rng.uniform(0, 2*np.pi, n); z = rng.uniform(0.02, 0.14, n); r = 0.03
side = np.c_[0.5 + r*np.cos(th), 0.0 + r*np.sin(th), z]; nrm = np.c_[np.cos(th), np.sin(th), np.zeros(n)]
cap = rng.uniform(0, 1, (1500, 2)); cap = cap[np.linalg.norm(cap-0.5, axis=1) < 0.5]; rr = 0.03*np.sqrt(rng.uniform(0, 1, len(cap))); ph = rng.uniform(0, 2*np.pi, len(cap))
top = np.c_[0.5 + rr*np.cos(ph), rr*np.sin(ph), np.full(len(cap), 0.14)]; P = np.vstack([side, top]); N = np.vstack([nrm, np.tile([0, 0, 1.0], (len(top), 1))])
cam = np.array([0.15, -0.55, 0.65]); vis = ((cam - P) * N).sum(1) > 0; pc = P[vis] + rng.normal(0, 0.002, (vis.sum(), 3)); print("partial cloud points:", len(pc), "of", len(P))
torch.cuda.reset_peak_memory_stats(); t0 = time.time()
G, conf, tags, _ = run_planner_on_object(pc, sampler, planner="graspmoe", grasp_threshold=-1.0, num_grasps=200, topk_num_grasps=100); torch.cuda.synchronize(); dt = time.time() - t0
o = np.argsort(-conf); G, conf = G[o], conf[o]
print(f"grasps {len(G)}, top conf {conf[:5].round(3).tolist()}, median {np.median(conf):.3f}, {dt:.2f} s (includes first-call warm-up), peak {torch.cuda.max_memory_allocated()/2**20:.0f} MiB")
t0 = time.time(); G2, conf2, _, _ = run_planner_on_object(pc, sampler, planner="graspmoe", grasp_threshold=-1.0, num_grasps=200, topk_num_grasps=100); torch.cuda.synchronize(); print(f"second call (warm): {time.time()-t0:.2f} s")
# sanity of the best grasp: position near the cylinder, approach direction
T = G[0]; print("best grasp position (m):", np.round(T[:3, 3], 3).tolist(), "| approach axis (z of the pose):", np.round(T[:3, 2], 2).tolist(), "| distance to the cylinder axis: %.3f m" % np.hypot(T[0, 3]-0.5, T[1, 3]))
near = np.hypot(G[:, 0, 3]-0.5, G[:, 1, 3]) < 0.08; print(f"grasps within 8 cm of the object axis: {int(near.sum())}/{len(G)}; with approach pointing downwards (z-axis z<-0.5): {int((G[:,2,2] < -0.5).sum())}")
