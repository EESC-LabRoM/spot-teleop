#!/usr/bin/env python3
"""GraspGen-X inference on the lab GPU (called by grasp_loop stage 4).

Reads an (N,3) object cloud in fr3_link0, writes (M,17) float32 rows of
[16-elem row-major 4x4 pose, confidence] (same layout as claude_grasps.npy).
Runs with the graspgenx .venv on the lab PC:
  CUDA_VISIBLE_DEVICES=1 /home/murilo/graspgenx/.venv/bin/python grasp_infer.py cloud.npy grasps.npy [--num 100]
"""
import sys
import time

import numpy as np

cloud_path, out_path = sys.argv[1], sys.argv[2]
num = int(sys.argv[4]) if len(sys.argv) > 4 and sys.argv[3] == '--num' else 100

from pathlib import Path
from graspgenx import get_checkpoints_version_dir
from graspgenx.utils.checkpoint_io import load_model_cfg
from graspgenx.grasp_server import GraspGenXSampler

import torch
pcd = torch.from_numpy(np.load(cloud_path).astype(np.float32)).cuda()
assert pcd.ndim == 2 and pcd.shape[1] == 3 and len(pcd) >= 50, f'bad cloud {tuple(pcd.shape)}'
ck = Path(get_checkpoints_version_dir())
cfg = load_model_cfg(str(ck / 'gen'), str(ck / 'dis'))
sampler = GraspGenXSampler(cfg, gripper_name='franka_panda',
                           assets_dir='/home/murilo/graspgenx/assets')
t0 = time.time()
grasps, conf, _ = sampler.sample(pcd, num_grasps=num)
G = grasps.cpu().numpy().reshape(-1, 16)
C = conf.cpu().numpy().reshape(-1, 1)
out = np.concatenate([G, C], axis=1).astype(np.float32)
np.save(out_path, out)
print(f'grasps {out.shape} conf [{out[:, 16].min():.3f}, {out[:, 16].max():.3f}] in {time.time() - t0:.1f} s',
      flush=True)
