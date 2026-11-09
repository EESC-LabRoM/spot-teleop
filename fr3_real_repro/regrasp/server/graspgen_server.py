#!/usr/bin/env python3
"""Persistent GraspGen server (runs on the LAB box, GPU1).

STANDALONE: scp this one file to the lab and run it there. It must never
import anything from the regrasp package (the lab box does not have it);
torch/graspgenx import lazily inside load_model so --help and the inbox
logic stay importable anywhere.

Protocol (mirrors regrasp/graspgen.py; keep the names identical):
  inbox/cloud_<id>.npy + inbox/cloud_<id>.json {num} (client writes .tmp,
  then renames; *.tmp is ignored) ->
  outbox/grasps_<id>.npy, rows of [16-elem 4x4 pose, confidence] float32.
  <rdir>/alive.json heartbeat {wall} every few seconds.

Start (lab, AFTER the MPC p99 measurement has Murilo's go):
  CUDA_VISIBLE_DEVICES=1 /home/murilo/graspgenx/.venv/bin/python \\
      graspgen_server.py --dir /tmp/regrasp_graspgen
"""
import argparse
import glob
import json
import os
import sys
import time

HEARTBEAT = 'alive.json'


def process_inbox(inbox, outbox, sample_fn, log=None):
    """One sweep: every job with input present and output missing gets
    sampled. sample_fn(cloud_path, num) -> (M,17) float32 array. Returns the
    number of jobs completed. Unknown files are left alone."""
    done = 0
    for cp in sorted(glob.glob(os.path.join(inbox, 'cloud_*.npy'))):
        if cp.endswith('.tmp.npy') or '.tmp' in os.path.basename(cp):
            continue
        jid = os.path.basename(cp)[len('cloud_'):-len('.npy')]
        if not jid or '/' in jid or jid.startswith('.'):
            continue
        out = os.path.join(outbox, f'grasps_{jid}.npy')
        if os.path.exists(out):
            continue
        meta_p = os.path.join(inbox, f'cloud_{jid}.json')
        try:
            num = int(json.load(open(meta_p)).get('num', 100))
        except (OSError, ValueError, KeyError):
            num = 100
        try:
            G = sample_fn(cp, num)
        except Exception as e:
            if log:
                log(f'job {jid}: sampler failed: {e}')
            continue
        # NOTE: np.save appends .npy unless present, so the tmp name must
        # keep the suffix; the client polls the exact final name only.
        tmp = out[:-len('.npy')] + '.tmp.npy'
        try:
            import numpy as np
            np.save(tmp, np.asarray(G, np.float32))
            os.rename(tmp, out)
            done += 1
            if log:
                log(f'job {jid}: {np.asarray(G).shape} done')
        except OSError as e:
            if log:
                log(f'job {jid}: write failed: {e}')
    return done


def load_model(cuda='1', assets='/home/murilo/graspgenx/assets'):
    """Import the heavy stack and return sample_fn. Called once at boot."""
    import numpy as np
    import torch
    from pathlib import Path
    from graspgenx import get_checkpoints_version_dir
    from graspgenx.utils.checkpoint_io import load_model_cfg
    from graspgenx.grasp_server import GraspGenXSampler

    ck = Path(get_checkpoints_version_dir())
    cfg = load_model_cfg(str(ck / 'gen'), str(ck / 'dis'))
    sampler = GraspGenXSampler(cfg, gripper_name='franka_panda',
                               assets_dir=assets)

    def sample(cloud_path, num):
        pcd = torch.from_numpy(np.load(cloud_path).astype(np.float32)).cuda()
        assert pcd.ndim == 2 and pcd.shape[1] == 3 and len(pcd) >= 50
        grasps, conf, _ = sampler.sample(pcd, num_grasps=num)
        G = grasps.cpu().numpy().reshape(-1, 16)
        C = conf.cpu().numpy().reshape(-1, 1)
        return np.concatenate([G, C], axis=1).astype(np.float32)

    return sample


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dir', default='/tmp/regrasp_graspgen')
    ap.add_argument('--poll', type=float, default=1.0)
    a = ap.parse_args()
    inbox = os.path.join(a.dir, 'inbox')
    outbox = os.path.join(a.dir, 'outbox')
    os.makedirs(inbox, exist_ok=True)
    os.makedirs(outbox, exist_ok=True)

    def log(m):
        print(f'{time.strftime("%H:%M:%S")} {m}', flush=True)

    log('loading model...')
    sample = load_model()
    log('ready')
    while True:
        try:
            process_inbox(inbox, outbox, sample, log)
            json.dump({'wall': time.time()},
                      open(os.path.join(a.dir, HEARTBEAT), 'w'))
        except Exception as e:
            log(f'sweep failed (retrying): {e}')
        time.sleep(a.poll)


if __name__ == '__main__':
    main()
