"""Build the merged dual-camera drill cloud for a fresh grasp (offline-safe).

Replicates the 20261105_011430 recipe (see that run's cloud.json source):
stacked live viewer snapshots (ZED1 SAM2 mask + ZED2 SAM2-mask-gated points),
2.5 mm dedupe, table cut, 12 mm outlier removal. No ROS, no motion.
Run in vm-viewer (has numpy): python3 build_dual_cloud.py <run_dir>
"""
import json
import os
import sys

import numpy as np

R = sys.argv[1]
snaps = sorted(f for f in os.listdir(R) if f.startswith('snap_') and f.endswith('.json'))
assert len(snaps) >= 1, 'no snapshots'
P, meta = [], []
for f in snaps:
    d = json.load(open(os.path.join(R, f)))
    pts = np.asarray(d['cloud'], float)
    assert pts.ndim == 2 and pts.shape[1] == 3 and len(pts) >= 50, f'bad cloud in {f}'
    P.append(pts)
    live = d.get('live', {})
    meta.append({'snap': f, 'run': d.get('run'), 'n': int(len(pts)),
                 'n_zed2': int(live.get('n_zed2', -1)) if isinstance(live, dict) else -1,
                 'zed2_src': (live.get('cloud') if isinstance(live, dict) else None)})
P = np.vstack(P)
n0 = len(P)
# 2.5 mm dedupe
vox = np.floor(P / 0.0025).astype(np.int64)
_, keep = np.unique(vox, axis=0, return_index=True)
P = P[np.sort(keep)]
n1 = len(P)
# table cut (table ~= z 0 in fr3_link0; matches the 011430 recipe)
P = P[P[:, 2] >= 0.025]
n2 = len(P)
# outlier removal: <8 neighbors within 12 mm go
v = np.floor(P / 0.012).astype(np.int64)
keys, inv = np.unique(v, axis=0, return_inverse=True)
cnt = np.bincount(inv)
k2i = {tuple(k): i for i, k in enumerate(keys)}
nbr = np.zeros(len(P), int)
off = [(dx, dy, dz) for dx in (-1, 0, 1) for dy in (-1, 0, 1) for dz in (-1, 0, 1)]
cmap = {k2: cnt[i] for k2, i in k2i.items()}
for i, k in enumerate(keys):
    kx, ky, kz = tuple(k)
    nbr[inv == i] = sum(cmap.get((kx + dx, ky + dy, kz + dz), 0) for dx, dy, dz in off)
P = P[nbr >= 8]
n3 = len(P)
assert n3 >= 200, f'cloud too small after cleaning ({n3})'
np.save(os.path.join(R, 'cloud.npy'), P.astype(np.float32))
out = dict(n=int(n3), centroid=P.mean(axis=0).round(4).tolist(),
           top=float(P[:, 2].max()),
           extent=(P.max(axis=0) - P.min(axis=0)).round(4).tolist(),
           source=('live merged ZED1(SAM2 mask)+ZED2(SAM2-mask-gated), robotfit7 '
                    '(2026-11-05); %d snapshots, 2.5 mm dedupe; table z<0.025 removed; '
                    'outliers removed (<8 nbrs in 12 mm)' % len(snaps)),
           snapshots=meta,
           cleaning=dict(stacked=int(n0), deduped=int(n1), tablecut=int(n2), final=int(n3)))
json.dump(out, open(os.path.join(R, 'cloud.json'), 'w'), indent=1)
print(f'cloud {n0} -> dedupe {n1} -> table {n2} -> final {n3}')
print(f"centroid {out['centroid']} top {out['top']:.4f} extent {out['extent']}")
