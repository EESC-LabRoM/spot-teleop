"""Rank a dual-camera run with the FIXED density gate (offline-safe).

Replicates s4_grasp selection (extent gate -> density_gate -> rank_grasps)
without ROS/SSH. Run in vm-viewer: python3 rank_dual_run.py <run_dir>
Expects cloud.npy, cloud.json + /tmp/dual2_grasps.npy in the container.
"""
import json
import os
import shutil
import sys

import numpy as np

sys.path.insert(0, '/home/nexus/spot-teleop/fr3_real_repro/grasp_loop')
from grasp_loop import density_gate, quat_from_matrix, rank_grasps

R = sys.argv[1]
shutil.copy('/tmp/dual2_grasps.npy', os.path.join(R, 'grasps.npy'))
cloud = json.load(open(os.path.join(R, 'cloud.json')))
Pc = np.load(os.path.join(R, 'cloud.npy'))[:, :3].astype(float)
G = np.load(os.path.join(R, 'grasps.npy')).astype(float)
assert G.shape[1] == 17 and len(G) > 0

C = np.array(cloud['centroid'])
E = np.array(cloud.get('extent', [0.3, 0.3, 0.3]))
lo, hi = C - E / 2 - 0.05, C + E / 2 + np.array([0.05, 0.05, 0.08])
cand = [i for i in range(G.shape[0])
        if np.all(G[i, :16].reshape(4, 4)[:3, 3] >= lo)
        and np.all(G[i, :16].reshape(4, 4)[:3, 3] <= hi)]
assert cand, f'0/{G.shape[0]} grasps inside cloud extent'
print(f'pick pool: {len(cand)}/{G.shape[0]} inside extent')
cand, starved, scored = density_gate(G, Pc, cand)
print(f"density gate (FIXED tcp_of_pose): {len(cand)} kept, starved={starved}")
Cmed = np.median(Pc, axis=0)
Gc = G[np.array(cand)]
ranked = rank_grasps(Gc, Pc, Cmed[:2], E[:2], float(cloud['top']))
order = [cand[i] for i, _, _, _ in ranked]
for i, s, m, b in ranked[:5]:
    gi = cand[i]
    print(f'rank idx={gi:3d} score={s:.4f} conf={float(G[gi, 16]):.2f} '
          f'off={1000 * m["off_xy"]:.0f}mm nnear={m["nnear"]} '
          f'bil={m["bilateral"]:.2f} wrap={1000 * m["wrap"]:+.0f}mm vert={m["vert"]:+.2f}')
bi = order[0]
_, bs, bm, bb = ranked[0]
Rm = G[bi, :16].reshape(4, 4)
pos = Rm[:3, 3]
pick = dict(pos=[float(v) for v in pos],
            quat_xyzw=[float(v) for v in quat_from_matrix(Rm[:3, :3])],
            conf=float(G[bi, 16]), n_total=int(G.shape[0]), idx=int(bi),
            score=float(bs),
            metrics=dict(off_xy_mm=round(1000 * bm['off_xy'], 1),
                         nnear=bm['nnear'], bilateral=round(bm['bilateral'], 3),
                         wrap_mm=round(1000 * bm['wrap'], 1),
                         vert=round(bm['vert'], 3)),
            weights={k: round(float(v), 4) for k, v in bb.items()},
            source='merged ZED1(SAM2)+ZED2(SAM2-mask-gated) cloud, fixed density gate (tcp_of_pose)')
json.dump(pick, open(os.path.join(R, 'pick.json'), 'w'), indent=1)
json.dump([dict(idx=int(cand[i]), score=round(float(s), 5),
                conf=round(float(G[cand[i], 16]), 4),
                off_xy_mm=round(1000 * m['off_xy'], 1), nnear=m['nnear'],
                bilateral=round(m['bilateral'], 3),
                wrap_mm=round(1000 * m['wrap'], 1), vert=round(m['vert'], 3))
          for i, s, m, _ in ranked],
          open(os.path.join(R, 'rank.json'), 'w'), indent=1)
json.dump(dict(starved=starved, pool=len(cand), total=int(G.shape[0]),
               gate='fixed-tcp_of_pose'),
          open(os.path.join(R, 'summary.json'), 'w'), indent=1)
print(f'PICK idx={bi} pos={[round(float(v), 4) for v in pos]} conf={pick["conf"]:.3f} score={pick["score"]:.4f}')
