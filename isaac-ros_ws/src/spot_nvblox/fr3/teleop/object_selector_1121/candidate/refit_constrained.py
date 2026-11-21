"""Offline experiment, NOT for live collision geometry.
Compare 3D clusters against archived 4-sphere models on recorded partial clouds.
No worst-case shape guarantee; candidate must not be deployed from this script.
"""
import json
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parent.parent/'fit_inputs'
runs=('20261121_050322_drill_oneshot_regen','20261121_051628_drill_loop_regen','20261121_052029_drill_loop_regen')
rng=np.random.default_rng(27)
for run in runs:
 folder=root/'grasp_loop/runs'/run
 points=np.load(folder/'cloud.npy')[:,:3]
 plan=json.loads((folder/'plan.json').read_text())
 hand=(points-np.asarray(plan['grasp_pos']))@np.asarray(plan['hand_R'])
 zfar=np.percentile(hand[:,2],99.5)
 old=np.asarray(json.loads((root/'regrasp/state'/('held_model_'+run+'.json')).read_text())['spheres'])
 best=None
 for rep in range(200):
  centers=hand[rng.choice(len(hand),4,False)].copy()
  for _ in range(25):
   distance=np.linalg.norm(hand[:,None]-centers[None,:,:],axis=2)
   assignment=distance.argmin(1)
   new=np.array([hand[assignment==j].mean(0) if (assignment==j).any() else centers[j] for j in range(4)])
   if np.max(abs(new-centers))<1e-5:break
   centers=new
  distance=np.linalg.norm(hand[:,None]-centers[None,:,:],axis=2)
  assignment=distance.argmin(1)
  radius=np.array([min(np.percentile(distance[assignment==j,j],97)+.012,.12) if (assignment==j).sum()>20 else .005 for j in range(4)])
  centers[:,2]=np.minimum(centers[:,2],zfar+.015-radius)
  distance=np.linalg.norm(hand[:,None]-centers[None,:,:],axis=2)
  coverage=(distance<=radius).any(1).mean()
  key=(coverage>=.98,-float(np.sum(radius**3))) if coverage>=.98 else (False,float(coverage))
  if best is None or key>best[0]:best=(key,centers.copy(),radius.copy(),float(coverage))
 print(json.dumps(dict(run=run,coverage=round(best[3],4),radii=best[2].round(4).tolist(),volume_ratio=round(float(np.sum(best[2]**3)/np.sum(old[:,3]**3)),4),far_end_limit_m=.015,offline_only=True)))
