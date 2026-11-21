"""Offline diagnostic: fixed-radius sphere constrained to measured table.
Reports disagreement without attributing its cause or validating a robot grasp.
"""
from pathlib import Path
import json
import numpy as np
from scipy.optimize import least_squares
p=Path(__file__).parent
radius=.0335
table=.0287
center_z=table+radius
results={}
for cam in (1,2):
 points=np.load(p/f'zed{cam}_ball_cluster.npy')
 fit=least_squares(lambda xy:np.linalg.norm(points-np.array([xy[0],xy[1],center_z]),axis=1)-radius,np.median(points[:,:2],axis=0),loss='soft_l1',f_scale=.003)
 errors=np.linalg.norm(points-np.r_[fit.x,center_z],axis=1)-radius
 results[str(cam)]={'xy':fit.x.tolist(),'rms_m':float(np.sqrt(np.mean(errors**2))),'median_abs_m':float(np.median(abs(errors))),'p90_abs_m':float(np.quantile(abs(errors),.9))}
results['xy_gap_m']=float(np.linalg.norm(np.array(results['1']['xy'])-np.array(results['2']['xy'])))
results['radius_m']=radius
results['assumed_table_m']=table
results['interpretation']='Model-constrained diagnostic only; camera calibration, stereo bias, partial surface, and object motion remain possible contributors.'
(p/'fixed_radius_fit.json').write_text(json.dumps(results,indent=2)+'\n')
print(json.dumps(results,indent=2))
