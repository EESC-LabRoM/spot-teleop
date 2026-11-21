"""Project a time-paired cached RGB-D capture into base-frame object clouds.
Offline planning input only; no live binding or robot commands.
"""
import json
from pathlib import Path
import cv2
import numpy as np
p=Path(__file__).parent;a=np.load(p/'pair.npz');d=a['depth'];k=a['K'].reshape(3,3);x,y,z,qx,qy,qz,qw=a['camera']
R=np.array([[1-2*(qy*qy+qz*qz),2*(qx*qy-qz*qw),2*(qx*qz+qy*qw)], [2*(qx*qy+qz*qw),1-2*(qx*qx+qz*qz),2*(qy*qz-qx*qw)], [2*(qx*qz-qy*qw),2*(qy*qz+qx*qw),1-2*(qx*qx+qy*qy)]])
for target in ('tennis_ball','sprite'):
 m=cv2.imread(str(p/(target+'_pair_mask.png')),cv2.IMREAD_GRAYSCALE)>0
 valid=m&np.isfinite(d)&(d>.1)&(d<3)
 yy,xx=np.nonzero(valid);dep=d[yy,xx];P=np.c_[(xx-k[0,2])*dep/k[0,0],(yy-k[1,2])*dep/k[1,1],dep]@R.T+np.array([x,y,z])
 print(target,json.dumps({'mask_pixels':int(m.sum()),'valid_depth':len(P),'xyz_min':np.min(P,axis=0).round(3).tolist(),'xyz_max':np.max(P,axis=0).round(3).tolist(),'xyz_median':np.median(P,axis=0).round(3).tolist(),'z_q05_q95':np.quantile(P[:,2],[.05,.95]).round(3).tolist()}))
 np.save(p/(target+'_snapshot_cloud.npy'),P.astype('float32'))
