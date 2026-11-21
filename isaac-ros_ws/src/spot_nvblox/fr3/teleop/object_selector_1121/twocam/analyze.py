from pathlib import Path
import sys,json
import cv2,numpy as np
from sklearn.cluster import DBSCAN
p=Path(__file__).parent;sys.path.insert(0,str(p.parent/'backend'))
from ball_grasp import fit_ball,assess_sprite_cloud
fits={}
for cam in (1,2):
 a=np.load(p/f'zed{cam}.npz');d=a['depth'];k=a['K'].reshape(3,3);x,y,z,qx,qy,qz,qw=a['camera'];R=np.array([[1-2*(qy*qy+qz*qz),2*(qx*qy-qz*qw),2*(qx*qz+qy*qw)],[2*(qx*qy+qz*qw),1-2*(qx*qx+qz*qz),2*(qy*qz-qx*qw)],[2*(qx*qz-qy*qw),2*(qy*qz+qx*qw),1-2*(qx*qx+qy*qy)]])
 for target in ('tennis_ball','sprite'):
  mask=cv2.imread(str(p/f'zed{cam}_{target}_mask.png'),0)>0;valid=mask&np.isfinite(d)&(d>.1)&(d<3);yy,xx=np.nonzero(valid);dep=d[yy,xx];P=np.c_[(xx-k[0,2])*dep/k[0,0],(yy-k[1,2])*dep/k[1,1],dep]@R.T+np.array([x,y,z]);np.save(p/f'zed{cam}_{target}_cloud.npy',P.astype('float32'))
  if target=='tennis_ball':
   P=P[P[:,2]>.04];groups=DBSCAN(eps=.015,min_samples=8).fit_predict(P);ids,counts=np.unique(groups[groups>=0],return_counts=True);P=P[groups==ids[counts.argmax()]];np.save(p/f'zed{cam}_ball_cluster.npy',P.astype('float32'));f=fit_ball(P,.0287);fits[cam]=f;print('zed',cam,'ball',f)
  else:print('zed',cam,'sprite',json.dumps(assess_sprite_cloud(P)))
print('delta XY',float(np.linalg.norm(np.asarray(fits[1].center[:2])-np.asarray(fits[2].center[:2]))),'delta z',float(abs(fits[1].center[2]-fits[2].center[2])))
