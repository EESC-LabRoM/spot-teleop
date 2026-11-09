"""Compose the commanded TCP pose with the measured fixed TCP-to-hand transform."""
import numpy as np

def quat_mul(a,b):
 x,y,z,w=a;u,v,s,t=b
 return np.array([w*u+x*t+y*s-z*v,w*v-x*s+y*t+z*u,w*s+x*v-y*u+z*t,w*t-x*u-y*v-z*s])

def tcp_to_hand(position,quaternion,translation,rotation):
 p,q,t,r=(np.asarray(v,float) for v in (position,quaternion,translation,rotation))
 if p.shape!=(3,) or t.shape!=(3,) or q.shape!=(4,) or r.shape!=(4,) or not np.all(np.isfinite(np.r_[p,q,t,r])):
  raise ValueError('Invalid pose/transform')
 if not .99<np.linalg.norm(q)<1.01 or not .99<np.linalg.norm(r)<1.01:raise ValueError('Invalid quaternion norm')
 q=q/np.linalg.norm(q);r=r/np.linalg.norm(r)
 moved=t+2*np.cross(q[:3],np.cross(q[:3],t)+q[3]*t)
 return (p+moved).tolist(),quat_mul(q,r).tolist()
