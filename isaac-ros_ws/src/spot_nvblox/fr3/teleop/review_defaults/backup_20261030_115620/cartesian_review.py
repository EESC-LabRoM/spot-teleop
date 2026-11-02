"""Straight TCP axes and explicit single-motion review profiles.

User axes: X=horizontal (robot Y), Y=height (robot Z), Z=reach (robot X).
Profiles isolate a recorded exercise; they are not an inferred gesture classifier.
"""
import math
import numpy as np
from teleop_mapping import _quat_mul,palm_normal

PROFILES={
 'full_6d':dict(translation=(1,1,1),rotation='full',label='Translação e orientação combinadas'),
 'free':dict(translation=(1,1,1),rotation='roll',label='Movimento livre'),
 'neutral':dict(translation=(0,0,0),rotation=None,label='Parado'),
 'horizontal':dict(translation=(0,1,0),rotation=None,label='Esquerda/direita → só horizontal X'),
 'height':dict(translation=(0,0,1),rotation=None,label='Frente/trás → só altura Y'),
 'reach':dict(translation=(1,0,0),rotation=None,label='Cima/baixo → só alcance Z'),
 'roll':dict(translation=(0,0,0),rotation='roll',label='Giro no plano → só giro da garra'),
 'pitch':dict(translation=(0,0,0),rotation='pitch',label='Inclinar dedos → só inclinação da garra'),
 'yaw':dict(translation=(0,0,0),rotation='yaw',label='Virar palma → só giro lateral da garra'),
 'gripper':dict(translation=(0,0,0),rotation=None,label='Abrir/fechar → só garra'),
 'cross':dict(translation=(1,1,0),rotation=None,label='Cruz → horizontal X e alcance Z'),
 'translation':dict(translation=(1,1,1),rotation=None,label='Translação e garra; orientação fixa')}
STAGES=['neutral','roll','pitch','yaw','height','horizontal','reach','gripper','cross','translation','translation','translation']

def exercise_profile(source_id):
 stage=int(source_id.split('_realsense_')[1].split('_')[0]);return STAGES[stage]

def projected_angle(delta,axis):
 delta=np.asarray(delta,float);delta/=np.linalg.norm(delta)
 if delta[3]<0:delta=-delta
 angle=2*math.atan2(float(np.dot(delta[:3],axis)),float(delta[3]))
 return (angle+math.pi)%(2*math.pi)-math.pi

def target(anchor_p,anchor_q,p0,q0,p,q,gains,rotation_gain,profile='free',deadband=.008,max_translation=.20):
 if profile not in PROFILES:raise ValueError('Unknown movement profile')
 cfg=PROFILES[profile];ap=np.asarray(anchor_p,float);aq=np.asarray(anchor_q,float);aq/=np.linalg.norm(aq)
 dp=np.asarray(p,float)-np.asarray(p0,float);g=np.asarray(gains,float)
 # Packet coordinates: depth X, horizontal Y, vertical Z. No polar arc or base yaw.
 d=np.array([g[0]*dp[2],g[1]*dp[1],-g[2]*dp[0]])
 d=np.sign(d)*np.maximum(np.abs(d)-deadband,0.)
 d=np.clip(d,-max_translation,max_translation)*np.asarray(cfg['translation'])
 out_p=ap+d;out_q=aq.copy()
 rotation=cfg['rotation']
 if rotation is not None and rotation_gain:
  q=np.asarray(q,float);q0=np.asarray(q0,float);delta=_quat_mul(q,[-q0[0],-q0[1],-q0[2],q0[3]])
  if rotation=='full':
   # Match the approved isolated axes; preserve signs in the quaternion delta.
   n0=palm_normal(q0)
   if np.dot(n0,[-1.,0,0])<-.2:n0=-n0
   angles=np.array([projected_angle(delta,[0.,0.,1.]),projected_angle(delta,[0.,1.,0.]),projected_angle(delta,n0)])
   angles=np.sign(angles)*np.maximum(np.abs(angles)-math.radians(3),0.)
   angles=np.clip(angles*np.array([1.2,1.2,1.6])*rotation_gain,-math.radians(75),math.radians(75))
   for output_axis,a in zip(np.eye(3),angles):out_q=_quat_mul(out_q,np.r_[output_axis*math.sin(a/2),math.cos(a/2)])
   return out_p,out_q/np.linalg.norm(out_q)
  if rotation=='roll':
   axis=palm_normal(q0)
   if np.dot(axis,[-1.,0,0])<-.2:axis=-axis
   output_axis=np.array([0.,0.,1.])
  elif rotation=='pitch':axis=np.array([0.,1.,0.]);output_axis=np.array([0.,1.,0.])
  else:axis=np.array([0.,0.,1.]);output_axis=np.array([1.,0.,0.])
  angle=projected_angle(delta,axis)
  angle=math.copysign(max(0.,abs(angle)-math.radians(3)),angle)
  angle=float(np.clip(angle*rotation_gain,-math.radians(75),math.radians(75)))
  out_q=_quat_mul(aq,np.r_[output_axis*math.sin(angle/2),math.cos(angle/2)])
 return out_p,out_q/np.linalg.norm(out_q)
