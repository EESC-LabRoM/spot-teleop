"""Fail-closed validation of one planned FR3 joint command against current feedback."""
import math
import numpy as np
ARM=[f'fr3_joint{i}' for i in range(1,8)]
# FR3 description limits, radians (not Panda limits).
LOW=np.array([-2.7437,-1.7837,-2.9007,-3.0421,-2.8065,.5445,-3.0159])
HIGH=np.array([2.7437,1.7837,2.9007,-.1518,2.8065,4.5169,3.0159])

def validate(packet,measured,now_ns,previous=None):
 if packet.get('version')!=1 or packet.get('frame')!='fr3_link0':raise ValueError('planner schema/frame')
 for key,timeout in (('stamp_ns',.15),('goal_stamp_ns',.35),('joint_stamp_ns',.2),('esdf_stamp_ns',1.5),('depth_stamp_ns',.7)):
  age=(now_ns-int(packet[key]))*1e-9
  if not 0<=age<timeout:raise ValueError('stale '+key)
 if packet.get('feasible') is not True:raise ValueError('MPC solution infeasible')
 if packet.get('grid_covers_robot') is not True:raise ValueError('robot outside map coverage')
 if packet.get('names')!=ARM:raise ValueError('joint names/order')
 q=np.asarray(packet['q'],float);v=np.asarray(packet['v'],float);m=np.asarray(measured,float)
 if q.shape!=(7,) or v.shape!=(7,) or m.shape!=(7,) or not np.all(np.isfinite(np.r_[q,v,m])):raise ValueError('nonfinite/malformed joints')
 if np.any(q<LOW) or np.any(q>HIGH):raise ValueError('joint limit')
 if np.max(np.abs(v))>1.201:raise ValueError("joint velocity limit")   # vcap1.2 MPC config (was 0.801 with vcap0.8)
 if np.max(np.abs(q-m))>.05:raise ValueError('measured joint tracking error')
 if previous is None:
  if np.max(np.abs(q-m))>.015:raise ValueError('first joint step')
 else:
  dt=(int(packet['stamp_ns'])-int(previous['stamp_ns']))*1e-9
  if not 0<dt<=.15:raise ValueError('replayed or interrupted command')
  if np.max(np.abs(q-np.array(previous["q"])))>.005+1.2*dt:raise ValueError("joint step limit")   # matches the 1.2 rad/s cap
  if np.max(np.abs(v-np.array(previous['v'])))>.05+10.*dt:raise ValueError('joint acceleration limit')
 return q.tolist(),v.tolist()
