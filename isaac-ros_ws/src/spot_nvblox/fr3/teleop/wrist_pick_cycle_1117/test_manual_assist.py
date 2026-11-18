"""Exercise the deployed native field with hand goals, expiry, and intent escape."""
import ast,math,time,sys,types,pathlib,numpy as np
src=pathlib.Path('/home/murilo/franka_ws/hub-curobo/node/curobo_mpc_node.py').read_text();tree=ast.parse(src)
functions=[x for x in tree.body if isinstance(x,ast.FunctionDef) and x.name in ['_pf_hold_factor','_qmul','_qconj','_qpow']]
cls=next(x for x in tree.body if isinstance(x,ast.ClassDef) and x.name=='CuroboMpcNode');method=next(x for x in cls.body if isinstance(x,ast.FunctionDef) and x.name=='_apply_grasp_attraction');functions.append(method)
ns=dict(np=np,math=math,time=time);exec(compile(ast.Module(body=functions,type_ignores=[]),'native_field','exec'),ns)
sys.path.insert(0,'/home/murilo/franka_ws/hub-curobo/node');from pf_intent import IntentGate
class T:
 def __init__(self,v):self.v=np.array(v,float)
 def cpu(self):return self
 def numpy(self):return self.v
 def copy_(self,x):self.v[:]=x.v
 def view_as(self,x):self.v=self.v.reshape(x.v.shape);return self
pose=types.SimpleNamespace(position=T([[.59,-.24,.35]]),quaternion=T([[1,0,0,0]]))
from threading import Lock
N=types.SimpleNamespace(_state_lock=Lock(),_grasp_msg=types.SimpleNamespace(header=types.SimpleNamespace(frame_id='fr3_link0'),pose=types.SimpleNamespace(position=types.SimpleNamespace(x=.59,y=-.24,z=.24),orientation=types.SimpleNamespace(w=1,x=0,y=0,z=0))),_grasp_rx=time.time(),goal_buffer=types.SimpleNamespace(goal_pose=pose),_pf={'pf_grasp_timeout_s':.5,'pf_max_weight':.65,'pf_rot_m_per_rad':.03,'pf_radius_m':.3,'pf_full_radius_m':.03},_pf_w=(0,0),_pf_t=time.time()-1,_pf_intent=IntentGate(),_pf_hold=True,_pf_delta=None,_pf_lost_t=None,_pf_hold_max=3.,_pf_hold_fade=2.,_pf_dist_from='hand',_pf_wmax_rot=.85,_pf_rate=1.5,target_frame='fr3_link0',tensor_args=types.SimpleNamespace(to_device=T),step_count=1,_log_info=lambda *a,**k:None,_log_warn=lambda *a,**k:None,current_state=None,mpc=types.SimpleNamespace(rollout_fn=types.SimpleNamespace(compute_kinematics=lambda s:types.SimpleNamespace(ee_pos_seq=T([[.59,-.24,.35]]),ee_quat_seq=T([[1,0,0,0]])))))
apply=lambda:ns['_apply_grasp_attraction'](N)
apply();assert .24<float(pose.position.v[0,2])<.35;print('PASS manual operator goal attracts toward live grasp')
for z in np.linspace(.35,.50,101):N._pf_intent.update([.59,-.24,z],[.59,-.24,.24])
assert N._pf_intent.f<.02;print('PASS outward hand intent releases attraction')
N._grasp_rx=time.time()-1;N._pf_lost_t=time.time()-6;pose.position=T([[.59,-.24,.35]]);apply();assert N._pf_delta is None and pose.position.v[0,2]==.35;print('PASS stale retained correction expires without goal displacement')
assert ns['_pf_hold_factor'](4,3,2)==.5;print('PASS held correction fade interval')
