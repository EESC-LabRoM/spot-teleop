from pathlib import Path
import sys,math,pathlib,numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent));from bounded_assist import bounded_delta
p=np.array([.59,-.24,.4234]);q=np.array([1.,0,0,0]);target=np.array([.59,-.24,.2222]);qt=np.array([math.cos(math.radians(35)/2),0,0,math.sin(math.radians(35)/2)])
for w in np.linspace(0,1,101):
 a,b=bounded_delta(p,q,p+w*(target-p),qt);assert np.linalg.norm(a-p)<=.020000001;assert 2*math.acos(min(1.,abs(b@q)))<=math.radians(10)+1e-9
# Repeated native steps each start from the same operator goal; no runaway.
for _ in range(1000):a,b=bounded_delta(p,q,target,qt)
assert abs(a[2]-.4034)<1e-9
assert np.allclose(bounded_delta(p,q,p,-q)[1],q)
print('PASS 101 weights, repeated operator-relative cap, antipodal rotation')
import importlib.util,time,types
s=importlib.util.spec_from_file_location('native_test',pathlib.Path(__file__).with_name('test_manual_assist.py'));m=importlib.util.module_from_spec(s);s.loader.exec_module(m)
from bounded_assist import apply
N=m.N;N._grasp_rx=time.time();N._pf_lost_t=None;N._pf_t=time.time()-1;N._pf_w=(0,0);N._pf_intent.reset();N.goal_buffer.goal_pose.position=m.T([[.59,-.24,.35]]);N._apply_grasp_attraction=types.MethodType(m.ns['_apply_grasp_attraction'],N)
apply(N);assert np.linalg.norm(N._pf_delta[0])<=.020000001;assert abs(N.goal_buffer.goal_pose.position.v[0,2]-.33)<1e-9
N._grasp_rx=time.time()-1;N._pf_lost_t=time.time()-6;N.goal_buffer.goal_pose.position=m.T([[.59,-.24,.35]]);apply(N);assert N._pf_delta is None and N.goal_buffer.goal_pose.position.v[0,2]==.35
print('PASS native integration caches bounded correction and preserves expiry')
