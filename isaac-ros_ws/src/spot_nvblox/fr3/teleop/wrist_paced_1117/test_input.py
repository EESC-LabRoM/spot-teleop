import ast,json,math,types
from pathlib import Path
import numpy as np
R=Path(__file__).parent
OLD=R.parent/'live_ready_1117'
def load(path):
 tree=ast.parse(path.read_text())
 cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and any(isinstance(f,ast.FunctionDef) and f.name=='_stable_zed' for f in n.body))
 funcs=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name.startswith('q')]
 method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='_stable_zed')
 scope=dict(np=np,math=math)
 exec(compile(ast.fix_missing_locations(ast.Module(body=funcs+[method],type_ignores=[])),str(path),'exec'),scope)
 return scope,cls
def node():
 return types.SimpleNamespace(zed_stable=[],zed_stable_stamp=-1,zed_stamp=-1,zed_position_only_protocol=True,zed_orientation_valid=True,_zed_live=lambda:True)

new,cls=load(R/'fr3_input_teleop_wrist_1117.py')
def feed(n,fn,t,p):
 n.zed_stamp=int(t*1e9);n.zed_sample=(np.asarray(p),np.array([0.,0.,0.,1.]))
 return fn(n,t)
# Continuous motion must engage, duplicates and a single spike must not.
n=node()
assert any(feed(n,new['_stable_zed'],1+i*.04,[i*.02,0,0]) for i in range(8))
n=node()
for i in range(12):assert not feed(n,new['_stable_zed'],1,[0,0,0])
n=node()
for i in range(3):assert not feed(n,new['_stable_zed'],1+i*.04,[0,0,0])
assert not feed(n,new['_stable_zed'],1.12,[1,0,0])
assert not feed(n,new['_stable_zed'],1.16,[0,0,0])
assert not feed(n,new['_stable_zed'],2.,[0,0,0])
# Test the exact target mapping statements from the staged consumer.
tick=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='tick')
lock=[n for n in tick.body if isinstance(n,ast.With)][-1]
mapping=lock.body[0].body
code=compile(ast.fix_missing_locations(ast.Module(body=mapping,type_ignores=[])),'consumer target mapping','exec')
n=types.SimpleNamespace(zed_anchor=(np.array([.3,0,.5]),np.array([1.,0,0,0]),np.zeros(3),np.array([0.,0,0,1.])),zed_sample=(np.zeros(3),np.array([0.,.7071067811865475,0,.7071067811865475])),cmd_q=np.array([1.,0,0,0]),zed_orientation_valid=True,_zed_orientation_anchor_valid=False,zed_forward_sign=-1,zed_scale=1.,zed_mapping_profile='legacy',zed_position_only_protocol=True,zed_rotation_pending=True,zed_orientation_mode='full_6d',zed_table_facing=False)
scope=dict(new,self=n,dt=.02);exec(code,scope)
assert new['qangle'](n.tgt_q,n.cmd_q)<1e-7
anchor=tuple(x.copy() for x in n.zed_anchor)
n.zed_orientation_valid=False;exec(code,scope)
n.zed_orientation_valid=True;n.zed_sample=(np.array([.1,.02,.03]),new['qmul'](new['qexp'](np.array([0,0,.4])),anchor[3]))
exec(code,scope)
assert all(np.array_equal(a,b) for a,b in zip(anchor,n.zed_anchor))
assert abs(new['qangle'](n.tgt_q,n.cmd_q)-.4)<1e-6
assert np.allclose(n.tgt_p,[.2,.02,.53])

from wrist_motion_filter import bounded_step
velocity=np.zeros(3)
for k in range(1000):
 desired=np.array([.003,0,.001])*(1 if k%120<60 else -1)
 step,new_velocity=bounded_step(velocity,desired,.02,.5)
 assert np.linalg.norm(new_velocity-velocity)<=.010000001
 assert np.linalg.norm(new_velocity)<=np.linalg.norm(desired/.02)+1e-8
 assert np.allclose(step,new_velocity*.02)
 velocity=new_velocity
for dt in [0,-1,float('nan')]:
 try:bounded_step(velocity,np.zeros(3),dt,.5)
 except ValueError:pass
 else:raise AssertionError(dt)
print('Passed moving startup, duplicate/spike/gap rejection, first orientation binding, occlusion reference preservation, combined motion, and 1000 acceleration reversal steps')
