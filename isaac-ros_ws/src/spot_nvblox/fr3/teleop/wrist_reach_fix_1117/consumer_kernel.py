"""Exact staged consumer mapping/follower in isolated dynamics tests. No ROS transport."""
import ast,math,types,os
from pathlib import Path
import numpy as np
from table_facing import TableFacing
class Kernel:
 def __init__(self,source,p,q,speed=1.):
  tree=ast.parse(Path(source).read_text());self.scope=dict(np=np,math=math)
  funcs=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name in ('qmul','qexp','qrot','qangle','qslerp','toward')]
  cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Fr3JoyTeleop')
  funcs += [n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in ('_stable_zed','_recover_moving_zed')]
  exec(compile(ast.fix_missing_locations(ast.Module(body=funcs,type_ignores=[])),str(source),'exec'),self.scope)
  tick=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='tick')
  block=[n for n in tick.body if isinstance(n,ast.With)][-1]
  block.body.insert(1,ast.parse('self.unclamped_intent_p=self.tgt_p.copy()').body[0])
  self.code=compile(ast.fix_missing_locations(ast.Module(body=block.body,type_ignores=[])),str(source),'exec')
  self.resets=0
  class Log:
   def info(_,message,**kw):
    if message.startswith('arm not following'):self.resets+=1
  self.n=types.SimpleNamespace(
   zed_rotation_control="rate_yaw",_rate_yaw=None,zed_vertical_gain=float(os.environ.get("VERTICAL_GAIN","1")),zed_preserve_workspace_anchor=os.environ.get("PRESERVE_ANCHOR","1")=="1",zed_input_position_gain=1.315,zed_translation_axes="legacy",zed_yaw_rate_deg_s=float(os.environ.get("YAW_RATE","15")),zed_rotation_soft_error=math.radians(8),zed_pacing_error=.05,zed_linear_accel=float(os.environ.get('CART_ACCEL','.5')),_zed_velocity=np.zeros(3),input_source='zed',zed_anchor=None,zed_stable=[],zed_stable_stamp=-1,zed_stamp=-1,
   zed_orientation_valid=True,zed_position_only_protocol=True,zed_forward_sign=1.,zed_scale=float(os.environ.get("TRANSLATION_GAIN","1")),
   zed_mapping_profile='legacy',zed_orientation_mode='full_6d',zed_rotation_pending=False,
   _zed_orientation_anchor_valid=False,zed_table_facing=True,_table_profile=None,
   cmd_p=p.copy(),cmd_q=q.copy(),tgt_p=p.copy(),tgt_q=q.copy(),
   box_lo=np.array([.2,-.55,.10]),box_hi=np.array([.8,.7,1.]),
   max_lead=.05,max_slew=math.radians(4),rate=50.,tau=.04,
   v_lin=.30,v_z=float(os.environ.get("VERTICAL_SPEED",".20")),speed_scale=1.*speed,w_ang=math.radians(90),rot_scale=.5,
   max_step=.02,max_error=.08,max_rot_err=math.radians(30),zed_recovery_pair=None,
   _zed_live=lambda:True,get_logger=lambda:Log())
  if os.environ.get('OLD_CONTROLS')=='1':self.n.v_lin=.25;self.n.v_z=.16;self.n.speed_scale=.625;self.n.zed_pacing_error=.03
  self.active=False;self.last=None;self.loss=False;self.started=None;self.frames=0
 def step(self,t,packet,receipt,p,q):
  n=self.n
  if packet is not None:
   n.zed_stamp=packet['stamp_ns'];n.zed_sample=(np.array(packet['p']),np.array(packet['q']))
   n.zed_orientation_valid=packet['orientation_valid'];self.last=receipt
  live=self.last is not None and t-self.last<.35
  if not live:
   if self.active and not self.loss:self.loss=True;n.zed_recovery_pair=None;n._zed_velocity=np.zeros(3)
   return n.cmd_p.copy(),n.cmd_q.copy()
  if not self.active:
   if not self.scope['_stable_zed'](n,t):return n.cmd_p.copy(),n.cmd_q.copy()
   n.cmd_p=p.copy();n.cmd_q=q.copy();n.tgt_p=p.copy();n.tgt_q=q.copy()
   n.zed_anchor=(p.copy(),q.copy(),n.zed_sample[0].copy(),n.zed_sample[1].copy())
   n._zed_orientation_anchor_valid=n.zed_orientation_valid;n.zed_rotation_pending=not n.zed_orientation_valid
   n._table_profile=TableFacing(q)
   from rate_yaw import RateYaw
   n._rate_yaw=RateYaw(n.zed_sample[1],q,n.zed_yaw_rate_deg_s) if n.zed_orientation_valid else None
   self.active=True;self.started=t
  if self.loss:
   if not self.scope['_recover_moving_zed'](n):return n.cmd_p.copy(),n.cmd_q.copy()
   self.loss=False
  exec(self.code,dict(self.scope,self=n,dt=.02,mp=p,mq=q,v=np.zeros(3),w=np.zeros(3)))
  self.frames+=1
  return n.cmd_p.copy(),n.cmd_q.copy()
