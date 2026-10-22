"""Simulation-only cuRobo MPC, operator calibration, gripper and local/Tailscale viewer.
All robot interfaces live under /fr3_sim in DDS domain 188. No hardware driver is used.
"""
import json,math,os,sys,time,threading
from pathlib import Path
from http.server import ThreadingHTTPServer,BaseHTTPRequestHandler
import numpy as np
import cv2
import torch
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState,Image
from std_msgs.msg import String,Float32
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent))
sys.path.insert(0,str(HERE.parents[4]/'spot-ros2_ws/src/arm_pose_estimator'))
from tracking_gate import TrackingGate
from fr3_curobo_mpc_node import fr3_robot_cfg
from arm_pose_estimator.hand_frame import quaternion_matrix,matrix_quaternion
from curobo.geom.types import WorldConfig
from curobo.geom.sdf.world import CollisionCheckerType
from curobo.rollout.rollout_base import Goal
from curobo.types.math import Pose
from curobo.types.state import JointState as CuState
from curobo.wrap.reacher.mpc import MpcSolver,MpcSolverConfig
ARM=[f'fr3_joint{i}' for i in range(1,8)]
def angle(a,b):return 2*math.acos(min(1.,abs(float(np.dot(a,b)))))
def toward(a,b,limit):
 d=float(a@b)
 if d<0:b=-b;d=-d
 theta=math.acos(np.clip(d,-1.,1.));f=min(1.,limit/max(2*theta,1e-9))
 if d>.9995:q=a+f*(b-a)
 else:q=(math.sin((1-f)*theta)*a+math.sin(f*theta)*b)/math.sin(theta)
 return q/np.linalg.norm(q)
class Session(Node):
 def __init__(self):
  super().__init__('fr3_remote_sim_session')
  self.lock=threading.RLock();self.q=None;self.q_at=0.;self.tcp=None;self.tcp_at=0.;self.sample=None;self.sample_at=0.;self.stamp=-1
  self.hold_q=None;self.hold_phase=None;self.gate=TrackingGate();self.sample_times=[];self.last_control=None;self.control_count=0;self.tracking_info={}
  self.phase='idle';self.note='Clique Start para calibrar';self.stable=[];self.start_deadline=0.;self.paused_at=None
  self.anchor=None;self.cmd=None;self.home=None;self.scale=.5;self.gesture=None;self.grip_candidate=None;self.grip_frames=0;self.grip_since=0.;self.grip_last=None;self.grip_time=0.;self.grip_stamp=-1
  self.overlay_at=0.;self.finger_width=None;self.overlay=None;self.steps=0;self.solver_ms=0.;self.goal_buffer=None;self.mpc_ready=False;self.goal_pose=None
  self.pub=self.create_publisher(JointState,'/fr3_sim/joint_command',1)
  self.goal_pub=self.create_publisher(PoseStamped,'/fr3_sim/goal',1)
  self.grip_pub=self.create_publisher(Float32,'/fr3_sim/gripper_width',1)
  self.create_subscription(JointState,'/fr3_sim/joint_states',self.on_js,10)
  self.create_subscription(PoseStamped,'/fr3_sim/tcp',self.on_tcp,10)
  self.create_subscription(String,'/fr3_sim/operator/sample',self.on_sample,qos_profile_sensor_data)
  self.create_subscription(String,'/fr3_sim/operator/tracking',self.on_tracking,qos_profile_sensor_data)
  self.create_subscription(Image,'/fr3_sim/operator/overlay',self.on_overlay,qos_profile_sensor_data)
  self.mpc_error=None;self.mpc_metrics={};self.mpc_warmed=False;self.warmup_ms=None
  try:self.load_mpc()
  except Exception as error:
   self.mpc_error=str(error);print('MPC unavailable:',self.mpc_error,flush=True)
  self.create_timer(.05,self.tick)
 def load_mpc(self):
  cfg=fr3_robot_cfg(str(HERE.parent/'fr3.urdf'))
  cfg['kinematics']['collision_link_names']=[n for n in cfg['kinematics']['collision_link_names'] if n!='fr3_link0']
  world=WorldConfig.from_dict({'cuboid':{
   'table':{'dims':[1.4,1.1,.08],'pose':[.35,0.,-.04,1.,0.,0.,0.]},
   'bottle':{'dims':[.09,.09,.24],'pose':[.57,.08,.12,1.,0.,0.,0.]},
   'mug':{'dims':[.11,.11,.14],'pose':[.35,.28,.07,1.,0.,0.,0.]},
   'box':{'dims':[.20,.16,.12],'pose':[.5,.43,.06,1.,0.,0.,0.]}}})
  print('Loading simulation cuRobo MPC',flush=True)
  mc=MpcSolverConfig.load_from_robot_config(cfg,world,use_cuda_graph=True,use_cuda_graph_metrics=False,use_cuda_graph_full_step=False,
     self_collision_check=True,collision_checker_type=CollisionCheckerType.PRIMITIVE,use_mppi=True,use_lbfgs=False,use_es=False,store_rollouts=False,step_dt=.05)
  self.mpc=MpcSolver(mc);self.mpc.enable_cspace_cost(False);self.mpc.enable_pose_cost(True);self.ta=self.mpc.tensor_args;self.jn=self.mpc.rollout_fn.joint_names
  self.mpc_ready=True
  print('Simulation controller ready; HTTP 8791; DDS 188; /fr3_sim only',flush=True)
 def on_js(self,m):
  d=dict(zip(m.name,m.position))
  if all(j in d for j in ('fr3_finger_joint1','fr3_finger_joint2')):self.finger_width=float(d['fr3_finger_joint1']+d['fr3_finger_joint2'])
  if all(j in d for j in ARM):self.q=np.array([d[j] for j in ARM],np.float32);self.q_at=time.monotonic()
 def on_tcp(self,m):
  p,q=m.pose.position,m.pose.orientation
  self.tcp=(np.array([p.x,p.y,p.z]),np.array([q.x,q.y,q.z,q.w]));self.tcp_at=time.monotonic()
  if self.home is None:self.home=tuple(v.copy() for v in self.tcp)
 def on_sample(self,m):
  try:
   d=json.loads(m.data);stamp=int(d['stamp_ns']);age=(time.time_ns()-stamp)*1e-9
   p,q=np.asarray(d['p'],float),np.asarray(d['q'],float)
   if stamp<=self.stamp or d.get('version')!=2 or d.get('frame')!='fr3_link0' or not 0<=age<.65:return
   if d.get('orientation_mode')!='full_6d' or int(d['orientation_stamp_ns'])!=stamp:return
   if p.shape!=(3,) or q.shape!=(4,) or not np.all(np.isfinite(np.r_[p,q])) or abs(np.linalg.norm(q)-1)>.01:return
   self.sample=(p,q);self.sample_at=time.monotonic()-age;self.stamp=stamp
   self.gate.observe(stamp,self.sample_at,p,q)
   self.sample_times=[v for v in self.sample_times if time.monotonic()-v<2.]+[time.monotonic()]
   self.gesture=d.get('gesture') if float(d.get('gesture_score',0))>=.75 else None
  except (ValueError,TypeError,KeyError):pass
 def on_tracking(self,m):
  try:
   d=json.loads(m.data)
   if not isinstance(d,dict):return
   self.tracking_info=d;self.gate.latest_reason=str(d.get('reason','Mostre a mão direita'))
  except (ValueError,TypeError):pass
 def on_overlay(self,m):
  try:
   data=np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3)
   ok,jpg=cv2.imencode('.jpg',data,[cv2.IMWRITE_JPEG_QUALITY,75])
   if ok:self.overlay=jpg.tobytes();self.overlay_at=time.monotonic()-(time.time_ns()-m.header.stamp.sec*10**9-m.header.stamp.nanosec)*1e-9
  except (ValueError,cv2.error):pass
 def control(self,action,value=None):
  with self.lock:
   self.last_control=dict(action=action,at=time.time());self.control_count+=1
   print('CONTROL',action,'phase',self.phase,flush=True)
   if action=='stop':self.gate.stop(time.monotonic());self.hold_q=None;self.phase='idle';self.note='Controle parado';self.paused_at=None;self.stable=[];self.anchor=None;return
   if action=='speed':
    speed=float(value)
    if not math.isfinite(speed):raise ValueError('Velocidade inválida')
    self.scale=float(np.clip(speed,.05,1.));return
   if not self.mpc_ready:raise ValueError('cuRobo indisponível: '+str(self.mpc_error))
   if not self.mpc_warmed:raise ValueError('Preparando o controle; aguarde um instante')
   if self.tcp is None or time.monotonic()-self.tcp_at>.5:raise ValueError('Aguardando estado da simulação')
   if action=='start':
    self.gate.start(time.monotonic());self.phase=self.gate.phase;self.note=self.gate.note;self.hold_q=None;return
   if action=='home':self.gate.stop(time.monotonic());self.phase='home';self.note='Voltando à posição inicial';self.cmd=tuple(v.copy() for v in self.tcp);return
   if action in ('open','close'):
    self.grip_pub.publish(Float32(data=.04 if action=='open' else 0.));self.grip_last='Open_Palm' if action=='open' else 'Closed_Fist';self.grip_time=time.monotonic();self.grip_candidate=None;self.grip_frames=0;self.note='Garra: '+action;return
   if action=='rotate':
    self.gate.stop(time.monotonic());self.phase='test';self.note='Teste de giro de 10 graus';self.cmd=tuple(v.copy() for v in self.tcp)
    t=math.radians(10);r=np.array([[math.cos(t),-math.sin(t),0],[math.sin(t),math.cos(t),0],[0,0,1]])
    self.test_pose=(self.tcp[0].copy(),matrix_quaternion(r@quaternion_matrix(self.tcp[1])));return
   if action=='test':self.gate.stop(time.monotonic());self.phase='test';self.note='Teste de 3 cm na simulação';self.cmd=tuple(v.copy() for v in self.tcp);self.test_pose=(self.tcp[0]+[0.,-.03,0.],self.tcp[1].copy());return
   raise ValueError('Ação desconhecida')
 def engage(self):
  self.anchor=(*[v.copy() for v in self.tcp],*[v.copy() for v in self.sample]);self.cmd=tuple(v.copy() for v in self.tcp)
  self.phase='active';self.note=self.gate.note;self.paused_at=None;self.grip_candidate=None;self.grip_frames=0
 def tick(self):
  if not self.mpc_ready:return
  now=time.monotonic()
  if self.q is None or self.tcp is None or now-self.q_at>.5 or now-self.tcp_at>.5:
   if self.phase in ('active','paused'):
    self.gate.pause(now,'Pausado: aguardando estado recente do simulador');self.phase=self.gate.phase;self.note=self.gate.note
   return
  if not self.mpc_warmed:
   warmup=time.monotonic()
   try:
    for _ in range(3):self.drive_mpc(*self.tcp,publish=False)
    self.warmup_ms=round((time.monotonic()-warmup)*1000,1);self.mpc_warmed=True
    print('MPC warmup complete',self.warmup_ms,'ms',flush=True)
   except RuntimeError as error:self.mpc_ready=False;self.mpc_error=str(error)
   return
  with self.lock:
   if self.phase in ('idle','calibrating','active','paused'):
    activated=self.gate.step(now)
    self.phase,self.note=self.gate.phase,self.gate.note
    if activated:self.engage()
   if self.phase=='active':
    ap,aq,p0,q0=self.anchor;p,q=self.sample
    # This workcell faces +x into the table. Physical robot remains configured separately.
    target=(ap+(p-p0),matrix_quaternion(quaternion_matrix(q)@quaternion_matrix(q0).T@quaternion_matrix(aq)))
   elif self.phase=='home':target=self.home
   elif self.phase=='test':target=self.test_pose
   else:target=tuple(v.copy() for v in self.tcp);self.cmd=target
   if self.phase in ('active','home','test'):
    p,q=self.cmd;tp,tq=target;tp=np.clip(tp,[.02,-.5,.04],[.8,.5,.8])
    d=tp-p;n=np.linalg.norm(d[:2]);limit=.6*self.scale*.05
    if n>limit:d[:2]*=limit/n
    d[2]=np.clip(d[2],-.4*self.scale*.05,.4*self.scale*.05)
    p=p+d;delta=p-self.tcp[0];dn=np.linalg.norm(delta)
    if dn>.06:p=self.tcp[0]+delta*.06/dn
    q=toward(q,tq,math.radians(90)*self.scale*.05)
    q=toward(self.tcp[1],q,math.radians(30));self.cmd=(p,q)
   gp,gq=self.cmd
   goalmsg=PoseStamped();goalmsg.header.frame_id='fr3_link0';goalmsg.header.stamp=self.get_clock().now().to_msg()
   goalmsg.pose.position.x,goalmsg.pose.position.y,goalmsg.pose.position.z=map(float,gp)
   goalmsg.pose.orientation.x,goalmsg.pose.orientation.y,goalmsg.pose.orientation.z,goalmsg.pose.orientation.w=map(float,gq)
   self.goal_pub.publish(goalmsg)
   if self.phase in ('active','home','test'):
    self.hold_q=None
    try:self.drive_mpc(gp,gq)
    except RuntimeError as error:
     self.mpc_ready=False;self.mpc_error=str(error);self.gate.stop(now);self.phase='idle';self.note='cuRobo indisponível; controle parado'
     hold=JointState();hold.name=ARM;hold.position=self.q.astype(float).tolist();self.pub.publish(hold)
     print('MPC stopped:',self.mpc_error,flush=True);return
    if self.phase in ('home','test') and np.linalg.norm(self.tcp[0]-target[0])<.008 and angle(self.tcp[1],target[1])<math.radians(2):
     self.phase='idle';self.note='Movimento concluído'
   else:
    if self.hold_q is None or self.hold_phase!=self.phase:self.hold_q=self.q.copy()
    self.hold_phase=self.phase
    hold=JointState();hold.name=ARM;hold.position=self.hold_q.astype(float).tolist();self.pub.publish(hold)
   if self.phase=='active':self.update_gripper(now)
 def drive_mpc(self,p,q,publish=True):
  state=CuState.from_position(self.ta.to_device(self.q[None]),joint_names=self.jn)
  pose=Pose(self.ta.to_device([p.tolist()]),quaternion=self.ta.to_device([[q[3],q[0],q[1],q[2]]]))
  if self.goal_buffer is None:
   goal=Goal(current_state=state,goal_state=CuState.from_position(state.position.clone(),joint_names=self.jn),goal_pose=pose)
   self.goal_buffer=self.mpc.setup_solve_single(goal,1)
  else:self.goal_buffer.goal_pose.copy_(pose)
  self.mpc.update_goal(self.goal_buffer);start=time.monotonic();result=self.mpc.step(state,max_attempts=2)
  self.solver_ms=1000*(time.monotonic()-start)
  self.mpc_metrics={'feasible':bool(torch.all(result.metrics.feasible).item())}
  for key in ('position_error','rotation_error','pose_error'):
   value=getattr(result.metrics,key,None)
   if value is not None:self.mpc_metrics[key]=float(value.max().detach().cpu().item())
  command=result.js_action.get_ordered_joint_state(self.jn)
  cmd=command.position.reshape(-1,len(self.jn))[0].detach().cpu().numpy()
  if not np.all(np.isfinite(cmd)):return
  msg=JointState();msg.header.stamp=self.get_clock().now().to_msg();msg.name=list(self.jn);msg.position=[float(v) for v in cmd]
  if publish:
   if not self.mpc_metrics['feasible']:
    msg.position=self.q.astype(float).tolist();self.note='Alvo limitado pelas restrições do planejador'
   self.pub.publish(msg);self.steps+=1
 def update_gripper(self,now):
  if self.grip_stamp==self.stamp:return
  self.grip_stamp=self.stamp;g=self.gesture
  if g not in ('Open_Palm','Closed_Fist'):self.grip_candidate=None;self.grip_frames=0;return
  if g!=self.grip_candidate:self.grip_candidate=g;self.grip_since=now;self.grip_frames=1
  else:self.grip_frames+=1
  if self.grip_frames>=3 and now-self.grip_since>=.35 and now-self.grip_time>=1. and g!=self.grip_last:
   self.grip_pub.publish(Float32(data=.04 if g=='Open_Palm' else 0.));self.grip_last=g;self.grip_time=now
 def status(self):
  xyz,rpy=None,None
  if self.anchor is not None and self.sample is not None:
   xyz=((self.sample[0]-self.anchor[2])*100).tolist()
   r=quaternion_matrix(self.sample[1])@quaternion_matrix(self.anchor[3]).T
   rpy=np.degrees([math.atan2(r[2,1],r[2,2]),math.atan2(-r[2,0],math.hypot(r[2,1],r[2,2])),math.atan2(r[1,0],r[0,0])]).tolist()
  performance={}
  try:performance=json.loads((HERE/'run/performance.json').read_text())
  except (OSError,ValueError):pass
  return dict(command_tcp=None if self.cmd is None else self.cmd[0].tolist(),mpc_metrics=self.mpc_metrics,mpc_warmed=self.mpc_warmed,warmup_ms=self.warmup_ms,mpc_ready=self.mpc_ready,mpc_error=self.mpc_error,simulation_performance=performance,relative_xyz_cm=xyz,relative_rpy_deg=rpy,simulation_only=True,domain=188,calibration=self.gate.progress(),last_control=self.last_control,control_count=self.control_count,tracking=self.tracking_info,hand_fps=round((len(self.sample_times)-1)/(self.sample_times[-1]-self.sample_times[0]),1) if len(self.sample_times)>1 and time.monotonic()-self.sample_at<.65 else 0.,phase=self.phase,note=self.note,scale=self.scale,steps=self.steps,solver_ms=round(self.solver_ms,1),
   tcp=None if self.tcp is None else self.tcp[0].tolist(),tcp_quaternion=None if self.tcp is None else self.tcp[1].tolist(),hand_age_ms=None if self.sample is None else round(1000*(time.monotonic()-self.sample_at)),
   gesture=self.gesture,gripper=self.grip_last,finger_width_m=self.finger_width,overlay_age_ms=None if not self.overlay_at else round(1000*(time.monotonic()-self.overlay_at)),sim_age_ms=None if self.tcp is None else round(1000*(time.monotonic()-self.tcp_at)))
def handler(node):
 class H(BaseHTTPRequestHandler):
  def log_message(self,*a):pass
  def send(self,code,body,mime):
   self.send_response(code);self.send_header('Content-Type',mime);self.send_header('Cache-Control','no-store');self.end_headers();self.wfile.write(body)
  def do_GET(self):
   path=self.path.split('?')[0]
   if path=='/':return self.send(200,(HERE/'dashboard.html').read_bytes(),'text/html; charset=utf-8')
   if path=='/state':return self.send(200,json.dumps(node.status(),ensure_ascii=False).encode(),'application/json')
   if path=='/operator.jpg' and node.overlay:return self.send(200,node.overlay,'image/jpeg')
   if path=='/view.jpg':
    p=HERE/'run/view.jpg'
    if p.exists():return self.send(200,p.read_bytes(),'image/jpeg')
   return self.send(404,b'Waiting for image','text/plain')
  def do_POST(self):
   if self.path!='/control' or self.headers.get('X-Sim-Control')!='1':return self.send(403,b'Forbidden','text/plain')
   origin=self.headers.get('Origin')
   if origin and origin.split('://')[-1]!=self.headers.get('Host'):return self.send(403,b'Origin','text/plain')
   try:
    d=json.loads(self.rfile.read(min(int(self.headers.get('Content-Length','0')),1024)));node.control(d['action'],d.get('value'));self.send(200,json.dumps(dict(ok=True,phase=node.phase,note=node.note)).encode(),'application/json')
   except (ValueError,KeyError,TypeError) as e:print('CONTROL rejected',str(e),flush=True);self.send(409,json.dumps({'error':str(e)}).encode(),'application/json')
 return H
rclpy.init(domain_id=188);node=Session()
for host in ('127.0.0.1','100.88.134.127'):
 server=ThreadingHTTPServer((host,8791),handler(node));threading.Thread(target=server.serve_forever,daemon=True).start()
try:rclpy.spin(node)
finally:node.destroy_node();rclpy.shutdown()
