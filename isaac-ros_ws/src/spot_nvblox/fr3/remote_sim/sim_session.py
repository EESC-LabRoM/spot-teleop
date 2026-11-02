"""Simulation-only cuRobo MPC, operator calibration, gripper and local/Tailscale viewer.
All robot interfaces live under /fr3_sim in DDS domain 188. No hardware driver is used.
"""
import json,math,os,sys,time,threading
from pathlib import Path
from http.server import ThreadingHTTPServer,BaseHTTPRequestHandler
from urllib.request import Request,urlopen
import numpy as np
import cv2
import torch
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState,Image
from std_msgs.msg import Empty,String,Float32
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
sys.path.insert(0,str(HERE.parent))
sys.path.insert(0,str(HERE.parents[4]/'spot-ros2_ws/src/arm_pose_estimator'))
from home_pose import HOME_Q, HOME_TCP_P, HOME_TCP_Q
from tracking_gate import TrackingGate
from manual_teaching import manual_target,ManualKinematics
from teleop_mapping import control_gesture, control_sample, map_target, mapping_settings, rotation_delta
from cartesian_review import target as cartesian_target, PROFILES
from teleop_mapping import _quat_mul
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
  self.manual_kinematics=ManualKinematics(HERE.parent/'fr3.urdf')
  self.bag_enabled=bool(os.environ.get('BAG_MODE'));self.replay_state=None;self.replay_error=None;self.replay_poll_at=0.;self.orientation_current=False
  self.position_gains,self.rotation_gain,self.tilt_gain=mapping_settings(
   [float(v) for v in os.environ.get('FR3_SIM_POSITION_GAINS','1,1,1').split(',')],
   float(os.environ.get('FR3_SIM_ROTATION_GAIN','1')),
   float(os.environ.get('FR3_SIM_TILT_GAIN','0')))
  self.mapping_mode='cartesian';self.review_profile='free';self.review_ik=None;self.ik_target=None;self.ik_joints=None
  self.lock=threading.RLock();self.q=None;self.q_at=0.;self.tcp=None;self.tcp_at=0.;self.sample=None;self.sample_at=0.;self.stamp=-1
  self.hold_q=None;self.hold_phase=None;self.gate=TrackingGate();self.sample_times=[];self.last_control=None;self.control_count=0;self.tracking_info={}
  self.phase='idle';self.note='Clique Start para calibrar';self.stable=[];self.start_deadline=0.;self.paused_at=None
  self.anchor=None;self.cmd=None;self.home=(HOME_TCP_P.copy(),HOME_TCP_Q.copy());self.scale=1.;self.gesture=None;self.grip_candidate=None;self.grip_frames=0;self.grip_since=0.;self.grip_last=None;self.grip_time=0.;self.grip_stamp=-1
  self.overlay_at=0.;self.finger_width=None;self.overlay=None;self.steps=0;self.solver_ms=0.;self.goal_buffer=None;self.mpc_ready=False;self.goal_pose=None
  self.pub=self.create_publisher(JointState,'/fr3_sim/joint_command',1)
  self.reset_pub=self.create_publisher(Empty,'/fr3_sim/reset',1)
  self.goal_pub=self.create_publisher(PoseStamped,'/fr3_sim/goal',1)
  self.grip_pub=self.create_publisher(Float32,'/fr3_sim/gripper_width',1)
  self.create_subscription(JointState,'/fr3_sim/joint_states',self.on_js,10)
  self.create_subscription(PoseStamped,'/fr3_sim/tcp',self.on_tcp,10)
  self.create_subscription(Empty,'/fr3_sim/reset',self.on_reset,10)
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
  self.review_world=world
  print('Loading simulation cuRobo MPC',flush=True)
  mc=MpcSolverConfig.load_from_robot_config(cfg,world,use_cuda_graph=True,use_cuda_graph_metrics=False,use_cuda_graph_full_step=False,
     self_collision_check=True,collision_checker_type=CollisionCheckerType.PRIMITIVE,use_mppi=True,use_lbfgs=False,use_es=False,store_rollouts=False,step_dt=.05)
  self.mpc=MpcSolver(mc);self.mpc.enable_cspace_cost(False);self.mpc.enable_pose_cost(True);self.ta=self.mpc.tensor_args;self.jn=self.mpc.rollout_fn.joint_names
  for r in (self.mpc.rollout_fn,self.mpc.solver.safety_rollout,*[o.rollout_fn for o in self.mpc.solver.optimizers]):
   r.goal_cost.weight[0]=180.
  self._init_cov=[opt.cov_action.clone() for opt in self.mpc.solver.optimizers]
  self._init_scale=[opt.scale_tril.clone() for opt in self.mpc.solver.optimizers]
  self.reset_cov=False
  self.load_cartesian_ik()
  self.mpc_ready=True
  print('Simulation controller ready; HTTP 8791; DDS 188; /fr3_sim only',flush=True)
 def reset_mpc_opt(self):
  self.reset_cov=True
 def on_js(self,m):
  d=dict(zip(m.name,m.position))
  if all(j in d for j in ('fr3_finger_joint1','fr3_finger_joint2')):self.finger_width=float(d['fr3_finger_joint1']+d['fr3_finger_joint2'])
  if all(j in d for j in ARM):self.q=np.array([d[j] for j in ARM],np.float32);self.q_at=time.monotonic()
 def on_tcp(self,m):
  p,q=m.pose.position,m.pose.orientation
  self.tcp=(np.array([p.x,p.y,p.z]),np.array([q.x,q.y,q.z,q.w]));self.tcp_at=time.monotonic()
 def on_reset(self,_m):
  with self.lock:
   now=time.monotonic();self.gate.stop(now);self.phase='idle';self.note='Simulação reiniciada em HOME'
   self.q=np.array(HOME_Q,np.float32);self.q_at=now
   self.tcp=(HOME_TCP_P.copy(),HOME_TCP_Q.copy());self.tcp_at=now
   self.hold_q=self.q.copy();self.hold_phase='idle';self.cmd=tuple(v.copy() for v in self.tcp)
   self.reset_mpc_opt()
   hold=JointState();hold.name=ARM;hold.position=self.hold_q.astype(float).tolist();self.pub.publish(hold)
 def on_sample(self,m):
  try:
   d=json.loads(m.data);parsed=control_sample(d,None if self.sample is None else self.sample[1])
   if parsed is None:return
   stamp,p,q,orientation_current=parsed;age=(time.time_ns()-stamp)*1e-9
   if stamp<=self.stamp or not 0<=age<.65:return
   self.orientation_current=orientation_current
   self.sample=(p,q);self.sample_at=time.monotonic()-age;self.stamp=stamp
   self.gate.observe(stamp,self.sample_at,p,q)
   self.sample_times=[v for v in self.sample_times if time.monotonic()-v<2.]+[time.monotonic()]
   self.gesture=control_gesture(d)
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
 def bag_request(self,action=None):
  request='http://127.0.0.1:8792/state' if action is None else Request(
   'http://127.0.0.1:8792/control',data=json.dumps(dict(action=action)).encode(),
   headers={'Content-Type':'application/json','X-Bag-Control':'1'})
  try:
   with urlopen(request,timeout=1) as response:data=json.load(response)
   if action is None:self.replay_state=data
   self.replay_error=None;return data
  except (OSError,ValueError) as error:
   self.replay_error=str(error);raise ValueError('Replay indisponível: '+str(error)) from error
 def control(self,action,value=None):
  with self.lock:
   self.last_control=dict(action=action,at=time.time());self.control_count+=1
   print('CONTROL',action,'phase',self.phase,flush=True)
   if action in ('start','home','rotate','test'):self.reset_mpc_opt()
   if action=='stop':
    self.gate.stop(time.monotonic());self.hold_q=None;self.phase='idle';self.note='Controle parado';self.paused_at=None;self.stable=[];self.anchor=None
    if self.bag_enabled:
     try:self.bag_request('pause')
     except ValueError:pass
    return
   if action=='speed':
    speed=float(value)
    if not math.isfinite(speed):raise ValueError('Velocidade inválida')
    self.scale=float(np.clip(speed,.05,1.));return
   if action=='mapping':
    if self.phase!='idle':raise ValueError('Clique Parar antes de mudar o mapeamento')
    mode=value.get('mode',self.mapping_mode);profile=value.get('profile',self.review_profile)
    if mode not in ('cartesian','legacy') or profile not in PROFILES:raise ValueError('Modo ou perfil desconhecido')
    self.mapping_mode,self.review_profile=mode,profile;self.ik_target=None
    self.position_gains,self.rotation_gain,self.tilt_gain=mapping_settings(
     value['position_gains'],float(value['rotation_gain']),float(value.get('tilt_gain',0.)))
    self.anchor=None;self.note='Mapeamento atualizado; clique Start para capturar nova referência';return
   if not self.mpc_ready:raise ValueError('cuRobo indisponível: '+str(self.mpc_error))
   if not self.mpc_warmed:raise ValueError('Preparando o controle; aguarde um instante')
   if self.tcp is None or time.monotonic()-self.tcp_at>.5:raise ValueError('Aguardando estado da simulação')
   if action in ('manual_begin','manual_home','manual_nudge'):
    if action=='manual_nudge' and self.phase!='manual':raise ValueError('Ative primeiro o controle manual')
    if action=='manual_nudge':
     target=manual_target(*self.manual_pose,value['axis'],value['amount'])
     if np.linalg.norm(target[0]-self.tcp[0])>.10:raise ValueError('Aguarde o robô alcançar o alvo antes de acumular mais passos')
     joints=self.manual_kinematics.solve(*target,self.manual_joints)
     self.manual_joints=joints;self.manual_pose=target;self.note='Você controla o robô simulado';return
    self.gate.stop(time.monotonic());self.anchor=None;self.hold_q=None
    if self.bag_enabled:
     try:self.bag_request('pause')
     except ValueError:pass
    self.reset_mpc_opt()
    if action=='manual_home':
     self.phase='resetting_home';self.note=self.gate.note='Reiniciando a simulação em HOME';self.reset_pub.publish(Empty());return
    self.phase='manual';self.note='Controle manual · mostre como o robô deve mover'
    self.manual_joints=self.q.astype(float).copy()
    self.manual_pose=tuple(v.copy() for v in self.tcp);self.cmd=tuple(v.copy() for v in self.tcp);return
   if action=='start':
    if self.bag_enabled:
     state=self.bag_request()
     if state['ended']:
      if state['mode']!='recorded':raise ValueError('Bag finalizada: reinicie o modo retrack para zerar o rastreador')
      self.gate.stop(time.monotonic());self.phase='idle';self.sample=None;self.anchor=None
      self.bag_request('restart')
     self.gate.start(time.monotonic());self.phase=self.gate.phase
     if self.phase=='calibrating':self.gate.note='Bag iniciada · aguardando referência estável da mão gravada'
     try:self.bag_request('play')
     except ValueError:
      self.gate.stop(time.monotonic());self.phase='idle';raise
    else:self.gate.start(time.monotonic())
    self.phase=self.gate.phase;self.note=self.gate.note;self.hold_q=None;return
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
  self.anchor_joint_q=self.q.copy()
  self.anchor=(*[v.copy() for v in self.tcp],*[v.copy() for v in self.sample]);self.cmd=tuple(v.copy() for v in self.tcp)
  self.phase='active';self.note=self.gate.note;self.paused_at=None;self.grip_candidate=None;self.grip_frames=0
 def tick(self):
  if not self.mpc_ready:return
  now=time.monotonic()
  if self.bag_enabled and now-self.replay_poll_at>=.5:
   self.replay_poll_at=now
   try:
    replay=self.bag_request()
    if replay['ended'] and self.phase in ('calibrating','active','paused'):
     self.gate.stop(now);self.phase='idle';self.note=self.gate.note='Bag finalizada · clique Repetir bag para rodar novamente';self.hold_q=None
   except ValueError:
    if self.phase in ('calibrating','active','paused'):
     self.gate.stop(now);self.phase='idle';self.note=self.gate.note='Replay sem conexão; controle parado'
  if self.q is None or self.tcp is None or now-self.q_at>.5 or now-self.tcp_at>.5:
   if self.phase in ('active','paused'):
    self.gate.pause(now,'Pausado: aguardando estado recente do simulador');self.phase=self.gate.phase;self.note=self.gate.note
   return
  if self.phase=='resetting_home':
   # GPU backends apply reset on the next physics step. Do not send the old
   # idle hold while waiting for the measured HOME acknowledgement.
   if np.linalg.norm(self.tcp[0]-HOME_TCP_P)<.003:
    self.phase='idle';self.hold_q=None;self.cmd=tuple(v.copy() for v in self.tcp);self.note='Home alcançada'
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
    target=(cartesian_target(ap,aq,p0,q0,p,q,self.position_gains,self.rotation_gain,self.review_profile) if self.mapping_mode=='cartesian' else map_target(ap,aq,p0,q0,p,q,self.position_gains,self.rotation_gain,self.tilt_gain))
   elif self.phase=='home':target=self.home
   elif self.phase=='test':target=self.test_pose
   elif self.phase=='manual':target=self.manual_pose
   else:target=tuple(v.copy() for v in self.tcp);self.cmd=target
   if self.phase in ('active','home','test','manual'):
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
   if self.phase in ('active','home','test','manual'):
    self.hold_q=None
    try:
     if self.phase=='manual':
      joints=self.q+np.clip(self.manual_joints-self.q,-.035,.035)
      command=JointState();command.header.stamp=self.get_clock().now().to_msg();command.name=ARM;command.position=joints.tolist()
      self.pub.publish(command);self.steps+=1
     elif self.phase=='active' and self.mapping_mode=='cartesian':self.drive_cartesian(gp,gq)
     else:self.drive_mpc(gp,gq)
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
  if self.reset_cov:
   self.reset_cov=False
   for opt,c,s in zip(self.mpc.solver.optimizers,self._init_cov,self._init_scale):
    opt.mean_action.zero_();opt.cov_action.copy_(c);opt.scale_tril.copy_(s)
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
 def load_cartesian_ik(self):
  if self.review_ik is None:
   from curobo.wrap.reacher.ik_solver import IKSolver,IKSolverConfig
   cfg=fr3_robot_cfg(str(HERE.parent/'fr3.urdf'))
   cfg['kinematics']['collision_link_names']=[n for n in cfg['kinematics']['collision_link_names'] if n!='fr3_link0']
   self.review_ik=IKSolver(IKSolverConfig.load_from_robot_config(cfg,self.review_world,collision_checker_type=CollisionCheckerType.PRIMITIVE,tensor_args=self.ta,num_seeds=8,position_threshold=.001,rotation_threshold=.01,use_cuda_graph=True,self_collision_check=True))
  seed=self.ta.to_device(np.array(HOME_Q,dtype=np.float32).reshape(1,7))
  goal=Pose(self.ta.to_device([HOME_TCP_P.tolist()]),quaternion=self.ta.to_device([[HOME_TCP_Q[3],*HOME_TCP_Q[:3]]]))
  self.review_ik.solve_single(goal,retract_config=seed,seed_config=seed.reshape(1,1,7).repeat(1,8,1),num_seeds=8)
 def drive_cartesian(self,p,q):
  # GPU IK targets avoid the MPC's centimetre-scale tracking offset in axis reviews.
  if self.review_profile in ('roll','neutral','gripper'):
   joints=self.anchor_joint_q.copy()
   if self.review_profile=='roll':
    aq=self.anchor[1];dq=_quat_mul(np.array([-aq[0],-aq[1],-aq[2],aq[3]]),q)
    roll=2*math.atan2(float(dq[2]),float(dq[3]))
    joints[6]=np.clip(joints[6]+roll,self.manual_kinematics.lower[6]+.001,self.manual_kinematics.upper[6]-.001)
   self.mpc_metrics=dict(feasible=True,solver='wrist_only' if self.review_profile=='roll' else 'joint_hold')
   delta=joints-self.q;largest=float(np.max(np.abs(delta)));joints=self.q+delta*min(1.,.12/max(largest,1e-9))
   msg=JointState();msg.header.stamp=self.get_clock().now().to_msg();msg.name=ARM;msg.position=joints.tolist();self.pub.publish(msg);self.steps+=1;return
  if self.review_ik is None:self.load_cartesian_ik()
  # Follow a short Cartesian step, rather than clipping each joint independently.
  # Independent joint saturation bent a straight TCP path into an arc.
  delta=np.asarray(p)-self.tcp[0];length=float(np.linalg.norm(delta))
  p=self.tcp[0]+delta*min(1.,.025/max(length,1e-9))
  q=toward(self.tcp[1],q,.12 if self.review_profile=='yaw' else .08)
  current=np.r_[p,q]
  if self.ik_target is None or np.linalg.norm(current-self.ik_target)>1e-5:
   seed=self.ta.to_device(self.q[None]);pose=Pose(self.ta.to_device([p.tolist()]),quaternion=self.ta.to_device([[q[3],q[0],q[1],q[2]]]))
   start=time.monotonic();result=self.review_ik.solve_single(pose,retract_config=seed,seed_config=seed.reshape(1,1,7).repeat(1,8,1),num_seeds=8)
   self.solver_ms=(time.monotonic()-start)*1000
   feasible=bool(result.success.reshape(-1)[0].item())
   self.mpc_metrics=dict(feasible=feasible,solver='gpu_ik',position_error=float(result.position_error.reshape(-1)[0].item()),rotation_error=float(result.rotation_error.reshape(-1)[0].item()))
   if not feasible:
    self.note='Alvo cartesiano não alcançável; segurando posição';return
   self.ik_joints=result.solution.reshape(-1,7)[0].detach().cpu().numpy();self.ik_target=current.copy()
  joint_step=.06 if self.review_profile=='pitch' else .12
  delta=self.ik_joints-self.q;largest=float(np.max(np.abs(delta)));joints=self.q+delta*min(1.,joint_step/max(largest,1e-9))
  msg=JointState();msg.header.stamp=self.get_clock().now().to_msg();msg.name=ARM;msg.position=joints.tolist();self.pub.publish(msg);self.steps+=1
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
  return dict(joint_positions=None if self.q is None else self.q.astype(float).tolist(),manual_target=None if self.phase!='manual' else dict(p=self.manual_pose[0].tolist(),q=self.manual_pose[1].tolist()),replay=self.replay_state,replay_error=self.replay_error,orientation_current=self.orientation_current,mapping=dict(position_gains=self.position_gains.tolist(),rotation_gain=self.rotation_gain,tilt_gain=self.tilt_gain,mode=self.mapping_mode,profile=self.review_profile,logical_axes=dict(X='horizontal',Y='height',Z='reach')),command_tcp=None if self.cmd is None else self.cmd[0].tolist(),mpc_metrics=self.mpc_metrics,mpc_warmed=self.mpc_warmed,warmup_ms=self.warmup_ms,mpc_ready=self.mpc_ready,mpc_error=self.mpc_error,simulation_performance=performance,relative_xyz_cm=xyz,relative_rpy_deg=rpy,simulation_only=True,domain=188,calibration=self.gate.progress(),last_control=self.last_control,control_count=self.control_count,tracking=self.tracking_info,hand_fps=round((len(self.sample_times)-1)/(self.sample_times[-1]-self.sample_times[0]),1) if len(self.sample_times)>1 and time.monotonic()-self.sample_at<.65 else 0.,phase=self.phase,note=self.note,scale=self.scale,steps=self.steps,solver_ms=round(self.solver_ms,1),
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
