"""Network-disabled GPU MPC + measured MuJoCo A/B. No ROS or hardware transport."""
import sys,os,json,time,copy,math,hashlib,shutil,xml.etree.ElementTree as ET
from pathlib import Path
R=Path('/checks/wrist_reach_fix_1117/results');R.mkdir(exist_ok=True)
OLD=Path('/checks/posture_sim_1117')
sys.path[:0]=[str(OLD/'deps'),str(OLD),'/sim']
PROFILE=os.environ.get('CONTROL_PROFILE','candidate')
sys.path.insert(0,'/checks/wrist_reach_fix_1117/'+PROFILE)
from consumer_kernel import Kernel
import numpy as np, torch, mujoco
from scipy.spatial.transform import Rotation
from mujoco_remote_arm import MujocoArm,HOME
from table_facing import TableFacing,table_tilt_deg
from curobo.util_file import load_yaml
from curobo.geom.types import WorldConfig
from curobo.geom.sdf.world import CollisionCheckerType
from curobo.types.state import JointState
from curobo.types.math import Pose
from curobo.rollout.rollout_base import Goal
from curobo.wrap.reacher.mpc import MpcSolver,MpcSolverConfig
mode,take=sys.argv[1:3];duration=float(sys.argv[3]) if len(sys.argv)>3 else 1e9
offhome=len(sys.argv)>4 and sys.argv[4]=='offhome'
out=R/(os.environ.get('EXPERIMENT',mode)+'_'+take+('_offhome' if offhome else ''));out.mkdir(exist_ok=True)
(out/'source').mkdir(exist_ok=True)
for source_name in ['consumer_kernel.py','fr3_input_teleop_wrist_1117.py','wrist_motion_filter.py','table_facing.py','motion_preference.py','wrist_yaw_bias.py','mpc_posture_1117.py','consumer_launch_args.json','fr3_wrist_posture.yml','mpc_override.yml']:
 shutil.copy2(Path('/checks/wrist_reach_fix_1117')/PROFILE/source_name,out/'source'/source_name)
torch.manual_seed(123);np.random.seed(123)
scene=R/'scene'
if not scene.exists():
 shutil.copytree('/sim/mujoco',scene)
 tree=ET.parse(scene/'workcell.xml');root=tree.getroot()
 for parent in root.iter():
  for child in list(parent):
   if child.tag=='body' and child.attrib.get('name') in ['drill','bottle','mug','box']:parent.remove(child)
 tree.write(scene/'workcell.xml')
arm=MujocoArm(xml_path=str(scene/'workcell.xml'))
if offhome:
 engage_q=np.array([.4,-.55,-.3,-2.1,.3,1.4,.8])
 for i,v in enumerate(engage_q):arm.data.qpos[arm.qpos_adr[i]]=v
 arm.set_targets(arm.arm_names,engage_q.tolist(),[0.]*7);mujoco.mj_forward(arm.model,arm.data)
else:engage_q=np.array(arm.joint_state()[1][:7])
cfg=copy.deepcopy(load_yaml('/config/fr3_vcap1.2_acc10.yml')['robot_cfg'])
override=copy.deepcopy(load_yaml('/config/mpc_override.yml'))
if mode!='baseline':
 cfg['kinematics']['cspace']['retract_config']=engage_q.tolist()+[.04,.04]
 cfg['kinematics']['cspace']['null_space_weight']=[3.,3.,3.,2.,1.,1.,1.,1.,1.]
 cfg['kinematics']['cspace']['cspace_distance_weight']=[2.,2.,2.,1.5,1.,1.,1.,1.,1.]
if os.environ.get('PROXIMAL_WEIGHT'):
 cfg['kinematics']['cspace']['null_space_weight']=[float(os.environ['PROXIMAL_WEIGHT'])]*4+[1.,1.,.2,1.,1.]
override['cost']['bound_cfg']['smooth_weight'][1]=float(os.environ.get('VELOCITY_COST','100'))
override['cost']['bound_cfg']['null_space_weight']=[float(os.environ.get('POSTURE_WEIGHT','100'))]
override['cost']['pose_cfg']['weight'][0]=float(os.environ.get('ROTATION_WEIGHT','200'))
override['cost']['pose_cfg']['weight'][1]=float(os.environ.get('POSITION_WEIGHT','300'))
if os.environ.get('TABLE_WEIGHT'):
 override['cost']['pose_cfg']['vec_weight']=[float(os.environ['TABLE_WEIGHT']),float(os.environ['TABLE_WEIGHT']),1.,1.,1.,1.]
(out/'robot_config.json').write_text(json.dumps(cfg,indent=2))
(out/'override.yml').write_text(__import__('yaml').safe_dump(override))
# Controlled table-only scene, same model/world for every variant.
world=WorldConfig.from_dict({'cuboid':{'table':{'dims':[1.4,1.1,.08],'pose':[.35,0.,-.04,1,0,0,0]}}})
mc=MpcSolverConfig.load_from_robot_config(cfg,world,use_cuda_graph=True,use_cuda_graph_metrics=False,
 use_cuda_graph_full_step=False,self_collision_check=True,collision_checker_type=CollisionCheckerType.PRIMITIVE,
 override_particle_file=str(out/'override.yml'),use_mppi=True,use_lbfgs=False,use_es=False,
 store_rollouts=False,step_dt=.02)
mpc=MpcSolver(mc);names=mpc.joint_names
def js():
 ns,q,v=arm.joint_state()
 state=JointState.from_position(torch.tensor([q[:7]],device='cuda',dtype=torch.float32),joint_names=names)
 state.velocity=torch.tensor([v[:7]],device='cuda',dtype=torch.float32);state.acceleration=torch.zeros_like(state.position);return state
initial=js();kin=mpc.rollout_fn.compute_kinematics(initial)
goal=Goal(current_state=initial,goal_state=initial.clone(),goal_pose=Pose(kin.ee_pos_seq.clone(),quaternion=kin.ee_quat_seq.clone()))
buffer=mpc.setup_solve_single(goal,1);mpc.update_goal(buffer)
mpc.step(initial,max_attempts=1)
p0,q0=arm.tcp_pose()
fk_p=kin.ee_pos_seq.detach().cpu().numpy().reshape(-1,3)[0]
fk_qw=kin.ee_quat_seq.detach().cpu().numpy().reshape(-1,4)[0]
fk_q=np.r_[fk_qw[1:],fk_qw[0]]
fk_tcp=fk_p+Rotation.from_quat(fk_q).apply([0,0,.1034])
assert np.linalg.norm(fk_tcp-p0)<.005,(fk_tcp,p0)
rows=[json.loads(x) for x in (Path('/checks/live_ready_1117/bag_validation/results')/take/'bridge.jsonl').read_text().splitlines()]
packets=[x for x in rows if 'p' in x.get('packet',{}) and x.get('age_ms',1e6)<350]
t0=packets[0]['wall_ns'];ts=np.array([(x['wall_ns']-t0)/1e9 for x in packets])
end=min(float(ts[-1])+1.,duration);idx=-1
intent_errors=[];workspace_clamped=0
kernel=Kernel('/checks/wrist_reach_fix_1117/'+PROFILE+'/fr3_input_teleop_wrist_1117.py',p0,q0,float(os.environ.get('SPEED_FACTOR','1')))
table_profile=TableFacing(q0)
qfiltered=q0.copy();omega=np.zeros(3);ref=engage_q.copy()
last_cmd=engage_q.copy();seed=engage_q.copy()
first_target=np.r_[p0,q0]
trace=(out/'trace.jsonl').open('w',buffering=1)
bad=0;contacts=set();maxvel=0.;maxacc=0.;lastv=np.zeros(7);feasible=0;times=[];Q=[];errors=[];angles=[];effective_angles=[];tilts=[];path=np.zeros(7)
try:
 for k in range(int(end/.02)+1):
  simt=k*.02
  packet=None;receipt=None
  while idx+1<len(ts) and ts[idx+1]<=simt:
   idx+=1;packet=packets[idx]['packet'];receipt=float(ts[idx])-packets[idx]['age_ms']/1000.
  measured_p,measured_q=arm.tcp_pose()
  tp,tq=kernel.step(simt,packet,receipt,measured_p,measured_q)
  raw=np.r_[tp,tq]
  intent=getattr(kernel.n,'unclamped_intent_p',tp)
  workspace_clamped+=int(np.any(intent<kernel.n.box_lo) or np.any(intent>kernel.n.box_hi))
  if mode=='tabletop':tq=table_profile.step(tq,.02)
  if mode=='bounded':
   diff=(Rotation.from_quat(tq)*Rotation.from_quat(qfiltered).inv()).as_rotvec()
   desired=diff/.02;speed=np.linalg.norm(desired);limit=math.radians(22.5)
   if speed>limit:desired*=limit/speed
   delta=desired-omega;dn=np.linalg.norm(delta);alimit=math.radians(90)*.02
   if dn>alimit:delta*=alimit/dn
   omega+=delta
   if np.dot(omega*.02,diff)>0 and np.linalg.norm(omega*.02)>np.linalg.norm(diff):omega=diff/.02
   qfiltered=(Rotation.from_rotvec(omega*.02)*Rotation.from_quat(qfiltered)).as_quat();tq=qfiltered
  if mode in ('bounded','continuous','tabletop'):
   # Slow recent-posture reference: never an instantaneous re-anchor or branch jump.
   measured=js().position.detach().cpu().numpy()[0]
   ref+=np.clip((measured-ref)*.02/2.,-math.radians(5)*.02,math.radians(5)*.02)
   for rollout in [o.rollout_fn for o in mpc.solver.optimizers]+[mpc.solver.safety_rollout,mpc.rollout_fn]:
    rollout.dynamics_model.retract_config.copy_(torch.tensor(ref,device='cuda',dtype=torch.float32).reshape_as(rollout.dynamics_model.retract_config))
   if buffer.retract_state is not None:buffer.retract_state.copy_(torch.tensor(ref,device='cuda',dtype=torch.float32).reshape_as(buffer.retract_state))
  hand=tp-Rotation.from_quat(tq).apply([0,0,.1034])
  buffer.goal_pose.position.copy_(torch.tensor([hand.tolist()],device='cuda',dtype=torch.float32))
  buffer.goal_pose.quaternion.copy_(torch.tensor([[tq[3],*tq[:3]]],device='cuda',dtype=torch.float32))
  if os.environ.get('WRIST_BIAS')=='1':
   from wrist_yaw_bias import wrist_yaw_reference
   current=js();measured=current.position.detach().cpu().numpy()[0]
   kin_now=mpc.rollout_fn.compute_kinematics(current)
   quat_now=kin_now.ee_quat_seq.detach().cpu().numpy().reshape(-1,4)[0]
   limits=mpc.rollout_fn.dynamics_model.joint_limits.position.detach().cpu().numpy()
   preferred=ref.copy();preferred[6]=wrist_yaw_reference(measured[6],quat_now,[tq[3],*tq[:3]],float(limits[0,6]),float(limits[1,6]))
   for rollout in [o.rollout_fn for o in mpc.solver.optimizers]+[mpc.solver.safety_rollout,mpc.rollout_fn]:
    rollout.dynamics_model.retract_config.copy_(torch.tensor(preferred,device='cuda',dtype=torch.float32).reshape_as(rollout.dynamics_model.retract_config))
   if buffer.retract_state is not None:buffer.retract_state.copy_(torch.tensor(preferred,device='cuda',dtype=torch.float32).reshape_as(buffer.retract_state))
  if os.environ.get('DYNAMIC_POSTURE')=='1':
   from motion_preference import MotionPreference,apply_preference
   if k==0:preference=MotionPreference()
   apply_preference(mpc,preference.update(hand,.02))
  mpc.update_goal(buffer);start=time.perf_counter();result=mpc.step(js(),max_attempts=1);times.append(time.perf_counter()-start)
  ok=bool(torch.all(result.metrics.feasible).item());feasible+=int(ok)
  cmd=result.action.position.detach().cpu().numpy().reshape(-1,7)[0]
  vel=result.action.velocity.detach().cpu().numpy().reshape(-1,7)[0]
  if not np.all(np.isfinite(cmd)):raise RuntimeError('Nonfinite MPC output')
  if not ok:bad+=1;cmd=last_cmd.copy();vel=np.zeros(7)
  # Identical deployed startup blend + slew gate in both variants.
  u=min(1.,simt);alpha=10*u**3-15*u**4+6*u**5;rate=30*u**2-60*u**3+30*u**4
  vel=alpha*vel+rate*(cmd-seed);cmd=seed+alpha*(cmd-seed)
  cmd=last_cmd+np.clip(cmd-last_cmd,-.03,.03)
  arm.set_targets(names,cmd.tolist(),vel.tolist());last_cmd=cmd.copy()
  for sub in range(4):
   arm.step()
   for contact in arm.data.contact[:arm.data.ncon]:
    labels=tuple(mujoco.mj_id2name(arm.model,mujoco.mjtObj.mjOBJ_BODY,int(arm.model.geom_bodyid[g])) for g in [contact.geom1,contact.geom2])
    moving=[bool(x and x.startswith('fr3_') and x!='fr3_link0') for x in labels]
    if any(moving):contacts.add(labels)
  _,q,v=arm.joint_state();q=np.array(q[:7]);v=np.array(v[:7]);p,quat=arm.tcp_pose()
  if Q:path+=np.abs(q-Q[-1])
  Q.append(q);maxvel=max(maxvel,float(np.max(np.abs(v))));maxacc=max(maxacc,float(np.max(np.abs(v-lastv)))/.02);lastv=v
  intent_errors.append(float(np.linalg.norm(intent-p)))
  pe=float(np.linalg.norm(raw[:3]-p));ae=float(np.linalg.norm((Rotation.from_quat(raw[3:7])*Rotation.from_quat(quat).inv()).as_rotvec())*180/np.pi)
  ee=float(np.linalg.norm((Rotation.from_quat(tq)*Rotation.from_quat(quat).inv()).as_rotvec())*180/np.pi)
  tilt=table_tilt_deg(quat);errors.append(pe);angles.append(ae);effective_angles.append(ee);tilts.append(tilt)
  links={n:(arm.data.xpos[mujoco.mj_name2id(arm.model,mujoco.mjtObj.mjOBJ_BODY,n)]-arm.data.xpos[arm.base_body_id]).tolist() for n in ['fr3_link0','fr3_link1','fr3_link2','fr3_link3','fr3_link4','fr3_link5','fr3_link6','fr3_link7','fr3_hand']}
  trace.write(json.dumps(dict(links=links,t=simt,q=q.tolist(),v=v.tolist(),tcp=p.tolist(),quat=quat.tolist(),operator_target=raw.tolist(),target=np.r_[tp,tq].tolist(),table_tilt_deg=tilt,effective_orientation_error_deg=ee,effective_q=tq.tolist(),feasible=ok,position_error=pe,orientation_error_deg=ae))+'\n')
 finally_dummy=False
finally:
 trace.close();arm.close()
summary=dict(mode=mode,take=take,sim_seconds=end,steps=len(Q),feasible_fraction=feasible/max(1,len(Q)),infeasible_steps=bad,
 q_span_deg=(np.ptp(Q,axis=0)*180/np.pi).tolist(),joint_travel_deg=(path*180/np.pi).tolist(),
 proximal_travel_deg=float(sum(path[:4])*180/np.pi),max_joint_speed=maxvel,max_joint_accel=maxacc,
 tcp_error_m=dict(zip(['p50','p95','max'],np.percentile(errors,[50,95,100]).tolist())),
 orientation_error_deg=dict(zip(['p50','p95','max'],np.percentile(angles,[50,95,100]).tolist())),
 effective_orientation_error_deg=dict(zip(['p50','p95','max'],np.percentile(effective_angles,[50,95,100]).tolist())),
 table_tilt_deg=dict(zip(['p50','p95','max'],np.percentile(tilts,[50,95,100]).tolist())),
 contacts=[list(x) for x in sorted(contacts)],solver_p95_ms=float(np.percentile(times,95)*1000),
 fk_initial_error_m=float(np.linalg.norm(fk_tcp-p0)),translation_gain=kernel.n.zed_scale,workspace_clamped_steps=workspace_clamped,intent_error_m=dict(zip(['p50','p95','max'],np.percentile(intent_errors,[50,95,100]).tolist())),consumer_start_s=kernel.started,consumer_step_rejections=kernel.resets,consumer_active_frames=kernel.frames,scope='Recorded current tracker packets into exact staged consumer mapping/follower; native MPC; MuJoCo feedback; controlled table world; no ROS transport or raw retracking')
(out/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary),flush=True)
