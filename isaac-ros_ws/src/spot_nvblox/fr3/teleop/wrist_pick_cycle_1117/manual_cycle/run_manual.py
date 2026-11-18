"""Bounded synthetic hand cycle through production wrist consumer and guarded MPC/JTC."""
import os,sys,json,time,math,pathlib,subprocess,socket,threading
os.environ['PYTHONPATH']='/lab/fr3_zed_runtime:'+os.environ.get('PYTHONPATH','')
import numpy as np,rclpy,tf2_ros
from scipy.spatial.transform import Rotation as R
from std_srvs.srv import Trigger,SetBool
from std_msgs.msg import Bool,String
from controller_manager_msgs.srv import SwitchController
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory
from visualization_msgs.msg import Marker
from action_msgs.msg import GoalStatusArray
from franka_msgs.srv import SetLoad
from franka_msgs.action import Grasp
out=pathlib.Path('/tmp/wrist_manual_cycle_1117');rclpy.init();n=rclpy.create_node('wrist_manual_drill_cycle');tf=tf2_ros.Buffer();tf2_ros.TransformListener(tf,n);box={};children=[];trace=open(out/'trace.jsonl','w',buffering=1);config=json.loads((out/'plan.json').read_text());held=False;loaded=False;complete=False

def log(kind,**v):trace.write(json.dumps(dict(kind=kind,t=time.time(),**v))+'\n')
def joint(m):
 box.setdefault('js',{}).update(dict(zip(m.name,m.position)))
 if all(k in m.name for k in ['fr3_finger_joint1','fr3_finger_joint2']):box['width_at']=time.monotonic()
def command(m):box['command']=(np.array([m.pose.position.x,m.pose.position.y,m.pose.position.z]),np.array([m.pose.orientation.x,m.pose.orientation.y,m.pose.orientation.z,m.pose.orientation.w]));log('consumer',p=box['command'][0].tolist(),q=box['command'][1].tolist())
n.create_subscription(JointState,'/joint_states',joint,10)
n.create_subscription(PoseStamped,'/teleop/command_tool',command,10)
n.create_subscription(Bool,'/teleop/engaged',lambda m:box.update(engaged=m.data,engaged_at=time.monotonic()),10)
n.create_subscription(JointTrajectory,'/fr3_arm_controller/joint_trajectory',lambda m:log('controller',q=list(m.points[0].positions)) if m.points else None,10)
n.create_subscription(GoalStatusArray,'/franka_gripper/grasp/_action/status',lambda m:box.update(grasp_status=m),10)
n.create_subscription(String,'/wrist_pick/held_status',lambda m:box.update(held_status=json.loads(m.data)),10)
field=n.create_publisher(PoseStamped,'/wrist_pick/grasp_target',10);clear=n.create_publisher(Marker,'/wrist_pick/clear_box',10)
def publish_plan():
 # A bounded, explicitly frozen commanded grasp plan; not re-stamped perception.
 if time.monotonic()-began>180:return
 m=PoseStamped();m.header.stamp=n.get_clock().now().to_msg();m.header.frame_id='fr3_link0';m.pose.position.x,m.pose.position.y,m.pose.position.z=config['hand_position'];m.pose.orientation.x,m.pose.orientation.y,m.pose.orientation.z,m.pose.orientation.w=config['hand_quat_xyzw'];field.publish(m)
 b=Marker();b.header=m.header;b.ns='approved_static_drill_plan';b.id=1;b.type=Marker.CUBE;b.action=Marker.ADD;b.pose.orientation.w=1.;b.pose.position.x,b.pose.position.y,b.pose.position.z=config['box_center'];b.scale.x,b.scale.y,b.scale.z=config['box_size'];clear.publish(b)
began=time.monotonic();n.create_timer(.1,publish_plan)
def spin(sec):
 t=time.monotonic()
 while time.monotonic()-t<sec:rclpy.spin_once(n,timeout_sec=.01)
def call(name,typ,req,timeout=5):
 c=n.create_client(typ,name);assert c.wait_for_service(timeout_sec=20),'missing service '+name
 try:
  f=c.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=timeout);assert f.done() and f.result() is not None,'service timeout '+name;r=f.result();assert getattr(r,'success',getattr(r,'ok',True)),str(r);return r
 finally:n.destroy_client(c)
def trigger(name):return call(name,Trigger,Trigger.Request())
def active(v):r=SetBool.Request();r.data=v;call('/fr3_joystick_mpc/set_active',SetBool,r)
def switch(v):
 r=SwitchController.Request();r.activate_controllers=['fr3_arm_controller'] if v else [];r.deactivate_controllers=[] if v else ['fr3_arm_controller'];r.strictness=1;r.timeout.sec=4;call('/controller_manager/switch_controller',SwitchController,r)
def load(mass):
 global loaded
 r=SetLoad.Request();r.mass=mass;r.center_of_mass=[0.,0.,.1] if mass else [0.,0.,0.];r.load_inertia=[.005,0.,0.,0.,.005,0.,0.,0.,.002] if mass else [0.]*9;call('/service_server/set_load',SetLoad,r);loaded=bool(mass);log('payload',mass=mass)
def width():
 assert time.monotonic()-box.get('width_at',-100)<.5,'gripper width stale';return sum(box['js'][k] for k in ['fr3_finger_joint1','fr3_finger_joint2'])
def pose():
 a=tf.lookup_transform('fr3_link0','fr3_hand_tcp',rclpy.time.Time());age=(n.get_clock().now().nanoseconds-a.header.stamp.sec*10**9-a.header.stamp.nanosec)*1e-9;assert 0<=age<.3,'stale robot TF';t=a.transform;return np.array([t.translation.x,t.translation.y,t.translation.z]),np.array([t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w])
def state():return json.loads(subprocess.check_output(['python3','/tmp/wrist_ready_1117/state_probe.py'],text=True).strip().splitlines()[-1])
def check():
 p,q=pose();assert p[2]>.095,'TCP floor';assert p[2]<.55 and np.linalg.norm(p-anchor)<.45,'motion envelope';assert all(c.poll() is None for c in children),'controller process exited'
 if held:assert .010<width()<.072,'object lost/width invalid'
 log('measured',p=p.tolist(),q=q.tolist(),width=width(),held=held);return p,q
def launch(args,name):children.append(subprocess.Popen(args,stdout=open(out/name,'w'),stderr=subprocess.STDOUT))
sending=True;delta=np.zeros(3);handq=[0.,0.,0.,1.];gesture='Open_Palm';sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
def sender():
 while sending:
  stamp=time.time_ns();d=dict(version=3,frame='fr3_link0',orientation_mode='full_6d',position_valid=True,orientation_valid=True,stamp_ns=stamp,orientation_stamp_ns=stamp,p=(delta*1.315).tolist(),q=handq,gesture=gesture,gesture_score=1.);sock.sendto(json.dumps(d).encode(),('127.0.0.1',47984));time.sleep(.025)
threading.Thread(target=sender,daemon=True).start()
def move(goal,label,seconds=5,tolerance=.035,limit=22):
 global delta,handq
 goal=np.array(goal);origin=anchor+delta;start=time.monotonic();stable=None;previous=None;log('stage',stage=label,operator_goal=goal.tolist());print('STAGE '+label,flush=True)
 while time.monotonic()-start<limit:
  dt=time.monotonic()-start;u=min(1.,dt/seconds);delta=(origin-anchor)+(goal-origin)*(u*u*(3-2*u));spin(.04);p,q=check()
  if dt>seconds and np.linalg.norm(p-goal)<tolerance:
   if stable is None:stable=time.monotonic()
   if time.monotonic()-stable>.8:return p,q
  else:stable=None
 raise RuntimeError(label+' measured arrival not reached')
def close_result(after):
 deadline=time.monotonic()+10
 while time.monotonic()<deadline:
  spin(.05);check()
  for s in getattr(box.get('grasp_status'),'status_list',[]):
   at=s.goal_info.stamp.sec+s.goal_info.stamp.nanosec*1e-9
   if at>=after and s.status in [4,5,6]:
    request=Grasp.Impl.GetResultService.Request();request.goal_id=s.goal_info.goal_id
    r=call('/franka_gripper/grasp/_action/get_result',Grasp.Impl.GetResultService,request)
    log('close_result',status=r.status,success=r.result.success,error=r.result.error,width=width());assert r.status==4 and r.result.success and .012<width()<.070,'grasp failed or empty';return
 raise RuntimeError('manual close result timeout')
try:
 s=state();assert s['robot_mode']==1 and not any(s['current_errors'].values()) and not any(s['last_motion_errors'].values());assert s['inertia_load']['inertia']['m']==0
 ps=subprocess.check_output(['ps','-eo','args'],text=True);assert '/tmp/direct_topic_fwd.py' not in ps and '/lab/fw_rev/direct_goal_fwd.py' not in ps
 a=json.loads((out/'consumer_launch_args.json').read_text());a+=['-p','zed_gripper_enabled:=true','-p','auto_grasp_enabled:=false','-p','zed_calibration_port:=0','-p','workspace_min:=[0.2,-0.55,0.10]','-p','linear_speed:=.05','-p','vertical_speed:=.03','-p','grasp_width:=.03','-p','grasp_force:=60.0','-p','gripper_speed:=.03','-p','grasp_epsilon:=.05']
 launch(a,'consumer.log');spin(2);trigger('/teleop/disengage');active(False);switch(True);launch(['python3','-u','/lab/fw_rev/direct_goal_fwd.py'],'goal.log');active(True);launch(['python3','-u','/tmp/direct_topic_fwd.py'],'forwarder.log')
 for _ in range(180):
  spin(.1)
  try:
   v=json.loads(pathlib.Path('/tmp/wrist_forwarder_status.json').read_text())
   if time.time()-v['wall']<.5 and v['ready']:break
  except (OSError,ValueError):pass
 else:raise RuntimeError('measured preview seed not ready')
 requested=time.monotonic();trigger('/teleop/zed_start')
 for _ in range(160):
  spin(.1)
  if box.get('engaged') and box.get('engaged_at',0)>requested:break
 else:raise RuntimeError('engagement acknowledgement missing')
 anchor,q=pose();assert .25<anchor[2]<.4;print('ENGAGED_MANUAL_SEQUENCE',flush=True);spin(1);assert width()>.075
 # Offset only the private plan, as physically requested, not global calibration.
 p,q=move([config['contact_tcp'][0],config['contact_tcp'][1],.30],'right_alignment',seconds=4)
 yaw=math.atan2(R.from_quat(config['hand_quat_xyzw']).as_matrix()[1,0],R.from_quat(config['hand_quat_xyzw']).as_matrix()[0,0]);start=time.monotonic()
 while time.monotonic()-start<12:
  spin(.04);p,q=check();cq=box.get('command',(p,q))[1];angle=math.atan2(R.from_quat(cq).as_matrix()[1,0],R.from_quat(cq).as_matrix()[0,0]);err=math.atan2(math.sin(yaw-angle),math.cos(yaw-angle))
  if abs(err)<math.radians(3):handq=[0.,0.,0.,1.];break
  a=math.copysign(math.radians(20),err);handq=[math.sin(a/2),0.,0.,math.cos(a/2)]
 else:raise RuntimeError('yaw alignment timeout')
 spin(.7)
 for z,label in [(.22,'approach'),(.17,'precontact'),(.15,'contact')]:p,q=move([config['contact_tcp'][0],config['contact_tcp'][1],z],label,seconds=5)
 assert .110<p[2]<.158 and np.linalg.norm(p[:2]-np.array(config['contact_tcp'])[:2])<.025,'contact pose outside validated envelope';grasp_pose=p.copy();print('CLOSING_THROUGH_HAND_GESTURE',flush=True);closed_at=time.time();gesture='Closed_Fist';close_result(closed_at);held=True;load(1.3);spin(.5)
 p,q=move([config['contact_tcp'][0],config['contact_tcp'][1],.23],'lift',seconds=6,tolerance=.035)
 assert p[2]-grasp_pose[2]>.045,'lift height not reached';spin(1)
 hs=box.get('held_status',{});assert time.time()-hs.get('wall',0)<1.5 and hs.get('attached'),'held-object collision attachment not confirmed'
 # The native attachment should now be active; status is checked from log externally.
 p,q=move([config['contact_tcp'][0],config['contact_tcp'][1],.15],'place',seconds=6,tolerance=.025)
 assert np.linalg.norm(p-grasp_pose)<.015,'place did not return to grasp pose';gesture='Open_Palm';start=time.monotonic()
 while time.monotonic()-start<7:
  spin(.05)
  if width()>.075:break
 else:raise RuntimeError('open width not reached')
 held=False;load(0.);spin(.4);move([config['contact_tcp'][0],config['contact_tcp'][1],.30],'retreat',seconds=6);complete=True;(out/'result.json').write_text(json.dumps(dict(success=True,grasp_pose=grasp_pose.tolist(),final_tcp=pose()[0].tolist()),indent=2));print('MANUAL_CYCLE_COMPLETE',flush=True)
except Exception as e:
 log('failure',error=str(e),held=held,loaded=loaded);print('FAILED '+str(e),flush=True);raise
finally:
 sending=False
 for fn in [lambda:trigger('/teleop/disengage'),lambda:active(False)]:
  try:fn()
  except Exception as e:log('cleanup_error',error=str(e))
 for c in children:
  if c.poll() is None:c.send_signal(2)
 for c in children:
  try:c.wait(timeout=4)
  except subprocess.TimeoutExpired:c.terminate();c.wait(timeout=3)
 if not held:
  try:switch(False)
  except Exception as e:log('cleanup_error',error=str(e))
 else:print('HOLDING_OBJECT_JTC_LEFT_ACTIVE_DO_NOT_OPEN',flush=True)
 trace.close();n.destroy_node();rclpy.shutdown()
