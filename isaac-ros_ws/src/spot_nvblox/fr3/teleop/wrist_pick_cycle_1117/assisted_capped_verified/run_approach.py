"""Synthetic wrist approach at safe hover; measured arrival, all normal gates retained."""
import sys,json,time,math,pathlib,subprocess,socket,threading,os
os.environ["PYTHONPATH"]="/lab/fr3_zed_runtime:"+os.environ.get("PYTHONPATH", "")
import numpy as np,rclpy,tf2_ros
from std_srvs.srv import Trigger,SetBool
from std_msgs.msg import Bool
from controller_manager_msgs.srv import SwitchController
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory
out=pathlib.Path('/tmp/wrist_assisted_capped_verified_1117');rclpy.init();n=rclpy.create_node('wrist_drill_hover_trial');tf=tf2_ros.Buffer();tf2_ros.TransformListener(tf,n);box={};children=[];f=open(out/'hover_trace.jsonl','w',buffering=1)
def log(kind,**d):f.write(json.dumps(dict(kind=kind,t=time.time(),**d))+'\n')
def js(m):box['js']=dict(zip(m.name,m.position));box['joint_at']=time.monotonic()
n.create_subscription(JointState,'/joint_states',js,10)
n.create_subscription(PoseStamped,'/wrist_pick/grasp_target',lambda m:box.update(field_at=time.monotonic()),10)
n.create_subscription(Bool,'/teleop/engaged',lambda m:box.update(engaged=m.data,engaged_at=time.monotonic()),10)
n.create_subscription(JointTrajectory,'/fr3_arm_controller/joint_trajectory',lambda m:log('controller',q=list(m.points[0].positions)) if m.points else None,10)
n.create_subscription(PoseStamped,'/teleop/command_tool',lambda m:log('consumer',p=[m.pose.position.x,m.pose.position.y,m.pose.position.z],q=[m.pose.orientation.x,m.pose.orientation.y,m.pose.orientation.z,m.pose.orientation.w]),10)
def spin(sec):
 t=time.monotonic()
 while time.monotonic()-t<sec:rclpy.spin_once(n,timeout_sec=.01)
def call(name,typ,req):
 c=n.create_client(typ,name);assert c.wait_for_service(timeout_sec=3)
 try:
  for _ in range(3):
   fut=c.call_async(req);rclpy.spin_until_future_complete(n,fut,timeout_sec=3)
   if fut.done() and fut.result() is not None:
    r=fut.result();assert getattr(r,'success',getattr(r,'ok',True)),str(r);return r
   c.remove_pending_request(fut)
  raise RuntimeError('service timeout '+name)
 finally:n.destroy_client(c)
def active(v):r=SetBool.Request();r.data=v;call('/fr3_joystick_mpc/set_active',SetBool,r)
def sw(v):
 r=SwitchController.Request();r.activate_controllers=['fr3_arm_controller'] if v else [];r.deactivate_controllers=[] if v else ['fr3_arm_controller'];r.strictness=1;r.timeout.sec=4;call('/controller_manager/switch_controller',SwitchController,r)
def trigger(s):return call(s,Trigger,Trigger.Request())
def pose():
 t=tf.lookup_transform('fr3_link0','fr3_hand_tcp',rclpy.time.Time());age=(n.get_clock().now().nanoseconds-t.header.stamp.sec*10**9-t.header.stamp.nanosec)*1e-9;assert 0<=age<.3
 p=t.transform.translation;q=t.transform.rotation;return np.array([p.x,p.y,p.z]),np.array([q.x,q.y,q.z,q.w])
def launch(a,name):
 child=subprocess.Popen(a,stdout=open(out/name,'w'),stderr=subprocess.STDOUT);children.append(child)
sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);sending=True;delta=np.zeros(3)
def sender():
 while sending:
  ns=time.time_ns();d=dict(version=3,frame='fr3_link0',orientation_mode='full_6d',position_valid=True,orientation_valid=True,stamp_ns=ns,orientation_stamp_ns=ns,p=(1.315*delta).tolist(),q=[0.,0.,0.,1.],gesture='Open_Palm',gesture_score=1.)
  sock.sendto(json.dumps(d).encode(),('127.0.0.1',47984));time.sleep(.025)
threading.Thread(target=sender,daemon=True).start()
try:
 state=json.loads(subprocess.check_output(['python3','/tmp/wrist_ready_1117/state_probe.py'],text=True).strip().splitlines()[-1]);assert state['robot_mode']==1 and not any(state['current_errors'].values()) and not any(state['last_motion_errors'].values());assert state['inertia_load']['inertia']['m']==0
 ps=subprocess.check_output(['ps','-eo','args'],text=True);assert '/tmp/direct_topic_fwd.py' not in ps and '/lab/fw_rev/direct_goal_fwd.py' not in ps
 a=json.loads((out/'consumer_launch_args.json').read_text());a+=['-p','zed_gripper_enabled:=true','-p','auto_grasp_enabled:=false','-p','zed_calibration_port:=0','-p','linear_speed:=.10','-p','vertical_speed:=.10']
 launch(a,'hover_consumer.log');spin(2);trigger('/teleop/disengage');active(False)
 for _ in range(200):
  spin(.1)
  if time.monotonic()-box.get('field_at',-100)<.4:break
 else:raise RuntimeError('fresh live field target not received; no motion enabled')
 print('FRESH_FIELD_ACKNOWLEDGED',flush=True);sw(True)
 launch(['python3','-u','/lab/fw_rev/direct_goal_fwd.py'],'hover_goal.log');active(True)
 launch(['python3','-u','/tmp/direct_topic_fwd.py'],'hover_forwarder.log')
 ready=False
 for _ in range(140):
  spin(.1)
  try:
   s=json.loads(pathlib.Path('/tmp/wrist_forwarder_status.json').read_text())
   if time.time()-s['wall']<.5 and s['ready']:ready=True;break
  except (OSError,ValueError):pass
 assert ready,'fresh measured-seed readiness refused'
 requested=time.monotonic();trigger('/teleop/zed_start')
 for _ in range(160):
  spin(.1)
  if box.get('engaged') and box.get('engaged_at',0)>requested:break
 else:raise RuntimeError('engagement acknowledgement missing; no synthetic motion sent')
 print('ENGAGEMENT_ACKNOWLEDGED_BEFORE_WRIST_MOTION',flush=True)
 start,q0=pose();goal=np.array(json.loads((out/'hover_plan.json').read_text())['goal']);assert goal[2]>=.32
 print('WRIST_HOVER_APPROACH_ACTIVE',flush=True);log('start',p=start.tolist(),q=q0.tolist(),goal=goal.tolist());began=time.monotonic();arrived_since=None
 while time.monotonic()-began<40:
  t=time.monotonic()-began;u=min(1.,t/6.);smooth=u*u*(3-2*u);delta=(goal-start)*smooth;spin(.04);p,q=pose();err=float(np.linalg.norm(p-goal));log('measured',p=p.tolist(),q=q.tolist(),goal_error=err,desired=(start+delta).tolist())
  assert time.monotonic()-box.get('field_at',-100)<1.5,'field target source lost';assert p[2]>.25,'hover descent limit';assert np.linalg.norm(p-start)<.55,'hover envelope';assert all(c.poll() is None for c in children),'control process exited'
  if t>6 and err<.04:
   if arrived_since is None:arrived_since=time.monotonic()
   if time.monotonic()-arrived_since>1.5:break
  else:arrived_since=None
 else:raise RuntimeError('hover arrival not reached; stopped without contact')
 (out/'hover_result.json').write_text(json.dumps(dict(result='measured hover reached',p=p.tolist(),q=q.tolist(),goal=goal.tolist(),error_m=err),indent=2));print('WRIST_HOVER_REACHED',flush=True)
finally:
 sending=False
 for func in [lambda:trigger('/teleop/disengage'),lambda:active(False)]:
  try:func()
  except Exception as e:log('cleanup_error',error=str(e))
 for c in children:
  if c.poll() is None:c.send_signal(2)
 for c in children:
  try:c.wait(timeout=4)
  except subprocess.TimeoutExpired:c.terminate();c.wait(timeout=3)
 try:sw(False)
 except Exception as e:log('cleanup_error',error=str(e))
 f.close();n.destroy_node();rclpy.shutdown()
