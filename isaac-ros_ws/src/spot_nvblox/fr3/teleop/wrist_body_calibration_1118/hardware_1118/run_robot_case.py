"""Finite supervised 3D-only bag input via real consumer/MPC/gated JTC. No recovery."""
import json,math,os,pathlib,signal,socket,subprocess,sys,time
import numpy as np,rclpy,tf2_ros
from std_msgs.msg import Bool
from std_srvs.srv import Trigger,SetBool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory
from franka_msgs.msg import FrankaRobotState
from controller_manager_msgs.srv import SwitchController
case=sys.argv[1]; assert case in ['z06','z12']
root=pathlib.Path('/lab/wrist_bag_1118'); out=root/case;out.mkdir(exist_ok=False)
f=(out/'trace.jsonl').open('w',buffering=1)
def log(kind,**kw):f.write(json.dumps(dict(kind=kind,wall=time.time(),**kw),allow_nan=False)+'\n')
rclpy.init();n=rclpy.create_node('wrist_bag_bounded_1118');buf=tf2_ros.Buffer();listener=tf2_ros.TransformListener(buf,n);box={};children=[]
def js(m):
 names=[f'fr3_joint{i}' for i in range(1,8)];d=dict(zip(m.name,m.position));v=dict(zip(m.name,m.velocity))
 if all(x in d for x in names):
  box.update(q=[d[x] for x in names],v=[v.get(x,0.) for x in names],js_at=time.monotonic(),js_stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9)
  log('joints',names=names,q=box['q'],v=box['v'])
def state(m):
 box.update(mode=m.robot_mode,state_at=time.monotonic(),errors=[name for name in m.current_errors.get_fields_and_field_types() if getattr(m.current_errors,name)])
n.create_subscription(JointState,'/joint_states',js,10)
n.create_subscription(FrankaRobotState,'/franka_robot_state_broadcaster/robot_state',state,rclpy.qos.qos_profile_sensor_data)
n.create_subscription(Bool,'/teleop/engaged',lambda m:box.update(engaged=m.data,engaged_at=time.monotonic()),10)
n.create_subscription(PoseStamped,'/teleop/command_tool',lambda m:log('command',p=[m.pose.position.x,m.pose.position.y,m.pose.position.z],q=[m.pose.orientation.x,m.pose.orientation.y,m.pose.orientation.z,m.pose.orientation.w]),10)
n.create_subscription(JointTrajectory,'/fr3_arm_controller/joint_trajectory',lambda m:log('forwarded',q=list(m.points[0].positions),v=list(m.points[0].velocities)) if m.points else None,10)
def spin(sec):
 end=time.monotonic()+sec
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.01)
def rpc(name,typ,req):
 c=n.create_client(typ,name)
 try:
  assert c.wait_for_service(timeout_sec=4),name
  fut=c.call_async(req);rclpy.spin_until_future_complete(n,fut,timeout_sec=6)
  assert fut.done() and fut.result() is not None,name+' timeout'
  res=fut.result(); assert getattr(res,'success',getattr(res,'ok',True)),str(res)
  return res
 finally:n.destroy_client(c)
def active(v):
 r=SetBool.Request();r.data=v;rpc('/fr3_joystick_mpc/set_active',SetBool,r)
def switch(v):
 r=SwitchController.Request();r.activate_controllers=['fr3_arm_controller'] if v else [];r.deactivate_controllers=[] if v else ['fr3_arm_controller'];r.strictness=1;r.timeout.sec=4;rpc('/controller_manager/switch_controller',SwitchController,r)
def trigger(name):return rpc(name,Trigger,Trigger.Request())
def launch(argv,name):
 p=subprocess.Popen(argv,stdout=open(out/name,'w'),stderr=subprocess.STDOUT);children.append(p);return p
def pose():
 t=buf.lookup_transform('fr3_link0','fr3_hand_tcp',rclpy.time.Time());age=time.time()-t.header.stamp.sec-t.header.stamp.nanosec*1e-9
 assert 0<=age<.3,'TF stale: '+str(age)
 p=t.transform.translation;q=t.transform.rotation
 return np.array([p.x,p.y,p.z]),np.array([q.x,q.y,q.z,q.w])
def check():
 now=time.monotonic();assert now-box.get('state_at',0)<.3,'hardware state stale';assert not box['errors'],str(box['errors']);assert box['mode'] in [1,2],f"mode {box['mode']}";assert now-box.get('js_at',0)<.2,'joint receive stale';assert 0<=time.time()-box['js_stamp']<.2,'joint capture stale';assert all(c.poll() is None for c in children),'child exited'
 assert max(abs(x) for x in box['v'])<1.5,'joint speed envelope'
def interrupt(sig,frame):raise RuntimeError('signal '+str(sig))
signal.signal(signal.SIGTERM,interrupt);signal.signal(signal.SIGINT,interrupt)
sockpath=root/'input.sock';sockpath.unlink(missing_ok=True);rx=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM);rx.bind(str(sockpath));rx.setblocking(False);udp=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
result={'case':case,'scope':'raw RGBD inference -> fixed orientation adapter -> normal consumer/MPC/nvblox/feasible gated JTC; no grasp','status':'not started'};activated=False
try:
 raw=subprocess.check_output(['python3','/tmp/wrist_bag_1118/state_probe.py'],text=True);st=json.loads(raw.strip().splitlines()[-1]);(out/'preflight.json').write_text(json.dumps(st,indent=2))
 def leaves(v):
  if isinstance(v,dict):return [x for y in v.values() for x in leaves(y)]
  if isinstance(v,list):return [x for y in v for x in leaves(y)]
  return [v]
 assert not any(leaves(st['collision_indicators'])), 'contact indicator'
 assert st['robot_mode']==1 and st['inertia_load']['inertia']['m']==0 and not any(st['current_errors'].values()) and not any(st['last_motion_errors'].values())
 ps=subprocess.check_output(['ps','-eo','args'],text=True);assert '/lab/fw_rev/direct_goal_fwd.py' not in ps and '/tmp/wrist_bag_1118/direct_topic_fwd.py' not in ps
 launch(json.loads((root/'consumer_args.json').read_text()),'consumer.log');spin(2);trigger('/teleop/disengage');active(False)
 launch(['python3','-u','/lab/fw_rev/direct_goal_fwd.py'],'goal.log');launch(['python3','-u','/tmp/wrist_bag_1118/direct_topic_fwd.py'],'forwarder.log');active(True)
 ready=False
 for i in range(100):
  spin(.1)
  try:
   d=json.loads(pathlib.Path('/tmp/wrist_forwarder_status.json').read_text());ready=time.time()-d['wall']<.5 and d['ready']
  except (OSError,ValueError):pass
  if ready:break
 assert ready,'native measured-seed feasibility not ready'
 check();p0,q0=pose();j0=np.array(box['q']);log('start',p=p0.tolist(),q=q0.tolist(),j=j0.tolist());switch(True);activated=True
 (root/'ready.json').write_text(json.dumps({'case':case,'ready':True,'wall':time.time()}));print('READY_FOR_RAW_BAG',case,flush=True)
 start=time.monotonic();last_packet=None;received=0;sent=0;was_engaged=False;engage_attempt=0;last_measure=0
 while time.monotonic()-start<85:
  spin(.01);check()
  while True:
   try:data=rx.recv(65535)
   except BlockingIOError:break
   d=json.loads(data);received+=1;last_packet=time.monotonic();log('tracker',packet=d)
   age=(time.time_ns()-int(d['stamp_ns']))*1e-9
   if not d.get('position_valid') or not 0<=age<.35:continue
   d.update(orientation_valid=False,orientation_stamp_ns=None,q=None,gesture='Open_Palm',gesture_score=1.0)
   udp.sendto(json.dumps(d,allow_nan=False).encode(),('127.0.0.1',47984));sent+=1
   log('input_3d_locked',packet=d,age_s=age)
  now=time.monotonic()
  if sent>=12 and not box.get('engaged') and not was_engaged and now-engage_attempt>.6:
   engage_attempt=now
   try:trigger('/teleop/zed_start')
   except AssertionError as e:log('engage_wait',reason=str(e))
  if box.get('engaged'):was_engaged=True
  if was_engaged and not box.get('engaged'):raise RuntimeError('consumer disengaged during replay')
  if now-last_measure>.04:
   last_measure=now;p,q=pose();angle=2*math.acos(min(1.,abs(float(np.dot(q,q0)))))*180/math.pi
   log('measured',p=p.tolist(),q=q.tolist(),orientation_change_deg=angle)
   assert .24<p[2]<.95 and .18<p[0]<.82 and -.57<p[1]<.72,'TCP envelope'
   assert np.linalg.norm(p-p0)<.42,'displacement envelope';assert angle<10,'fixed orientation deviation';assert max(abs(np.array(box['q'])-j0))<1.2,'joint excursion envelope'
  if last_packet is not None and now-last_packet>1.5:break
 assert was_engaged and sent>30,'insufficient engaged bag input'
 result.update(status='completed',tracker_packets=received,sent_positions=sent,wall_s=time.monotonic()-start)
except BaseException as e:
 result.update(status='aborted',error=repr(e));print('ABORT',repr(e),flush=True)
finally:
 errors=[]
 for fn in [lambda:trigger('/teleop/disengage'),lambda:active(False)]:
  try:fn()
  except BaseException as e:errors.append(repr(e))
 for p in children:
  if p.poll() is None:p.send_signal(signal.SIGINT)
 for p in children:
  try:p.wait(timeout=4)
  except subprocess.TimeoutExpired:p.terminate();p.wait(timeout=3)
 if activated:
  try:switch(False)
  except BaseException as e:errors.append(repr(e))
 spin(.2);result.update(cleanup_errors=errors,final_mode=box.get('mode'));(out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True);sockpath.unlink(missing_ok=True);f.close();n.destroy_node();rclpy.shutdown()
 if result['status']!='completed' or errors:sys.exit(1)
