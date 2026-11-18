"""Bounded same-consumer synthetic input test; dry preview then supervised hardware."""
import os,sys,json,time,math,socket,threading,subprocess,pathlib
import rclpy,tf2_ros
from std_srvs.srv import Trigger,SetBool
from controller_manager_msgs.srv import SwitchController,ListControllers
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import PoseStamped
from mpc_packet_gate import SocketGate
rclpy.init();n=rclpy.create_node('bounded_fake_wrist');tf=tf2_ros.Buffer();tf2_ros.TransformListener(tf,n)
out=pathlib.Path('/tmp/wrist_drill_1117');record=open(out/'fake_test.jsonl','w',buffering=1);box={};children=[]
def log(k,**d):record.write(json.dumps(dict(kind=k,t=time.time(),**d))+'\n')
def js(m):box['js']=dict(zip(m.name,m.position));box['js_t']=time.monotonic()
n.create_subscription(JointState,'/joint_states',js,10)
n.create_subscription(JointTrajectory,'/fr3_arm_controller/joint_trajectory',lambda m:log('hardware_command',names=m.joint_names,q=list(m.points[0].positions)) if m.points else None,10)
n.create_subscription(PoseStamped,'/teleop/command_tool',lambda m:log('consumer_command',p=[m.pose.position.x,m.pose.position.y,m.pose.position.z],q=[m.pose.orientation.x,m.pose.orientation.y,m.pose.orientation.z,m.pose.orientation.w]),10)
def call(name,typ,req):
 c=n.create_client(typ,name);assert c.wait_for_service(timeout_sec=3),name
 f=c.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=7);assert f.done() and f.result() is not None,name
 r=f.result();assert getattr(r,'success',getattr(r,'ok',True)),str(r);n.destroy_client(c);return r
def active(v):
 r=SetBool.Request();r.data=v;call('/fr3_joystick_mpc/set_active',SetBool,r)
def switch(v):
 r=SwitchController.Request();r.activate_controllers=['fr3_arm_controller'] if v else [];r.deactivate_controllers=[] if v else ['fr3_arm_controller'];r.strictness=1;r.timeout.sec=4;call('/controller_manager/switch_controller',SwitchController,r)
def trig(name):return call(name,Trigger,Trigger.Request())
def spin(dt):
 t=time.monotonic()
 while time.monotonic()-t<dt:rclpy.spin_once(n,timeout_sec=.01)
def pose():
 tr=tf.lookup_transform('fr3_link0','fr3_hand_tcp',rclpy.time.Time());assert 0<=(n.get_clock().now().nanoseconds-(tr.header.stamp.sec*10**9+tr.header.stamp.nanosec))*1e-9<.3
 p=tr.transform.translation;q=tr.transform.rotation;return [p.x,p.y,p.z],[q.x,q.y,q.z,q.w]
def launch(args,fn):
 f=open(out/fn,'w');p=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT);children.append(p);return p
sending=True;motion=None;sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
def sender():
 while sending:
  t=0 if motion is None else time.monotonic()-motion
  u=min(1.,max(0.,t/3));s=u*u*(3-2*u);p=[1.315*.01*s,0.,1.315*.04*s]
  a=0.
  if 3<t<6:a=math.radians(15)*math.sin(math.pi*(t-3)/3)**2
  ns=time.time_ns();d=dict(version=3,frame='fr3_link0',orientation_mode='full_6d',position_valid=True,orientation_valid=True,stamp_ns=ns,orientation_stamp_ns=ns,p=p,q=[math.sin(a/2),0.,0.,math.cos(a/2)],gesture=None,gesture_score=0.)
  sock.sendto(json.dumps(d).encode(),('127.0.0.1',47984));time.sleep(.025)
threading.Thread(target=sender,daemon=True).start();gate=None
try:
 # Consumers and gates start freshly disengaged; no hardware forwarder in dry run.
 a=json.loads((out/'consumer_launch_args.json').read_text());a+=['-p','zed_gripper_enabled:=false','-p','zed_calibration_port:=0','-p','zed_yaw_rate_deg_s:=15.0','-p','linear_speed:=.10','-p','vertical_speed:=.10']
 launch(a,'fake_consumer.log');spin(2)
 trig('/teleop/disengage');active(False);switch(True)
 launch(['python3','-u','/lab/fw_rev/direct_goal_fwd.py'],'fake_goal_forwarder.log')
 active(True);spin(2)
 gate=SocketGate();metrics=[]
 def preview(m):
  if m.points:metrics.append((time.monotonic(),gate.accept(list(m.joint_names),list(m.points[0].positions)),list(m.points[0].positions),list(m.joint_names)))
 sub=n.create_subscription(JointTrajectory,'/fr3_joystick_mpc/joint_preview',preview,10)
 spin(1)
 assert len(metrics)>=10 and box.get('js') and time.monotonic()-box['js_t']<.2
 assert max(abs(q-box['js'][k]) for k,q in zip(metrics[-1][3],metrics[-1][2]))<.05,'unsafe seed'
 for phase in ['dry','physical']:
  if phase=='physical':
   gate.sock.close();gate=None;n.destroy_subscription(sub)
   launch(['python3','-u','/tmp/direct_topic_fwd.py'],'fake_hardware_forwarder.log');spin(.6)
  motion=None;trig('/teleop/zed_start');spin(1)
  p0,q0=pose();log('phase_start',phase=phase,p=p0,q=q0);motion=time.monotonic();start=motion
  while time.monotonic()-start<11:
   spin(.04);p,q=pose();log('measured',phase=phase,p=p,q=q)
   assert max(abs(p[i]-p0[i]) for i in range(3))<.065,'motion envelope exceeded'
   assert p[2]>.235,'unexpected descent'
   if phase=='dry' and time.monotonic()-start>1:
    recent=[x for x in metrics if x[0]>time.monotonic()-.5]
    assert len(recent)>=5 and sum(x[1] for x in recent)/len(recent)>.95,'dry preview infeasible/stale'
  trig('/teleop/disengage');active(False);motion=None
  if phase=='dry':
   assert len(metrics)>100
   print('DRY_PREVIEW_PASS',flush=True);active(True);spin(1)
  else:print('PHYSICAL_BOUNDED_TEST_COMPLETE',flush=True)
finally:
 sending=False
 try:trig('/teleop/disengage')
 except Exception as e:log('cleanup_error',error=str(e))
 try:active(False)
 except Exception as e:log('cleanup_error',error=str(e))
 for c in children:
  if c.poll() is None:c.send_signal(2)
 for c in children:
  try:c.wait(timeout=4)
  except subprocess.TimeoutExpired:c.terminate();c.wait(timeout=3)
 try:switch(False)
 except Exception as e:log('cleanup_error',error=str(e))
 if gate is not None:gate.sock.close()
 record.close();n.destroy_node();rclpy.shutdown()
