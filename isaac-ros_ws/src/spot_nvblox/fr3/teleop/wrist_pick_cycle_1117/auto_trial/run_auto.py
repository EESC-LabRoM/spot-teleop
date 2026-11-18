"""Exercise actual autonomous grasp service, recorder, canonical broker and guards."""
import pathlib,json,time,subprocess,os
os.environ['PYTHONPATH']='/lab/fr3_zed_runtime:'+os.environ.get('PYTHONPATH','')
import rclpy
from std_srvs.srv import Trigger
from std_msgs.msg import String
from controller_manager_msgs.srv import SwitchController
from trajectory_msgs.msg import JointTrajectory
out=pathlib.Path('/tmp/wrist_auto_trial_1117');rclpy.init();n=rclpy.create_node('wrist_autonomous_grasp_trial');box={};child=None;f=open(out/'trace.jsonl','w',buffering=1)
def log(k,**d):f.write(json.dumps(dict(kind=k,t=time.time(),**d))+'\n')
def status(m):
 d=json.loads(m.data);box.update(status=d,at=time.monotonic());log('status',**d)
n.create_subscription(String,'/teleop/grasp_status',status,10)
n.create_subscription(JointTrajectory,'/fr3_arm_controller/joint_trajectory',lambda m:log('controller',q=list(m.points[0].positions)) if m.points else None,10)
def spin(sec):
 t=time.monotonic()
 while time.monotonic()-t<sec:rclpy.spin_once(n,timeout_sec=.02)
def call(s,typ,r):
 c=n.create_client(typ,s);assert c.wait_for_service(timeout_sec=15),s
 try:
  v=c.call_async(r);rclpy.spin_until_future_complete(n,v,timeout_sec=8);assert v.done() and v.result() is not None,s;return v.result()
 finally:n.destroy_client(c)
def switch(v):
 r=SwitchController.Request();r.activate_controllers=['fr3_arm_controller'] if v else [];r.deactivate_controllers=[] if v else ['fr3_arm_controller'];r.strictness=1;r.timeout.sec=4;assert call('/controller_manager/switch_controller',SwitchController,r).ok
try:
 s=json.loads(subprocess.check_output(['python3','/tmp/wrist_ready_1117/state_probe.py'],text=True).strip().splitlines()[-1]);assert s['robot_mode']==1 and not any(s['current_errors'].values());assert s['inertia_load']['inertia']['m']==0
 ps=subprocess.check_output(['ps','-eo','args'],text=True);assert '/tmp/direct_topic_fwd.py' not in ps and '/lab/fw_rev/direct_goal_fwd.py' not in ps
 a=json.loads((out/'consumer_launch_args.json').read_text());a+=['-p','auto_grasp_enabled:=true','-p','zed_calibration_port:=0'];child=subprocess.Popen(a,stdout=open(out/'consumer.log','w'),stderr=subprocess.STDOUT);switch(True)
 for _ in range(140):
  spin(.1)
  if box.get('status',{}).get('ready_reason') is None and box.get('at',0)>time.monotonic()-1:break
 else:raise RuntimeError('autonomous consumer readiness missing')
 r=call('/teleop/grasp',Trigger,Trigger.Request());log('request',success=r.success,message=r.message);assert r.success,r.message;print('AUTONOMOUS_GRASP_TRIGGERED',flush=True);began=time.monotonic()
 while time.monotonic()-began<90:
  spin(.05);assert child.poll() is None,'consumer exited'
  s=box.get('status',{})
  if s.get('phase')=='cancelled':print('AUTONOMOUS_REFUSED '+s.get('note',''),flush=True);(out/'result.json').write_text(json.dumps(dict(result='refused',status=s),indent=2));break
  if s.get('phase')=='held':print('AUTONOMOUS_HELD',flush=True);(out/'result.json').write_text(json.dumps(dict(result='held',status=s),indent=2));break
 else:raise RuntimeError('autonomous trial timeout')
finally:
 try:call('/teleop/grasp_cancel',Trigger,Trigger.Request());spin(1)
 except Exception as e:log('cleanup_error',error=str(e))
 try:call('/teleop/disengage',Trigger,Trigger.Request())
 except Exception as e:log('cleanup_error',error=str(e))
 if child and child.poll() is None:
  child.send_signal(2)
  try:child.wait(timeout=5)
  except subprocess.TimeoutExpired:child.terminate();child.wait(timeout=3)
 if not box.get('status',{}).get('held',False):switch(False)
 else:print('OBJECT_HELD_LEAVING_JTC_HOLD_ACTIVE',flush=True)
 f.close();n.destroy_node();rclpy.shutdown()
