"""Bound native grasp assistance relative to each unassisted operator goal."""
import math,time,numpy as np

def bounded_delta(p0,q0,p1,q1,max_translation=.02,max_rotation=math.radians(10)):
 p0,p1=np.asarray(p0,float),np.asarray(p1,float);q0=np.asarray(q0,float);q1=np.asarray(q1,float);q0=q0/np.linalg.norm(q0);q1=q1/np.linalg.norm(q1)
 d=p1-p0;length=np.linalg.norm(d)
 if length>max_translation:d=d*max_translation/length
 dot=float(np.dot(q0,q1))
 if dot<0:q1=-q1;dot=-dot
 theta=math.acos(min(1.,dot));a=min(1.,max_rotation/max(2*theta,1e-12))
 if dot>.9995:q=q0+a*(q1-q0)
 else:q=(math.sin((1-a)*theta)*q0+math.sin(a*theta)*q1)/math.sin(theta)
 return p0+d,q/np.linalg.norm(q)

def apply(node):
 pose=node.goal_buffer.goal_pose;p0=pose.position.cpu().numpy().reshape(-1)[:3].copy();q0=pose.quaternion.cpu().numpy().reshape(-1)[:4].copy()
 node._apply_grasp_attraction()
 p1=pose.position.cpu().numpy().reshape(-1)[:3];q1=pose.quaternion.cpu().numpy().reshape(-1)[:4]
 p,q=bounded_delta(p0,q0,p1,q1)
 pose.position.copy_(node.tensor_args.to_device(p).view_as(pose.position));pose.quaternion.copy_(node.tensor_args.to_device(q).view_as(pose.quaternion))
 # Cache the actual bounded correction once per LIVE target; preserve native
 # loss clock/fade cache while the target is absent.
 if node._pf_delta is not None and node._grasp_msg is not None and time.time()-node._grasp_rx<=node._pf['pf_grasp_timeout_s']:
  w,x,y,z=q;v,a,b,c=q0/np.linalg.norm(q0);node._pf_delta=(p-p0,np.array([w*v+x*a+y*b+z*c,-w*a+x*v-y*c+z*b,-w*b+x*c+y*v-z*a,-w*c-x*b+y*a+z*v]))
 if node.step_count%25==0:
  import json
  from std_msgs.msg import String
  if not hasattr(node,'_manual_held_pub'):node._manual_held_pub=node.create_publisher(String,'/wrist_pick/held_status',10)
  node._manual_held_pub.publish(String(data=json.dumps(dict(wall=time.time(),attached=bool(node._held_attached),candidate=bool(node._held_candidate),fit_cached=node._held_fit_cache is not None))))
  ang=2*math.acos(min(1.,abs(float(np.dot(q0/np.linalg.norm(q0),q)))))
  node._log_info(f'Bounded manual assistance dp={np.linalg.norm(p-p0):.4f}m rot={math.degrees(ang):.2f}deg (caps .020m/10deg)',event='target_object',step=node.step_count)

class ApproachArming:
 """Engagement alone never attracts; require net inward operator movement."""
 def __init__(self,threshold=.01):self.threshold=threshold;self.reset()
 def reset(self):self.previous=None;self.inward=0.;self.armed=False
 def update(self,engaged,goal,target):
  if not engaged:self.reset();return False
  if target is None:return self.armed
  goal=np.asarray(goal,float);target=np.asarray(target,float)
  if self.previous is not None:
   self.inward+=float(np.linalg.norm(self.previous-target)-np.linalg.norm(goal-target));self.inward=max(0.,self.inward)
   if self.inward>=self.threshold:self.armed=True
  self.previous=goal.copy();return self.armed

def apply_guarded(node):
 if not hasattr(node,'_manual_arming'):
  from std_msgs.msg import Bool
  node._manual_arming=ApproachArming();node._manual_engaged=False;node._manual_engaged_at=0.
  def engaged(m):node._manual_engaged=m.data;node._manual_engaged_at=time.monotonic()
  node._manual_engaged_sub=node.create_subscription(Bool,'/teleop/engaged',engaged,10)
 g=node._grasp_msg;target=None
 if g is not None and time.time()-node._grasp_rx<=node._pf['pf_grasp_timeout_s']:target=np.array([g.pose.position.x,g.pose.position.y,g.pose.position.z])
 goal=node.goal_buffer.goal_pose.position.cpu().numpy().reshape(-1)[:3]
 live=node._manual_engaged and time.monotonic()-node._manual_engaged_at<1.5
 if node._manual_arming.update(live,goal,target):apply(node)
 else:
  node._pf_delta=None;node._pf_lost_t=None;node._pf_w=(0.,0.);node._pf_t=None
  if node._pf_intent is not None:node._pf_intent.reset()
