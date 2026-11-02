#!/usr/bin/env python3
"""Private MPC goal transport and guarded effort-JTC output. Starts DISARMED; never switches controllers."""
import argparse,json,socket,time,threading
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from std_msgs.msg import Bool,String
from std_srvs.srv import SetBool
from controller_manager_msgs.srv import ListControllers
from tf2_ros import Buffer,TransformListener
from goal_math import tcp_to_hand
from command_guard import validate,ARM

class Bridge(Node):
 def __init__(self,mode,goal_port=47828,packet_port=47827,status_port=8784):
  self.goal_port=goal_port
  super().__init__('fr3_curobo_'+mode);self.mode=mode;self.sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);self.sock.setblocking(False)
  self.armed=False;self.engaged=False;self.engaged_at=0.;self.source=None;self.status=None;self.status_at=0.;self.command=None;self.q=None;self.q_at=0.;self.last_packet=None;self.last_sent=None;self.last_previous=None;self.controllers={};self.controllers_at=0.;self.reason='disarmed';self.sent=0;self.goal_stamp=0
  self.tf=Buffer();self.listener=TransformListener(self.tf,self)
  if mode=='receive_goals':
   self.sock.bind(('127.0.0.1',goal_port));self.goal_pub=self.create_publisher(PoseStamped,'/fr3_joystick_mpc/wrist_goal',1);self.create_timer(.005,self.receive_goals);return
  self.http_state={};self.http_servers=[]
  node=self
  class Handler(BaseHTTPRequestHandler):
   def log_message(self,*args):pass
   def do_GET(self):
    if self.path.split('?')[0]!='/status':self.send_response(404);self.end_headers();return
    data=json.dumps(node.http_state).encode();self.send_response(200);self.send_header('Content-Type','application/json');self.send_header('Cache-Control','no-store');self.send_header('Access-Control-Allow-Origin','*');self.send_header('Content-Length',str(len(data)));self.end_headers()
    try:self.wfile.write(data)
    except (BrokenPipeError,ConnectionResetError):pass
  for host in ('127.0.0.1','100.88.134.127'):
   server=ThreadingHTTPServer((host,status_port),Handler);server.daemon_threads=True;threading.Thread(target=server.serve_forever,daemon=True).start();self.http_servers.append(server)
  self.sock.bind(('127.0.0.1',packet_port));self.jtc=self.create_publisher(JointTrajectory,'/fr3_arm_controller/joint_trajectory',1)
  self.state_pub=self.create_publisher(String,'/fr3_curobo/status',1)
  self.create_subscription(Bool,'/teleop/engaged',self.engage_status,1)
  latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
  self.create_subscription(String,'/teleop/input_source',lambda m:setattr(self,'source',m.data),latched)
  self.create_subscription(PoseStamped,'/teleop/command_tool',lambda m:setattr(self,'command',m),1)
  self.create_subscription(String,'/teleop/zed_status',self.zed_status,1)
  self.create_subscription(JointState,'/joint_states',self.joints,10)
  self.cm=self.create_client(ListControllers,'/controller_manager/list_controllers');self.cm_pending=None
  self.create_timer(.25,self.poll_controllers);self.create_timer(.02,self.send_goal);self.create_timer(.005,self.output)
  self.create_timer(.2,self.publish_status);self.create_service(SetBool,'/fr3_curobo/enable',self.enable)
 def engage_status(self,m):self.engaged=m.data;self.engaged_at=time.monotonic()
 def zed_status(self,m):
  try:self.status=json.loads(m.data);self.status_at=time.monotonic()
  except ValueError:self.status=None
 def joints(self,m):
  d=dict(zip(m.name,m.position))
  if all(j in d for j in ARM):self.q=[d[j] for j in ARM];self.q_at=time.monotonic()
 def poll_controllers(self):
  if self.cm_pending is not None and not self.cm_pending.done():return
  if not self.cm.service_is_ready():return
  self.cm_pending=self.cm.call_async(ListControllers.Request())
  def done(future):
   try:self.controllers={c.name:c.state for c in future.result().controller};self.controllers_at=time.monotonic()
   except Exception:self.controllers={}
  self.cm_pending.add_done_callback(done)
 def controller_reason(self):
  if time.monotonic()-self.controllers_at>=.8:return 'controller state stale'
  if self.controllers.get('fr3_arm_controller')!='active':return 'joint controller inactive'
  if self.controllers.get('cartesian_impedance_target_controller','inactive')!='inactive':return 'Cartesian controller still active'
  if self.jtc.get_subscription_count()<1:return 'joint controller has no command subscription'
  if len(self.get_publishers_info_by_topic('/fr3_arm_controller/joint_trajectory'))!=1:return 'unexpected joint command publisher ownership'
  return None
 def controller_ready(self):return self.controller_reason() is None
 def current_packet(self,previous=None):
  if self.q is None or time.monotonic()-self.q_at>.2:raise ValueError('robot joint feedback stale')
  if self.last_packet is None:raise ValueError('no MPC packet')
  return validate(self.last_packet,self.q,self.get_clock().now().nanoseconds,previous)
 def enable(self,request,response):
  if not request.data:self.armed=False;self.last_sent=None;response.success=True;response.message='cuRobo output disabled';return response
  try:
   if self.engaged or time.monotonic()-self.engaged_at>.75:raise ValueError('disengage teleop before enabling MPC')
   if not self.controller_ready():raise ValueError('joint controller ownership not ready')
   self.current_packet();self.armed=True;self.last_sent=None;response.success=True;response.message='MPC ready; press joystick Start'
  except (ValueError,KeyError,TypeError) as error:response.success=False;response.message=str(error)
  return response
 def send_goal(self):
  try:
   now=self.get_clock().now().nanoseconds
   live=self.armed and self.engaged and time.monotonic()-self.engaged_at<.75
   # ZED pauses refresh only the measured hold, never an old hand target.
   if live and self.source=='zed':
    stamp=0 if self.command is None else self.command.header.stamp.sec*10**9+self.command.header.stamp.nanosec
    live=self.status is not None and time.monotonic()-self.status_at<=.4 and self.status.get('phase')=='active' and 0<=(now-stamp)*1e-9<.25
   if live:
    if self.command is None:return
    message=self.command;stamp=message.header.stamp.sec*10**9+message.header.stamp.nanosec
    if not 0<=(now-stamp)*1e-9<.25 or message.header.frame_id!='fr3_link0':return
    tr=self.tf.lookup_transform('fr3_hand_tcp','fr3_hand',rclpy.time.Time()).transform;p=message.pose.position;q=message.pose.orientation;t=tr.translation;r=tr.rotation
    pos,quat=tcp_to_hand([p.x,p.y,p.z],[q.x,q.y,q.z,q.w],[t.x,t.y,t.z],[r.x,r.y,r.z,r.w])
   else:
    tr=self.tf.lookup_transform('fr3_link0','fr3_hand',rclpy.time.Time());stamp=tr.header.stamp.sec*10**9+tr.header.stamp.nanosec
    if not 0<=(now-stamp)*1e-9<.25:return
    t=tr.transform.translation;q=tr.transform.rotation;pos=[t.x,t.y,t.z];quat=[q.x,q.y,q.z,q.w]
   packet=dict(version=1,frame='fr3_link0',stamp_ns=stamp,p=pos,q=quat)
   self.sock.sendto(json.dumps(packet,allow_nan=False).encode(),('127.0.0.1',self.goal_port))
  except Exception:pass
 def receive_goals(self):
  packet=None
  for _ in range(32):
   try:packet=json.loads(self.sock.recv(8192))
   except BlockingIOError:break
   except ValueError:continue
  if packet is None:return
  try:
   ns=int(packet['stamp_ns'])
   if packet['version']!=1 or packet['frame']!='fr3_link0' or ns<=self.goal_stamp or not 0<=(self.get_clock().now().nanoseconds-ns)*1e-9<.3:return
   p,q=tcp_to_hand(packet['p'],packet['q'],[0,0,0],[0,0,0,1]);m=PoseStamped();m.header.frame_id='fr3_link0';m.header.stamp.sec,m.header.stamp.nanosec=divmod(ns,10**9)
   m.pose.position.x,m.pose.position.y,m.pose.position.z=p;m.pose.orientation.x,m.pose.orientation.y,m.pose.orientation.z,m.pose.orientation.w=q
   self.goal_pub.publish(m);self.goal_stamp=ns
  except (ValueError,TypeError,KeyError):pass
 def output(self):
  latest=None
  for _ in range(64):
   try:latest=json.loads(self.sock.recv(16384))
   except BlockingIOError:break
   except ValueError:continue
  if latest is not None:self.last_packet=latest
  if not self.armed:self.reason='disarmed';return
  if not self.controller_ready():self.reason='joint controller ownership lost';self.armed=False;self.last_sent=None;return
  if not self.engaged or time.monotonic()-self.engaged_at>.75:self.reason='waiting for joystick Start';self.last_sent=None;return
  if self.source not in ('joystick','zed'):self.reason='input source unknown';return
  if self.source=='zed' and (self.status is None or time.monotonic()-self.status_at>.4 or self.status.get('phase')!='active'):self.reason='ZED paused';return
  try:
   if self.last_sent is not None and (self.get_clock().now().nanoseconds-self.last_sent['stamp_ns'])*1e-9>.15:self.last_sent=None
   if self.last_sent is not None and self.last_packet is not None and self.last_packet.get('stamp_ns')==self.last_sent.get('stamp_ns'):return
   q,v=self.current_packet(self.last_sent);message=JointTrajectory();message.joint_names=ARM
   point=JointTrajectoryPoint();point.positions=q;point.velocities=v;point.time_from_start.nanosec=20000000;message.points=[point]
   self.jtc.publish(message);self.last_previous=self.last_sent;self.last_sent=self.last_packet.copy();self.sent+=1;self.reason='active'
  except (ValueError,KeyError,TypeError,OverflowError) as error:self.reason=str(error)
 def publish_status(self):
  packet_ready=False;packet_reason=None
  # Recheck a sent packet against its original predecessor, without replaying it.
  # New packets use the same history and interruption limit as output().
  previous=self.last_sent
  if previous is not None:
   if (self.get_clock().now().nanoseconds-previous['stamp_ns'])*1e-9>.15:previous=None
   elif self.last_packet is not None and self.last_packet.get('stamp_ns')==previous['stamp_ns']:previous=self.last_previous
  try:self.current_packet(previous);packet_ready=True
  except (ValueError,KeyError,TypeError,OverflowError) as error:packet_reason=str(error)
  ownership=self.controller_ready()
  ready_reason=packet_reason or self.controller_reason() or ('cuRobo output disarmed' if not self.armed else None)
  mode_ready=self.armed and ownership and packet_ready and self.last_packet is not None and 0<=(self.get_clock().now().nanoseconds-self.last_packet.get('stamp_ns',0))*1e-9<.3
  self.http_state=dict(armed=self.armed,controller_ready=ownership,ready_reason=ready_reason,mode_ready=mode_ready,command_ready=packet_ready,reason=self.reason,packet_reason=packet_reason,sent=self.sent,controllers=self.controllers,controllers_age_ms=round((time.monotonic()-self.controllers_at)*1000),joint_feedback_age_ms=round((time.monotonic()-self.q_at)*1000) if self.q_at else None,hardware_publisher_count=len(self.get_publishers_info_by_topic('/fr3_arm_controller/joint_trajectory')))
  self.state_pub.publish(String(data=json.dumps(self.http_state)))
 def destroy_node(self):self.sock.close();return super().destroy_node()
if __name__=='__main__':
 parser=argparse.ArgumentParser();parser.add_argument('mode',choices=['hardware','receive_goals']);parser.add_argument('--domain',type=int);parser.add_argument('--goal-port',type=int,default=47828);parser.add_argument('--packet-port',type=int,default=47827);parser.add_argument('--status-port',type=int,default=8784);args=parser.parse_args()
 rclpy.init(domain_id=args.domain if args.domain is not None else (77 if args.mode=='hardware' else 8));node=Bridge(args.mode,args.goal_port,args.packet_port,args.status_port)
 try:rclpy.spin(node)
 except KeyboardInterrupt:pass
 finally:node.destroy_node();rclpy.try_shutdown()
