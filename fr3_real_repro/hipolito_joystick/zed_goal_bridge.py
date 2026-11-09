#!/usr/bin/env python3
"""Read-only ZED goal bridge: ROS77 TCP goals -> UDP loopback -> ROS8 MPC preview.
No hardware publishers, controller changes, Start, Home or gripper calls.
"""
import argparse,json,socket,time
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from tf2_ros import Buffer,TransformListener
from goal_math import tcp_to_hand

class Bridge(Node):
 def __init__(self,mode,port):
  super().__init__('zed_mpc_goal_'+mode);self.mode=mode
  self.sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);self.sock.setblocking(False)
  self.target=('127.0.0.1',port);self.status=None;self.status_at=0.;self.last_stamp=0
  if mode=='send':
   self.tf=Buffer();self.listener=TransformListener(self.tf,self)
   self.create_subscription(String,'/teleop/zed_status',self.on_status,10)
   self.create_subscription(PoseStamped,'/teleop/command_tool',self.on_goal,1)
  else:
   self.sock.bind(self.target);self.pub=self.create_publisher(PoseStamped,'/fr3_zed_mpc/wrist_goal',1)
   self.create_timer(.01,self.receive)
  self.forwarded=0;self.create_timer(10.,lambda:self.get_logger().info(f'{self.mode}: forwarded {self.forwarded} fresh ZED goals; preview only'))
 def on_status(self,m):
  try:self.status=json.loads(m.data);self.status_at=time.monotonic()
  except ValueError:self.status=None
 def on_goal(self,m):
  # Only the operator's active, calibrated ZED session can feed the preview.
  if self.status is None or time.monotonic()-self.status_at>.4 or self.status.get('phase')!='active':return
  ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec
  if m.header.frame_id!='fr3_link0' or not 0<=(time.time_ns()-ns)*1e-9<.3:return
  try:
   tr=self.tf.lookup_transform('fr3_hand_tcp','fr3_hand',Time()).transform
   p=m.pose.position;q=m.pose.orientation;t=tr.translation;r=tr.rotation
   pos,quat=tcp_to_hand([p.x,p.y,p.z],[q.x,q.y,q.z,q.w],[t.x,t.y,t.z],[r.x,r.y,r.z,r.w])
  except Exception:return
  packet=dict(version=1,frame='fr3_link0',stamp_ns=ns,p=pos,q=quat)
  self.sock.sendto(json.dumps(packet,allow_nan=False).encode(),self.target);self.forwarded+=1
 def receive(self):
  packet=None
  for _ in range(20):
   try:packet=json.loads(self.sock.recv(4096))
   except BlockingIOError:break
   except (ValueError,UnicodeError):continue
  if packet is None:return
  try:
   ns=int(packet['stamp_ns'])
   if packet['version']!=1 or packet['frame']!='fr3_link0' or ns<=self.last_stamp or not 0<=(time.time_ns()-ns)*1e-9<.3:return
   p,q=tcp_to_hand(packet['p'],packet['q'],[0,0,0],[0,0,0,1])
   msg=PoseStamped();msg.header.frame_id='fr3_link0';msg.header.stamp.sec,msg.header.stamp.nanosec=divmod(ns,10**9)
   msg.pose.position.x,msg.pose.position.y,msg.pose.position.z=p
   msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w=q
   self.pub.publish(msg);self.last_stamp=ns;self.forwarded+=1
  except (KeyError,TypeError,ValueError):return
 def destroy_node(self):self.sock.close();return super().destroy_node()

if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('mode',choices=['send','receive']);ap.add_argument('--port',type=int,default=47826);a=ap.parse_args()
 rclpy.init(domain_id=77 if a.mode=='send' else 8);n=Bridge(a.mode,a.port)
 try:rclpy.spin(n)
 except KeyboardInterrupt:pass
 finally:n.destroy_node();rclpy.try_shutdown()
