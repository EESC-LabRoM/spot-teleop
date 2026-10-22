"""Pull latest JPEG + millimetre depth over TCP; preserve source capture timestamps."""
import json,socket,time,zlib
from array import array
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,CameraInfo
rclpy.init(domain_id=188);node=Node('remote_rgbd_camera')
pubs=[node.create_publisher(Image,'/fr3_sim/operator/'+n,qos_profile_sensor_data) for n in ('rgb','depth')]
info_pub=node.create_publisher(CameraInfo,'/fr3_sim/operator/camera_info',1)
count=0;received=0
try:
 while rclpy.ok():
  try:
   with socket.create_connection(('100.118.129.74',30020),timeout=5) as sock:
    sock.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1);stream=sock.makefile('rb')
    def exact(n):
     b=stream.read(n)
     if len(b)!=n:raise ConnectionError('short frame')
     return b
    while rclpy.ok():
     sock.sendall(b'G');header=stream.readline(2048)
     if not header:raise ConnectionError('closed')
     d=json.loads(header);nr,nd=int(d['rgb_bytes']),int(d['depth_bytes'])
     if not 0<nr<2000000 or not 0<nd<200000:raise ValueError('invalid frame size')
     rgb=cv2.imdecode(np.frombuffer(exact(nr),np.uint8),cv2.IMREAD_COLOR)
     mm=np.frombuffer(zlib.decompress(exact(nd)),dtype='<u2').reshape(180,320)
     depth=cv2.resize(mm,(640,360),interpolation=cv2.INTER_NEAREST).astype(np.float32)*.001;depth[depth==0]=np.nan
     age=(time.time_ns()-int(d['stamp_ns']))*1e-9
     received+=1
     if received%20==1:print('RGBD received',received,'capture_age_ms',round(age*1000,1),'bytes',nr+nd,flush=True)
     if not 0<=age<.65:continue # Never restamp old frames as fresh.
     ci=CameraInfo();ci.header.stamp.sec,ci.header.stamp.nanosec=divmod(int(d['stamp_ns']),10**9);ci.header.frame_id='operator_left_camera_optical_frame'
     ci.width=640;ci.height=360;ci.k=[float(d['fx']),0.,float(d['cx']),0.,float(d['fy']),float(d['cy']),0.,0.,1.];ci.p=[ci.k[0],0.,ci.k[2],0.,0.,ci.k[4],ci.k[5],0.,0.,0.,1.,0.];info_pub.publish(ci)
     for pub,data,enc,step in zip(pubs,(rgb,depth),('bgr8','32FC1'),(1920,2560)):
      msg=Image();msg.header=ci.header;msg.width=640;msg.height=360;msg.encoding=enc;msg.step=step;msg.data=array('B',np.ascontiguousarray(data).tobytes());pub.publish(msg)
     count+=1
     if count%50==0:print('RGBD frames',count,'capture_age_ms',round(age*1000,1),'bytes',nr+nd,flush=True)
     rclpy.spin_once(node,timeout_sec=0.)
  except (OSError,ValueError,ConnectionError,zlib.error) as e:print('RGBD reconnect:',e,flush=True);time.sleep(1)
finally:
 node.destroy_node()
 if rclpy.ok():rclpy.shutdown()
