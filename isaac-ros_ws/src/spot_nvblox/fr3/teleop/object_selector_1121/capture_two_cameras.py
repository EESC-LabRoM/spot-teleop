"""Read synchronized VM fan-in frames for offline calibration analysis."""
import os,time,json
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,CameraInfo
from tf2_ros import Buffer,TransformListener
rclpy.init();n=Node('selector_two_camera_capture');queues={k:[] for k in ('r1','d1','i1','r2','d2','i2')}
def add(k,msg):
 queues[k].append(msg)
 if len(queues[k])>40:queues[k].pop(0)
for name,key,cls in [('/vm/cable/rgb','r1',Image),('/vm/cable/depth','d1',Image),('/zed1/zed_node/depth/camera_info','i1',CameraInfo),('/vm/cable2/rgb','r2',Image),('/vm/cable2/depth','d2',Image),('/zed2/zed_node/depth/camera_info','i2',CameraInfo)]:
 n.create_subscription(cls,name,lambda msg,k=key:add(k,msg),qos_profile_sensor_data)
b=Buffer();listener=TransformListener(b,n);stamp=lambda m:m.header.stamp.sec+m.header.stamp.nanosec*1e-9
end=time.monotonic()+15;result=None
while time.monotonic()<end:
 rclpy.spin_once(n,timeout_sec=.05)
 if not all(queues.values()):continue
 choices=[]
 for r1 in queues['r1']:
  d1=min(queues['d1'],key=lambda x:abs(stamp(x)-stamp(r1)))
  r2=min(queues['r2'],key=lambda x:abs(stamp(x)-stamp(r1)))
  d2=min(queues['d2'],key=lambda x:abs(stamp(x)-stamp(r2)))
  stamps=[stamp(x) for x in (r1,d1,r2,d2)]
  choices.append((max(stamps)-min(stamps),(r1,d1,r2,d2)))
 spread,msgs=min(choices,key=lambda x:x[0]);age=time.time()-max(stamp(x) for x in msgs)
 if spread <= .08 and 0 <= age <= 1.5:result=(spread,msgs);break
if result is None:raise RuntimeError('No fresh two-camera frames within 80ms')
os.makedirs('/tmp/object_selector_twocam',exist_ok=True)
for camera,(rgb,depth) in enumerate(((result[1][0],result[1][1]),(result[1][2],result[1][3])),1):
 if rgb.encoding not in ('rgb8','bgr8','rgba8','bgra8') or depth.encoding!='32FC1':raise RuntimeError('Unsupported frame encoding')
 channels=4 if 'a8' in rgb.encoding else 3
 img=np.frombuffer(rgb.data,np.uint8).reshape(rgb.height,rgb.step)[:,:rgb.width*channels].reshape(rgb.height,rgb.width,channels)[:,:,:3]
 if rgb.encoding.startswith('rgb'):img=img[:,:,::-1]
 dep=np.frombuffer(depth.data,dtype=('>f4' if depth.is_bigendian else '<f4')).reshape(depth.height,depth.step//4)[:,:depth.width]
 if img.shape[:2]!=dep.shape:raise RuntimeError('Unaligned image sizes')
 tr=b.lookup_transform('fr3_link0',depth.header.frame_id,rclpy.time.Time()).transform
 T,Q=tr.translation,tr.rotation
 np.savez_compressed(f'/tmp/object_selector_twocam/zed{camera}.npz',rgb_bgr=img,depth=dep,K=np.asarray(queues['i'+str(camera)][-1].k),camera=np.array([T.x,T.y,T.z,Q.x,Q.y,Q.z,Q.w]),stamp=stamp(rgb),depth_stamp=stamp(depth))
print(json.dumps({'spread_s':result[0],'age_s':time.time()-max(stamp(x) for x in result[1]),'paths':['/tmp/object_selector_twocam/zed1.npz','/tmp/object_selector_twocam/zed2.npz']}))
n.destroy_node();rclpy.shutdown()
