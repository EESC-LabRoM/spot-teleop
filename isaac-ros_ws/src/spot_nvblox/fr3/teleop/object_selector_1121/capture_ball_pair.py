"""Read a synchronized VM fan-in RGB/depth pair for offline perception only."""
import os,time,json
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,CameraInfo
from tf2_ros import Buffer,TransformListener

rclpy.init();n=Node('selector_offline_capture');rgbs=[];depths=[];infos=[]
n.create_subscription(Image,'/vm/cable/rgb',lambda m:rgbs.append(m),qos_profile_sensor_data)
n.create_subscription(Image,'/vm/cable/depth',lambda m:depths.append(m),qos_profile_sensor_data)
n.create_subscription(CameraInfo,'/zed1/zed_node/depth/camera_info',lambda m:infos.append(m),qos_profile_sensor_data)
b=Buffer();t=TransformListener(b,n);end=time.monotonic()+12;pair=None
st=lambda m:m.header.stamp.sec+m.header.stamp.nanosec/1e9
while time.monotonic()<end:
 rclpy.spin_once(n,timeout_sec=.05)
 if len(rgbs)>20:rgbs.pop(0)
 if len(depths)>20:depths.pop(0)
 if not (rgbs and depths and infos):continue
 pair=min(((abs(st(a)-st(d)),a,d) for a in rgbs for d in depths),key=lambda x:x[0])
 if pair[0]<.03 and 0<=time.time()-st(pair[1])<2:break
if pair is None or pair[0]>=.03 or time.time()-st(pair[1])>=2:raise RuntimeError('No fresh aligned frames')
_,rgb,depth=pair;info=infos[-1]
if rgb.encoding not in ('rgb8','bgr8','rgba8','bgra8'):raise RuntimeError('Unexpected rgb '+rgb.encoding)
if depth.encoding!='32FC1':raise RuntimeError('Unexpected depth '+depth.encoding)
channels=4 if 'a8' in rgb.encoding else 3
img=np.frombuffer(rgb.data,np.uint8).reshape(rgb.height,rgb.step)[:,:rgb.width*channels].reshape(rgb.height,rgb.width,channels)[:,:,:3]
if rgb.encoding.startswith('rgb'):img=img[:,:,::-1]
d=np.frombuffer(depth.data,dtype=('>f4' if depth.is_bigendian else '<f4')).reshape(depth.height,depth.step//4)[:,:depth.width]
if img.shape[:2]!=d.shape:raise RuntimeError(f'unmatched rgb {img.shape} depth {d.shape}')
tr=b.lookup_transform('fr3_link0',depth.header.frame_id,rclpy.time.Time())
pos=tr.transform.translation;q=tr.transform.rotation
out='/tmp/object_selector_capture';os.makedirs(out,exist_ok=True)
np.savez_compressed(out+'/pair.npz',rgb_bgr=img,depth=d,K=np.array(info.k),camera=np.array([pos.x,pos.y,pos.z,q.x,q.y,q.z,q.w]),stamp=st(rgb),depth_stamp=st(depth))
print(json.dumps({'rgb':rgb.encoding,'depth':depth.encoding,'shape':d.shape,'delta_s':pair[0],'age_s':time.time()-st(rgb),'depth_frame':depth.header.frame_id,'path':out+'/pair.npz'}))
n.destroy_node();rclpy.shutdown()
