"""Fresh source-paired masked depth; no cached/reprojected grasp cloud accepted."""
import json,time,pathlib,collections,numpy as np,cv2,rclpy,tf2_ros
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Image,CameraInfo,CompressedImage
from rclpy.qos import qos_profile_sensor_data,QoSProfile,ReliabilityPolicy
out=pathlib.Path('/tmp/wrist_pick_cycle_1117');rclpy.init();n=rclpy.create_node('wrist_fresh_drill_cloud');tf=tf2_ros.Buffer();tf2_ros.TransformListener(tf,n);depths=collections.deque(maxlen=90);masks=collections.deque(maxlen=10);box={};samples=[];last=0
stamp=lambda m:m.header.stamp.sec+m.header.stamp.nanosec*1e-9
n.create_subscription(Image,'/franka_seg_zed1/depth',depths.append,qos_profile_sensor_data)
n.create_subscription(CompressedImage,'/wrist_map/drill_mask/compressed',masks.append,QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE))
n.create_subscription(CameraInfo,'/zed1/zed_node/depth/camera_info',lambda m:box.update(K=np.array(m.k).reshape(3,3)),qos_profile_sensor_data)
t0=time.monotonic()
while time.monotonic()-t0<30:
 rclpy.spin_once(n,timeout_sec=.03)
 if not depths or not masks or 'K' not in box:continue
 m=masks[-1];d=min(depths,key=lambda d:abs(stamp(d)-stamp(m)));now=n.get_clock().now().nanoseconds*1e-9
 if stamp(d)<=last or abs(stamp(m)-stamp(d))>.025 or not 0<now-stamp(d)<1.5:continue
 try:t=tf.lookup_transform('fr3_link0',d.header.frame_id,rclpy.time.Time()).transform
 except Exception:continue
 assert d.encoding=='32FC1' and 'png' in m.format.lower()
 mask=cv2.imdecode(np.frombuffer(m.data,np.uint8),0);mask=cv2.resize(mask,(d.width,d.height),interpolation=cv2.INTER_NEAREST);mask=cv2.erode((mask>0).astype(np.uint8),np.ones((13,13),np.uint8))>0
 dep=np.frombuffer(d.data,np.float32).reshape(d.height,d.step//4)[:,:d.width];ok=mask&np.isfinite(dep)&(dep>.15)&(dep<2.)
 yy,xx=np.nonzero(ok);z=dep[ok];K=box['K'];pc=np.column_stack(((xx-K[0,2])*z/K[0,0],(yy-K[1,2])*z/K[1,1],z));R=Rotation.from_quat([t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w]).as_matrix();base=pc@R.T+[t.translation.x,t.translation.y,t.translation.z]
 if len(base)<300:continue
 samples.append(base);last=stamp(d);print(json.dumps(dict(n=len(base),stamp=last,age=now-last,centroid=np.median(base,axis=0).tolist())),flush=True)
 if len(samples)>=3:break
assert len(samples)>=3,'fresh paired masked depth missing'
P=np.concatenate(samples);P=P[np.all(np.isfinite(P),axis=1)];np.save(out/'fresh_drill_cloud.npy',P.astype(np.float32));np.savez_compressed(out/'fresh_drill_cloud_evidence.npz',depth=dep,mask=mask,K=K,R_base_camera=R,t_base_camera=[t.translation.x,t.translation.y,t.translation.z]);(out/'fresh_drill_cloud.json').write_text(json.dumps(dict(stamp=last,captured_wall=time.time(),n=len(P),centroid=np.median(P,axis=0).tolist(),bounds=np.quantile(P,[.01,.99],axis=0).tolist(),calibration='existing ZED1, known common error; NOT contact validated',source='source-matched PNG mask and robot-filtered depth'),indent=2));n.destroy_node();rclpy.shutdown()
