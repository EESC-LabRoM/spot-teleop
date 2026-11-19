"""Replay a ZED bag through actual tracker at 1x; isolated ROS; explicit UNIX sample socket feeds supervised hardware adapter."""
import argparse,json,sys,time,socket
from pathlib import Path
from collections import Counter
from array import array
from types import SimpleNamespace
import cv2,numpy as np,rclpy,rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image,CompressedImage,CameraInfo
from std_msgs.msg import String
import os
sys.path.insert(0,os.environ.get('TRACKER_DIR','/home/spot-teleop/isaac-ros_ws/src/spot_nvblox/fr3/remote_sim'))
from operator_hand_tracker_debug import OperatorHandTracker,robust_depth
from depth_codec import decode_depth
p=argparse.ArgumentParser();p.add_argument('--bag',required=True,action='append');p.add_argument('--output',required=True);p.add_argument('--duration',type=float,default=12);p.add_argument('--seed-arm-length',type=float);p.add_argument('--orientation-output',default='full_6d');p.add_argument('--shoulder-mode',default=None);p.add_argument('--operator-profile',type=Path);p.add_argument('--pose-source',choices=['rgbd_fixed','rgbd_body'],default='rgbd_body');a=p.parse_args()
rgb='/zed3/zed_node/rgb/image_rect_color/compressed';dep='/zed3/zed_node/depth/depth_registered/compressedDepth';info='/zed3/zed_node/depth/camera_info'
colors={};depths={};cal=None;stamps=[];take_of={};receipt={};received={}
for bagpath in a.bag:
 reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=bagpath,storage_id='mcap'),rosbag2_py.ConverterOptions('cdr','cdr'));reader.set_filter(rosbag2_py.StorageFilter(topics=[rgb,dep,info]));c={};d={}
 while reader.has_next():
  topic,data,record_ns=reader.read_next()
  if topic==info:
   if cal is None:cal=deserialize_message(data,CameraInfo)
  else:
   msg=deserialize_message(data,CompressedImage);st=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
   (c if topic==rgb else d)[st]=msg;received.setdefault(st,{})[topic]=record_ns
 ss=sorted(c.keys()&d.keys());ss=[x for x in ss if x-ss[0]<=a.duration*1e9]
 if not ss:continue
 off=0
 if stamps and ss[0]<=stamps[-1]:raise ValueError('nonmonotonic bag session timestamps')
 for x in ss:colors[x+off]=c[x];depths[x+off]=d[x];stamps.append(x+off);take_of[x+off]=__import__('re').search(r'_(\d{2})_',Path(bagpath).parent.parent.name).group(1)
receipt={s:max(received[s][rgb],received[s][dep]) for s in stamps}
base=stamps[0]
if any(not 0<=receipt[s]-s<=10_000_000_000 for s in stamps):raise ValueError('recording and acquisition clocks incompatible')
rclpy.init(domain_id=199);kw=dict(domain=199,output_prefix='/isolated',max_input_age=.25,max_output_age=.35,calibrate_arm=a.pose_source=='rgbd_body',smooth_preview=False,pose_source=a.pose_source,operator_profile=json.loads(a.operator_profile.read_text()) if a.operator_profile else None)
if a.orientation_output!='full_6d':kw['orientation_output']=a.orientation_output
if a.shoulder_mode:kw['shoulder_mode']=a.shoulder_mode
node=OperatorHandTracker(**kw)
node.info(cal);last={};last_take=None
if a.seed_arm_length is not None:
 if not .45 < a.seed_arm_length < .95:raise ValueError('Invalid saved arm length')
 # Controlled offline comparison: restore the same operator's measured length,
 # and require a fresh shoulder detection from the actual first RGB-D frame.
 c=colors[stamps[0]]; rgb0=cv2.imdecode(np.frombuffer(c.data,np.uint8),cv2.IMREAD_COLOR);depth0=decode_depth(depths[stamps[0]])
 body=node.body.process(cv2.cvtColor(rgb0,cv2.COLOR_BGR2RGB))
 if not body.pose_landmarks:raise RuntimeError('First frame has no visible shoulder; cannot seed')
 shoulder=body.pose_landmarks.landmark[12];u,v=int(shoulder.x*rgb0.shape[1]),int(shoulder.y*rgb0.shape[0])
 z=robust_depth(depth0,u,v,4,(.3,3.)) if 4<=u<rgb0.shape[1]-4 and 4<=v<rgb0.shape[0]-4 else None
 if shoulder.visibility<.7 or z is None:raise RuntimeError('No valid shoulder in first frame; cannot seed')
 node.arm.length=a.seed_arm_length;node.arm.scale=node.arm.reach/a.seed_arm_length;node.arm.shoulder=np.array([(u-node.k[0,2])*z/node.k[0,0],(v-node.k[1,2])*z/node.k[1,1],z])
class Sink:
 def __init__(self,key):self.key=key
 def publish(self,msg):
  try:last[self.key]=json.loads(msg.data)
  except (ValueError,AttributeError):pass
control_socket=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM)
original_publish=Sink.publish
def streamed_publish(self,msg):
 original_publish(self,msg)
 if self.key=='sample':
  control_socket.sendto(msg.data.encode(),'/io/input.sock')
Sink.publish=streamed_publish
node.sample_pub=Sink('sample');node.diag_pub=Sink('diag');node.camera_pose_pub=SimpleNamespace(publish=lambda m:last.__setitem__('tf',[m.pose.position.x,m.pose.position.y,m.pose.position.z,m.pose.orientation.x,m.pose.orientation.y,m.pose.orientation.z,m.pose.orientation.w]));node.overlay_pub=SimpleNamespace(publish=lambda x:None);node.tf_broadcaster=SimpleNamespace(sendTransform=lambda x:None)
wall=time.time_ns()+300_000_000;results=[];drops=0
for stamp in stamps:
 if take_of[stamp]!=last_take:
  last_take=take_of[stamp];print('REPLAY_TAKE',last_take,flush=True)
 capture=wall+(stamp-base)
 due=wall+(receipt[stamp]-base)
 while time.time_ns()<due:time.sleep(max(0.,min(.01,(due-time.time_ns())/1e9)))
 if time.time_ns()-due>250_000_000:drops+=1;continue
 color=colors[stamp];depth=depths[stamp];im=cv2.imdecode(np.frombuffer(color.data,np.uint8),cv2.IMREAD_COLOR);dm=decode_depth(depth)
 if im is None or im.shape[:2]!=dm.shape:raise RuntimeError('invalid frame')
 def make(data,encoding,step):
  m=Image();m.header=color.header;m.header.stamp.sec,m.header.stamp.nanosec=divmod(capture,10**9);m.height,m.width=data.shape[:2];m.encoding=encoding;m.step=step;m.data=array('B',data.tobytes());return m
 last.clear();before=time.perf_counter();node.frame(make(im,'bgr8',im.shape[1]*3),make(dm,'32FC1',dm.shape[1]*4));elapsed=(time.perf_counter()-before)*1000
 results.append(dict(arm=node.arm.status() if node.arm is not None else None,body_reference=node.body_reference.status(capture*1e-9),recorded_input_age_ms=(receipt[stamp]-stamp)*1e-6,take=take_of[stamp],offset_s=(stamp-base)*1e-9,processing_ms=round(elapsed,2),sample=last.get('sample'),diagnostics=last.get('diag'),tf=last.get('tf')))
node.destroy_node();rclpy.shutdown()
counts=Counter((x.get('diagnostics') or {}).get('reason','no diagnostic') for x in results);valid=[x for x in results if x.get('sample') and x['sample'].get('position_valid')]
summary={'bag':a.bag,'seed_arm_length_m':a.seed_arm_length,'recorded_duration_s':a.duration,'paired_frames':len(stamps),'frames_processed':len(results),'late_frames_dropped':drops,'valid_positions':len(valid),'diagnostic_reasons':counts,'processing_median_ms':float(np.median([x['processing_ms'] for x in results])) if results else None,'processing_max_ms':max((x['processing_ms'] for x in results),default=None),'wall_elapsed_s':(time.time_ns()-wall)/1e9,'note':'CPU limited; raw image inference; network-none container deliberately emits samples through UNIX socket to supervised real-robot adapter'}
out=Path(a.output);out.mkdir(parents=True,exist_ok=True);(out/'summary.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False));(out/'trace.jsonl').write_text(''.join(json.dumps(x,ensure_ascii=False)+chr(10) for x in results));print(json.dumps(summary,indent=2,ensure_ascii=False),flush=True)
