"""Replay every saved ZED RGB/depth pair at capture time; never publish ROS commands.

Outputs isolated derived bags, detector cache and timestamped video. The observed
arm length is restored from this session, rather than recalibrated per exercise.
"""
import argparse,json,sys,subprocess,shutil
from pathlib import Path
from array import array
from collections import Counter
from types import SimpleNamespace
import cv2,numpy as np,rclpy,rosbag2_py
from rclpy.serialization import deserialize_message,serialize_message
from sensor_msgs.msg import Image,CompressedImage,CameraInfo
HERE=Path(__file__).resolve().parent
sys.path[:0]=[str(HERE.parent/'realsense'),str(HERE.parent/'remote_sim')]
import reprocess_bags as rp
from bag_replay import decode_depth
tracking=rp.tracking

def main():
 ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--bags',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);ap.add_argument('--stage',default='*');a=ap.parse_args()
 a.output.mkdir(parents=True,exist_ok=True)
 tracking.time=rp.clock;rclpy.init(domain_id=199)
 observations=rp.Observations(a.output/'detector_cache');original=tracking.SolutionHandDetector
 tracking.SolutionHandDetector=lambda *x,**kw:observations.wrap(original(*x,**kw),'crop' if kw.get('engine') else 'hand')
 for stage in sorted(a.bags.glob(a.stage)):
  if not (stage/'d8').exists():continue
  out=a.output/stage.name
  if (out/'summary.json').exists():continue
  out.mkdir(exist_ok=True)
  # Only discard an incomplete derived output; original bags are never touched.
  if (out/'replay').exists():shutil.rmtree(out/'replay')
  frames=out/'frames';frames.mkdir(exist_ok=True)
  observations.load(stage.name);state={};node=tracking.OperatorHandTracker(domain=199,output_prefix='/offline',calibrate_arm=True)
  node.body=observations.wrap(node.body,'body')
  node.sample_pub=rp.Sink('/fr3_operator/teleop_sample','json',state);node.diag_pub=rp.Sink('/fr3_operator/tracking','json',state);node.camera_pose_pub=rp.Sink('/fr3_operator/hand_camera_pose','pose',state)
  node.overlay_pub=SimpleNamespace(publish=lambda _:None);node.tf_broadcaster=SimpleNamespace(sendTransform=lambda _:None)
  reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(next((stage/'d8').glob('*.mcap'))),storage_id='mcap'),rosbag2_py.ConverterOptions('cdr','cdr'))
  rgb='/zed3/zed_node/rgb/image_rect_color/compressed';depth='/zed3/zed_node/depth/depth_registered/compressedDepth';info='/zed3/zed_node/depth/camera_info';diag='/fr3_operator/tracking'
  reader.set_filter(rosbag2_py.StorageFilter(topics=[rgb,depth,info,diag]));colors={};depths={};cal=None;arm=None
  while reader.has_next():
   topic,buf,t=reader.read_next()
   if topic==info:cal=deserialize_message(buf,CameraInfo)
   elif topic==diag and arm is None:
    d=json.loads(deserialize_message(buf,__import__('std_msgs.msg',fromlist=['String']).String).data);arm=d.get('arm_length_m')
   else:
    if topic not in (rgb,depth):continue
    m=deserialize_message(buf,CompressedImage);ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec
    (colors if topic==rgb else depths)[ns]=m
  if cal is None or arm is None:raise RuntimeError('Missing calibration or original arm length')
  stamps=sorted(colors.keys() & depths.keys());node.info(cal)
  writer=rosbag2_py.SequentialWriter();writer.open(rosbag2_py.StorageOptions(uri=str(out/'replay'),storage_id='mcap'),rosbag2_py.ConverterOptions('cdr','cdr'))
  for name,kind in [('/fr3_operator/teleop_sample','std_msgs/msg/String'),('/fr3_operator/tracking','std_msgs/msg/String'),('/fr3_operator/hand_camera_pose','geometry_msgs/msg/PoseStamped'),(rgb,'sensor_msgs/msg/CompressedImage')]:writer.create_topic(rosbag2_py.TopicMetadata(name=name,type=kind,serialization_format='cdr'))
  stream=(out/'samples.jsonl').open('w');state.update(writer=writer,stream=stream,last={});counts=Counter();reasons=Counter();p=[];q=[];vid=[]
  for i,ns in enumerate(stamps):
   rp.clock.ns=ns;state['last']={};cm=colors[ns];im=cv2.imdecode(np.frombuffer(cm.data,np.uint8),cv2.IMREAD_COLOR);dm=decode_depth(depths[ns])
   if im.shape[:2]!=dm.shape:raise ValueError('Unaligned image dimensions')
   def message(data,enc,step):
    m=Image();m.header=cm.header;m.height,m.width=data.shape[:2];m.encoding=enc;m.step=step;m.data=array('B',data.tobytes());return m
   if node.arm.scale is None:
    body=node.body.process(cv2.cvtColor(im,cv2.COLOR_BGR2RGB))
    if body.pose_landmarks:
     lm=body.pose_landmarks.landmark[12];u,v=lm.x*im.shape[1],lm.y*im.shape[0];z=tracking.robust_depth(dm,int(u),int(v),4,(.3,3.)) if 4<=u<im.shape[1]-4 and 4<=v<im.shape[0]-4 else None
     if lm.visibility>=.7 and z is not None:
      node.arm.length=arm;node.arm.scale=node.arm.reach/arm;node.arm.shoulder=np.array([(u-node.k[0,2])*z/node.k[0,0],(v-node.k[1,2])*z/node.k[1,1],z])
   node.frame(message(im,'bgr8',im.shape[1]*3),message(dm,'32FC1',dm.shape[1]*4))
   writer.write(rgb,serialize_message(cm),ns)
   sample=state['last'].get('/fr3_operator/teleop_sample',{});d=state['last'].get('/fr3_operator/tracking',{})
   counts['frames']+=1
   for key in ['position_valid','orientation_valid']:counts[key]+=bool(sample.get(key))
   reasons[d.get('orientation_reason') or d.get('reason')]+=1
   if sample.get('p'):p.append([ns,*sample['p']])
   if sample.get('q'):q.append([ns,*sample['q']])
   if node.preview_hand is not None and node.preview_hand[0]==ns:
    px=node.preview_hand[1].astype(int)
    for b,c in tracking.mp.solutions.hands.HAND_CONNECTIONS:cv2.line(im,tuple(px[b]),tuple(px[c]),(50,220,210),2)
   axes=node.preview_axes
   if d.get('orientation_valid') and axes is not None and axes[0]==ns:
    origin,rotation=axes[1:]
    def project(p):return tuple(np.rint([node.k[0,0]*p[0]/p[2]+node.k[0,2],node.k[1,1]*p[1]/p[2]+node.k[1,2]]).astype(int))
    for axis,color in enumerate([(0,0,255),(0,255,0),(255,0,0)]):cv2.arrowedLine(im,project(origin),project(origin+.08*rotation[:,axis]),color,2)
   im=cv2.resize(im,(640,360));cv2.rectangle(im,(0,0),(640,48),(0,0,0),-1)
   label='6D OK' if d.get('orientation_valid') else 'POSITION ONLY' if d.get('position_valid') else 'NO POSE'
   cv2.putText(im,stage.name+' | '+label,(8,18),cv2.FONT_HERSHEY_SIMPLEX,.48,(255,255,255),1)
   cv2.putText(im,str(d.get('orientation_reason') or d.get('reason',''))[:85],(8,40),cv2.FONT_HERSHEY_SIMPLEX,.35,(0,200,255),1)
   path=frames/f'{i:06}.jpg';cv2.imwrite(str(path),im);vid.append((path.name,ns))
   if i%150==0:print('PROGRESS',stage.name,i,len(stamps),dict(counts),flush=True)
  stream.close();state.clear();del writer
  np.savez_compressed(out/'trajectory.npz',p=np.asarray(p),q=np.asarray(q))
  concat=frames/'list.txt';concat.write_text(''.join("file '"+name+"'\nduration "+str((vid[i+1][1]-ns)/1e9 if i+1<len(vid) else .05)+'\n' for i,(name,ns) in enumerate(vid)))
  # Host encoder preserves capture intervals (container FFmpeg lacks libx264).
  thumbs=[cv2.imread(str(frames/vid[int(j*(len(vid)-1)/8)][0])) for j in range(9)];cv2.imwrite(str(out/'timeline.jpg'),np.hstack([cv2.resize(im,(160,90)) for im in thumbs]))
  summary=dict(stage=stage.name,source=str(stage),arm_length_m=arm,source_rgb=len(colors),source_depth=len(depths),paired_frames=len(stamps),counts=dict(counts),reasons=reasons.most_common(),duration_s=(stamps[-1]-stamps[0])*1e-9,capture_start_ns=stamps[0],capture_end_ns=stamps[-1],replay_only=True)
  observations.save();node.destroy_node();(out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2));print('DONE',json.dumps(summary),flush=True)
 rclpy.shutdown()
if __name__=='__main__':main()
