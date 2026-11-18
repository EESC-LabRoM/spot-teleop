"""Fit the operator's arm length from one RGB-D bag; never imports robot control."""
import argparse,json,sys,hashlib
from pathlib import Path
import cv2,numpy as np,mediapipe as mp,rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage,CameraInfo
ROOT=Path(__file__).resolve().parent;sys.path.insert(0,str(ROOT/'runtime'))
from arm_calibration import ArmCalibration
from arm_pose_estimator.hand_frame import robust_depth
from depth_codec import decode_depth
p=argparse.ArgumentParser();p.add_argument('--bag',required=True);p.add_argument('--out',required=True);a=p.parse_args()
rgb='/zed3/zed_node/rgb/image_rect_color/compressed';depth='/zed3/zed_node/depth/depth_registered/compressedDepth';info='/zed3/zed_node/depth/camera_info'
r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=a.bag,storage_id='mcap'),rosbag2_py.ConverterOptions('cdr','cdr'));r.set_filter(rosbag2_py.StorageFilter(topics=[rgb,depth,info]));colors={};depths={};K=None
while r.has_next():
 topic,data,_=r.read_next()
 if topic==info:
  if K is None:K=np.array(deserialize_message(data,CameraInfo).k).reshape(3,3)
 else:
  m=deserialize_message(data,CompressedImage);stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9;(colors if topic==rgb else depths)[stamp]=m
pose=mp.solutions.pose.Pose(model_complexity=1,enable_segmentation=False,min_detection_confidence=.5,min_tracking_confidence=.5);arm=ArmCalibration();last=-float('inf');observations=[];latched=None
for t in sorted(colors.keys()&depths.keys()):
 if t-last<.18:continue
 last=t;im=cv2.imdecode(np.frombuffer(colors[t].data,np.uint8),cv2.IMREAD_COLOR);D=decode_depth(depths[t]);result=pose.process(cv2.cvtColor(im,cv2.COLOR_BGR2RGB))
 if not result.pose_landmarks:continue
 marks=result.pose_landmarks.landmark;xyz=[]
 for idx in (12,14,16):
  pt=marks[idx];u,v=pt.x*im.shape[1],pt.y*im.shape[0]
  if pt.visibility<.7 or not 4<=u<im.shape[1]-4 or not 4<=v<im.shape[0]-4:break
  z=robust_depth(D,int(u),int(v),4,(.3,3.))
  if z is None:break
  xyz.append([(u-K[0,2])*z/K[0,0],(v-K[1,2])*z/K[1,1],z])
 if len(xyz)==3 and arm.observe(*xyz,t):
  observations.append(dict(stamp=t,xyz=xyz))
  if arm.scale is not None and latched is None:latched=t
pose.close()
if arm.scale is None:raise RuntimeError('Donor bag did not support a stable arm calibration')
out=dict(version=1,operator_id='recorded_operator_20261113',arm_length_m=arm.length,arm_upper_m=arm.upper_length,arm_forearm_m=arm.fore_length,robot_reach_m=arm.reach,arm_scale=arm.scale,donor_bag=Path(a.bag).parent.parent.name,calibrated_at_capture_s=latched,valid_arm_observations=len(observations),postfit_check_length_m=arm.check_length,note='Arm lengths only; body axes and origin must be re-established from fresh landmarks. No robot/camera calibration.')
Path(a.out).write_text(json.dumps(out,indent=2)+'\n');Path(a.out).with_suffix('.observations.json').write_text(json.dumps(observations,indent=2)+'\n');print(json.dumps(out,indent=2))
