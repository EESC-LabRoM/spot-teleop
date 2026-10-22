"""Decode the notebook's ZED SDK stereo stream and publish aligned operator RGB-D (simulation DDS only)."""
import argparse,time
import numpy as np
import pyzed.sl as sl
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,CameraInfo
ap=argparse.ArgumentParser();ap.add_argument('--ip',default='100.118.129.74');ap.add_argument('--port',type=int,default=30000);a=ap.parse_args()
rclpy.init(domain_id=188);n=Node('remote_operator_camera')
image_pub=n.create_publisher(Image,'/fr3_sim/operator/rgb',qos_profile_sensor_data)
depth_pub=n.create_publisher(Image,'/fr3_sim/operator/depth',qos_profile_sensor_data)
info_pub=n.create_publisher(CameraInfo,'/fr3_sim/operator/camera_info',1)
cam=sl.Camera();init=sl.InitParameters();init.set_from_stream(a.ip,a.port)
init.depth_mode=sl.DEPTH_MODE.NEURAL_LIGHT;init.coordinate_units=sl.UNIT.METER
init.depth_minimum_distance=.3;init.depth_maximum_distance=3.;init.sdk_verbose=1
result=cam.open(init)
if result!=sl.ERROR_CODE.SUCCESS:raise RuntimeError(str(result))
res=sl.Resolution(640,360);rgb=sl.Mat();depth=sl.Mat();runtime=sl.RuntimeParameters()
config=cam.get_camera_information(res).camera_configuration;calib=config.calibration_parameters.left_cam
print('REMOTE ZED READY',config.resolution,calib.fx,calib.fy,flush=True)
count=0
try:
 while rclpy.ok():
  if cam.grab(runtime)!=sl.ERROR_CODE.SUCCESS:time.sleep(.01);continue
  cam.retrieve_image(rgb,sl.VIEW.LEFT,sl.MEM.CPU,res);cam.retrieve_measure(depth,sl.MEASURE.DEPTH,sl.MEM.CPU,res)
  stamp=cam.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds()
  ci=CameraInfo();ci.header.stamp.sec,ci.header.stamp.nanosec=divmod(stamp,10**9);ci.header.frame_id='operator_left_camera_optical_frame'
  ci.width=640;ci.height=360;ci.k=[float(calib.fx),0.,float(calib.cx),0.,float(calib.fy),float(calib.cy),0.,0.,1.]
  ci.p=[float(calib.fx),0.,float(calib.cx),0.,0.,float(calib.fy),float(calib.cy),0.,0.,0.,1.,0.];info_pub.publish(ci)
  color=Image();color.header=ci.header;color.width=640;color.height=360;color.encoding='bgra8';color.step=640*4;color.data=np.ascontiguousarray(rgb.get_data()).tobytes()
  dm=Image();dm.header=ci.header;dm.width=640;dm.height=360;dm.encoding='32FC1';dm.step=640*4;dm.data=np.ascontiguousarray(depth.get_data(),dtype=np.float32).tobytes()
  image_pub.publish(color);depth_pub.publish(dm);rclpy.spin_once(n,timeout_sec=0.)
  count+=1
  if count%75==0:print('frames',count,'age_ms',round((time.time_ns()-stamp)/1e6,1),flush=True)
finally:cam.close();n.destroy_node();rclpy.shutdown()
