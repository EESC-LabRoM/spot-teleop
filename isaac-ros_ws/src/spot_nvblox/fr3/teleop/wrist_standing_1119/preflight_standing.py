"""Read-only operator readiness. Never engages or changes a controller."""
import json,time
import os
os.environ.setdefault("FASTRTPS_DEFAULT_PROFILES_FILE","/lab/fr3_zed_runtime/live_transport.xml")
import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
rclpy.init(domain_id=8);n=rclpy.create_node('standing_operator_readiness');box={}
def cb(msg):
 try:box.update(diag=json.loads(msg.data),received=time.monotonic())
 except ValueError:pass
n.create_subscription(String,'/wrist_standing/tracking',cb,qos_profile_sensor_data)
try:
 end=time.monotonic()+4
 while time.monotonic()<end and 'diag' not in box:rclpy.spin_once(n,timeout_sec=.05)
 d=box.get('diag',{});age=(time.time_ns()-int(d.get('stamp_ns',0)))*1e-9
 ok=0<=age<.35 and d.get('arm_calibrated') is True and d.get('body_reference_ready') is True and d.get('position_valid') is True
 print(json.dumps({'ready':ok,'capture_age_s':age,'arm_length_m':d.get('arm_length_m'),'scale':d.get('arm_scale'),'reason':d.get('reason'),'body_reference_ready':d.get('body_reference_ready')}))
 if not ok:raise SystemExit(2)
finally:n.destroy_node();rclpy.shutdown()
