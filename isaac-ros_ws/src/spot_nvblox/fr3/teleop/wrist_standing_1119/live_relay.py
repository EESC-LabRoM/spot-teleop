"""Perception-only live sample relay, original acquisition stamps retained."""
import json,socket,time
import os
os.environ.setdefault("FASTRTPS_DEFAULT_PROFILES_FILE","/lab/fr3_zed_runtime/live_transport.xml")
import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
rclpy.init(domain_id=8);n=rclpy.create_node('standing_wrist_sample_relay');sock=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM);sock.setblocking(False);counts={'sent':0,'stale_or_invalid':0,'socket_busy':0}
def cb(msg):
 try:
  d=json.loads(msg.data);age=(time.time_ns()-int(d['stamp_ns']))*1e-9
  if d.get('version')!=3 or d.get('position_valid') is not True or not 0<=age<.35:counts['stale_or_invalid']+=1;return
  sock.sendto(msg.data.encode(),'/lab/wrist_standing_1119/input.sock');counts['sent']+=1
 except BlockingIOError:counts['socket_busy']+=1
 except (ValueError,TypeError,KeyError):counts['stale_or_invalid']+=1
n.create_subscription(String,'/wrist_standing/sample',cb,qos_profile_sensor_data)
try:
 end=time.monotonic()+185
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.03)
finally:
 print(json.dumps(counts),flush=True);sock.close();n.destroy_node();rclpy.shutdown()
