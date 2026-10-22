"""publish one seed on /perception/seed_command, as vlm_relocalize_node does after the Qwen call (bbox and grasp points in 0..1000).
  python3 send_seed.py x1 y1 x2 y2 [gx gy] [--label drill]"""
import argparse, json, time, rclpy
from std_msgs.msg import String
ap = argparse.ArgumentParser(); ap.add_argument("box", type=float, nargs=4); ap.add_argument("--grasp", type=float, nargs=2); ap.add_argument("--label", default="object")
a, ros = ap.parse_known_args()
rclpy.init(args=ros); n = rclpy.create_node("send_seed"); pub = n.create_publisher(String, "/perception/seed_command", 10)
seed = dict(bbox_1000=a.box, grasps_1000=[a.grasp] if a.grasp else [], label=a.label, confidence=1.0, req_id="manual")
t0 = time.monotonic()
while pub.get_subscription_count() == 0 and time.monotonic() - t0 < 5: rclpy.spin_once(n, timeout_sec=0.1)
pub.publish(String(data=json.dumps(seed))); rclpy.spin_once(n, timeout_sec=0.3); print("seed sent:", seed, "subs", pub.get_subscription_count())
