"""dump the live ZED depth camera_info of zed1/zed2 to JSON (read-only helper for the simulated cameras)."""
import json, sys, time, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
rclpy.init(); n = Node("dump_cams"); out = {}
for c in ("zed1", "zed2"):
    n.create_subscription(CameraInfo, f"/{c}/zed_node/depth/camera_info", lambda m, c=c: out.__setitem__(c, dict(
        width=m.width, height=m.height, k=list(m.k), p=list(m.p), d=list(m.d), distortion_model=m.distortion_model)), qos_profile_sensor_data)
t0 = time.monotonic()
while len(out) < 2 and time.monotonic() - t0 < 10: rclpy.spin_once(n, timeout_sec=0.1)
json.dump(out, open(sys.argv[1], "w"), indent=1); print({k: (v["width"], v["height"], round(v["k"][0], 1)) for k, v in out.items()})
