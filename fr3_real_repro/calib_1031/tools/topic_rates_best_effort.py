import time, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
rclpy.init(); n = Node("claude_sizes"); be = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
S = {}
def mk(t):
    S[t] = dict(n=0, wh=None, t0=None, stat="")
    def cb(m):
        s = S[t]; s["n"] += 1; s["wh"] = (m.width, m.height)
        if s["t0"] is None: s["t0"] = time.time()
        if m.encoding == "32FC1" and s["n"] % 10 == 1:
            d = np.frombuffer(bytes(m.data), np.float32).reshape(m.height, m.width); ok = np.isfinite(d) & (d > 0.3)
            s["stat"] = f"valid {100*ok.mean():.0f}%, median {np.median(d[ok]):.2f} m" if ok.any() else "no valid depth"
    return cb
for t in ("/zed2/zed_node/left/image_rect_color", "/zed2/zed_node/depth/depth_registered", "/franka_seg_zed2/depth", "/zed1/zed_node/depth/depth_registered", "/franka_seg_zed1/depth"):
    n.create_subscription(Image, t, mk(t), be)
S["/zed2/zed_node/depth/camera_info"] = dict(n=0, wh=None, t0=None, stat=""); 
def ci(m): s = S["/zed2/zed_node/depth/camera_info"]; s["n"] += 1; s["wh"] = (m.width, m.height); s["stat"] = f"fx {m.k[0]:.1f} cx {m.k[2]:.1f}"
n.create_subscription(CameraInfo, "/zed2/zed_node/depth/camera_info", ci, be)
t0 = time.time()
while time.time() - t0 < 9: rclpy.spin_once(n, timeout_sec=0.2)
for t, s in S.items(): print(f"{s['n']/9:5.1f} Hz  {str(s['wh']):12s} {t}   {s['stat']}")
