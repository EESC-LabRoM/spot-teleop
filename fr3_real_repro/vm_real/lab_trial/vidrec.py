# two ZED left images side by side + clock -> mp4v; run inside zed-build:  python3 vidrec.py <out.mp4> <seconds> [fps]
import sys, time, numpy as np, cv2, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
out, secs = sys.argv[1], float(sys.argv[2]); fps = float(sys.argv[3]) if len(sys.argv) > 3 else 15.0
rclpy.init(); n = Node("claude_vidrec"); L = {}
for c in ("zed1", "zed2"):
    def cb(m, c=c): L[c] = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, -1)[:, :, :3].copy()
    n.create_subscription(Image, f"/{c}/zed_node/left/image_rect_color", cb, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
t0 = time.time(); w = None; k = 0; nxt = t0
while time.time() - t0 < secs:
    rclpy.spin_once(n, timeout_sec=0.01)
    if time.time() < nxt or len(L) < 2: continue
    nxt += 1.0 / fps; a, b = L["zed1"], L["zed2"]; h = 360; a = cv2.resize(a, (int(a.shape[1] * h / a.shape[0]), h)); b = cv2.resize(b, (int(b.shape[1] * h / b.shape[0]), h)); f = np.hstack([a, b])
    cv2.putText(f, f"zed1 (front)", (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2); cv2.putText(f, f"zed2 (top)", (a.shape[1] + 8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    cv2.putText(f, f"t = {time.time()-t0:6.1f} s", (8, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    if w is None: w = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*"mp4v"), fps, (f.shape[1], f.shape[0]))
    w.write(f); k += 1
if w: w.release()
print(f"vidrec: {k} frames -> {out}", flush=True)
