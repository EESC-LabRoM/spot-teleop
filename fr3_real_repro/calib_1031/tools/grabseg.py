# robot-free depth (what nvblox integrates) from both segmenters -> base-frame points, median of N frames per pixel; K from the live camera_info, pose from TF
import time, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import Buffer, TransformListener
rclpy.init(); n = Node("claude_grabseg"); tf = Buffer(); TransformListener(tf, n)
be = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT); D = {c: [] for c in ("zed1", "zed2")}; INFO = {}
for c in D:
    n.create_subscription(Image, f"/franka_seg_{c}/depth", lambda m, c=c: D[c].append(np.frombuffer(bytes(m.data), np.float32).reshape(m.height, m.width).copy()) if len(D[c]) < 12 else None, be)
    n.create_subscription(CameraInfo, f"/{c}/zed_node/depth/camera_info", lambda m, c=c: INFO.__setitem__(c, np.array(m.k).reshape(3, 3)), be)
t0 = time.time()
while time.time() - t0 < 25 and (min(len(v) for v in D.values()) < 12 or len(INFO) < 2): rclpy.spin_once(n, timeout_sec=0.1)
out = {}
for c in D:
    if len(D[c]) < 4: print(c, "too few frames", len(D[c])); continue
    d = np.nanmedian(np.where(np.isfinite(np.stack(D[c])) & (np.stack(D[c]) > 0.3), np.stack(D[c]), np.nan), axis=0); K = INFO[c]; H, W = d.shape
    vv, uu = np.mgrid[0:H, 0:W]; ok = np.isfinite(d) & (d < 3.5); X = np.stack([(uu - K[0, 2]) * d / K[0, 0], (vv - K[1, 2]) * d / K[1, 1], d], -1)[ok]
    t = tf.lookup_transform("fr3_link0", f"{c}_left_camera_optical_frame", rclpy.time.Time()).transform; q = t.rotation; w, x, y, z = q.w, q.x, q.y, q.z
    R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)], [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    P = X @ R.T + np.array([t.translation.x, t.translation.y, t.translation.z]); out[c] = P.astype(np.float32); print(c, len(D[c]), "frames,", len(P), "points, K fx", round(float(K[0, 0]), 1))
np.savez("/tmp/claude_segcloud.npz", **out)
