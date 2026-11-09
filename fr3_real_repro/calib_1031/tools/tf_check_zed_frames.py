import time, rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
rclpy.init(); n = Node("claude_tfchk"); b = Buffer(); TransformListener(b, n); t0 = time.time()
while time.time() - t0 < 6: rclpy.spin_once(n, timeout_sec=0.2)
for c in ("zed1", "zed2"):
    fr = f"{c}_left_camera_optical_frame"
    try:
        t = b.lookup_transform("fr3_link0", fr, rclpy.time.Time()); p = t.transform.translation; print(f"{fr}: OK  t=({p.x:.3f},{p.y:.3f},{p.z:.3f})")
    except Exception as e: print(f"{fr}: FAIL {str(e)[:150]}")
print("\n".join(l for l in b.all_frames_as_string().splitlines() if "zed" in l or "base" in l))
