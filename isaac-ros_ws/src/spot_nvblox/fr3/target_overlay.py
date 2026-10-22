"""debug image: each ZED's colour with the object mask (sam2_tracker_node, green) and the depth-filter mask (arm + object, red outline)."""
import sys, time, numpy as np, cv2, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
rclpy.init(); n = Node("target_overlay"); rgb, tgt, flt = {}, {}, {}
for c in ("zed1", "zed2"):
    n.create_subscription(Image, f"/{c}/zed_node/rgb/image_rect_color", lambda m, c=c: rgb.__setitem__(c, m), qos_profile_sensor_data)
    n.create_subscription(Image, f"/{c}/segmentation_mask", lambda m, c=c: tgt.__setitem__(c, m), 10)
    n.create_subscription(Image, f"/{c}/arm_filter/mask", lambda m, c=c: flt.__setitem__(c, m), qos_profile_sensor_data)
t0 = time.monotonic()
while time.monotonic() - t0 < 6: rclpy.spin_once(n, timeout_sec=0.05)
tiles = []
for c in ("zed1", "zed2"):
    r = np.frombuffer(rgb[c].data, np.uint8).reshape(rgb[c].height, rgb[c].width, -1)[:, :, :3].copy(); o = r.copy()
    if c in tgt:
        k = np.frombuffer(tgt[c].data, np.uint8).reshape(tgt[c].height, tgt[c].width) > 0
        o[k] = (0.4 * o[k] + 0.6 * np.array([0, 255, 0])).astype(np.uint8); print(c, f"object mask {100 * k.mean():.1f}% of image")
    else: print(c, "no object mask")
    if c in flt:
        f = (np.frombuffer(flt[c].data, np.uint8).reshape(flt[c].height, flt[c].width) > 0).astype(np.uint8)
        cs, _ = cv2.findContours(f, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE); cv2.drawContours(o, cs, -1, (0, 0, 255), 2)
    cv2.putText(o, c, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2); tiles.append(np.hstack([r, o]))
cv2.imwrite(sys.argv[1] if len(sys.argv) > 1 else "/tmp/target_overlay.png", np.vstack(tiles))
