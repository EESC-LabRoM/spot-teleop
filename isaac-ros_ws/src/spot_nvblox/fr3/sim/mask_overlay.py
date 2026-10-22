import time, numpy as np, cv2, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
rclpy.init(); n = Node("mask_overlay"); rgb, msk = {}, {}
for c in ("zed1", "zed2"):
    n.create_subscription(Image, f"/{c}/zed_node/rgb/image_rect_color", lambda m, c=c: rgb.__setitem__(c, m), qos_profile_sensor_data)
    n.create_subscription(Image, f"/{c}/arm_filter/mask", lambda m, c=c: msk.__setitem__(c, m), qos_profile_sensor_data)
t0 = time.monotonic()
while time.monotonic() - t0 < 6: rclpy.spin_once(n, timeout_sec=0.05)
tiles = []
for c in ("zed1", "zed2"):
    if c not in rgb or c not in msk: print(c, "missing", c in rgb, c in msk); continue
    r = np.frombuffer(rgb[c].data, np.uint8).reshape(rgb[c].height, rgb[c].width, -1)[:, :, :3].copy()
    k = np.frombuffer(msk[c].data, np.uint8).reshape(msk[c].height, msk[c].width)
    o = r.copy(); o[k > 0] = (0.45 * o[k > 0] + 0.55 * np.array([0, 0, 255])).astype(np.uint8)
    cs, _ = cv2.findContours(k, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE); cv2.drawContours(o, cs, -1, (0, 255, 255), 1)
    cv2.putText(o, c, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    tiles.append(np.hstack([r, o])); print(c, f"robot {100 * (k > 0).mean():.0f}% of image")
cv2.imwrite("/tmp/sim/mask_overlay.png", np.vstack(tiles))
