"""Which mask did nvblox actually apply? Compare each /nvblox_node/dynamic_depth_frame_overlay (frame_id = the camera it integrated; masked
pixels have r=255 and g<255) with the latest /zed/target_mask and /zed2/target_mask: IoU with the own camera's mask vs the other camera's."""
import json, sys, time
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
SECS = float(sys.argv[1]) if len(sys.argv) > 1 else 40
def iou(a, b):
    u = (a | b).sum(); return float((a & b).sum() / u) if u else float("nan")
class P(Node):
    def __init__(self):
        super().__init__("overlay_probe"); self.m = {}; self.rows = []
        q = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE)
        for c in ("zed", "zed2"):
            self.create_subscription(Image, f"/{c}/target_mask", lambda m, c=c: self.m.__setitem__(c, np.frombuffer(m.data, np.uint8).reshape(m.height, m.width) > 0), q)
        self.create_subscription(Image, "/nvblox_node/dynamic_depth_frame_overlay", self.on_ov, q)
    def on_ov(self, m):
        if len(self.m) < 2: return
        ch = {"rgba8": 4, "rgb8": 3, "bgra8": 4, "bgr8": 3}.get(m.encoding, 4)
        im = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, ch)
        red = (im[..., 0] == 255) & (im[..., 1] < 255) if m.encoding.startswith("rgb") else (im[..., 2] == 255) & (im[..., 1] < 255)
        own = "zed2" if "zed2" in m.header.frame_id else "zed"; oth = "zed" if own == "zed2" else "zed2"
        if self.m[own].shape != red.shape or self.m[oth].shape != red.shape: return
        self.rows.append({"cam": own, "masked_px": int(red.sum()), "own_mask_px": int(self.m[own].sum()), "iou_own": round(iou(red, self.m[own]), 3), "iou_other": round(iou(red, self.m[oth]), 3)})
rclpy.init(); n = P(); t0 = time.time()
while time.time() - t0 < SECS: rclpy.spin_once(n, timeout_sec=0.1)
R = n.rows; print(json.dumps(R[:12]))
for c in ("zed", "zed2"):
    r = [x for x in R if x["cam"] == c]
    if not r: print(c, "no overlays"); continue
    io, ot = np.array([x["iou_own"] for x in r]), np.array([x["iou_other"] for x in r])
    print(f"{c}: {len(r)} overlays | IoU with own mask median {np.nanmedian(io):.3f}, frames with IoU<0.5: {int((io < 0.5).sum())} | IoU with the other camera's mask median {np.nanmedian(ot):.3f}, frames >0.5: {int((ot > 0.5).sum())}")
