"""Probe nvblox's layers around the grasp target: obstacle points of the static / dynamic / combined ESDF point clouds inside a box
(robot base frame), and one frame of the dynamic-depth/color overlays (nvblox's own masked-vs-static decision).
   python3 probe_layers.py --box x0 y0 z0 x1 y1 z1 --out /ws/nvblox/out/diag [--node nvblox_node]"""
import argparse, json, sys, time
import numpy as np, rclpy, cv2
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
ap = argparse.ArgumentParser(); ap.add_argument("--box", type=float, nargs=6, required=True); ap.add_argument("--out", required=True)
ap.add_argument("--node", default="nvblox_node"); ap.add_argument("--wait", type=float, default=40.0)
a = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:]); lo, hi = np.array(a.box[:3]), np.array(a.box[3:])
rclpy.init(); n = Node("probe_layers"); got = {}
def cloud_cb(name):
    def f(m):
        P = np.array([[p[0], p[1], p[2], p[3] if len(p) > 3 else 0.0] for p in pc2.read_points(m, field_names=[f.name for f in m.fields][:4], skip_nans=True)], float) if m.width * m.height else np.zeros((0, 4))
        got[name] = (m.header.frame_id, [f.name for f in m.fields], P)
    return f
def img_cb(name):
    def f(m):
        ch = 3 if "8" in m.encoding and m.step // m.width == 3 else (4 if m.step // m.width == 4 else 1)
        im = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, ch) if ch > 1 else np.frombuffer(m.data, np.uint8).reshape(m.height, m.width)
        cv2.imwrite(f"{a.out}_{name}.jpg", im[..., :3] if ch > 1 else im); got[name] = (m.header.frame_id, m.encoding, im.shape)
    return f
for rel in (ReliabilityPolicy.RELIABLE, ReliabilityPolicy.BEST_EFFORT):
    q = QoSProfile(depth=2, reliability=rel, durability=DurabilityPolicy.VOLATILE)
    for nm in ("static_esdf_pointcloud", "dynamic_esdf_pointcloud", "combined_esdf_pointcloud", "pessimistic_static_esdf_pointcloud"):
        n.create_subscription(PointCloud2, f"/{a.node}/{nm}", cloud_cb(nm), q)
    for nm in ("dynamic_depth_frame_overlay", "dynamic_color_frame_overlay"):
        n.create_subscription(Image, f"/{a.node}/{nm}", img_cb(nm), q)
t0 = time.time()
while time.time() - t0 < a.wait and len(got) < 6: rclpy.spin_once(n, timeout_sec=0.5)
res = {}
for k, v in got.items():
    if isinstance(v[2], np.ndarray):
        fr, fields, P = v; inb = np.all((P[:, :3] > lo) & (P[:, :3] < hi), 1) if len(P) else np.zeros(0, bool)
        res[k] = {"frame": fr, "fields": fields, "points": int(len(P)), "points_in_box": int(inb.sum()),
                  "intensity_in_box_min_med_max": [round(float(x), 3) for x in (P[inb, 3].min(), np.median(P[inb, 3]), P[inb, 3].max())] if inb.any() else None}
    else: res[k] = {"frame": v[0], "encoding": v[1], "shape": list(v[2])}
print(json.dumps(res, indent=1)); json.dump(res, open(a.out + "_layers.json", "w"), indent=1)
