"""Listen to the ZED wrapper's colour + depth topics for --seconds, report rates and save the last frame of each.
   docker run --rm --network host --ipc host -e ROS_DOMAIN_ID=8 -v $D:/out -v $D/install:/ws/install:ro zed-lite:5.4.1-humble \
     bash -c "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && python3 /out/check_topics.py --out /out/check"
"""
import argparse, json, os, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

ap = argparse.ArgumentParser()
ap.add_argument("--out", required=True)
ap.add_argument("--seconds", type=float, default=15.0)
ap.add_argument("--ns", default="/zed/zed_node")
a = ap.parse_args(); os.makedirs(a.out, exist_ok=True)
TOPICS = {"rgb": f"{a.ns}/rgb/color/rect/image", "depth": f"{a.ns}/depth/depth_registered", "info": f"{a.ns}/depth/depth_info"}


class Check(Node):
    def __init__(self):
        super().__init__("zed_topic_check")
        self.t = {k: [] for k in ("rgb", "depth", "info")}; self.last = {}
        self.create_subscription(Image, TOPICS["rgb"], lambda m: self.cb("rgb", m), qos_profile_sensor_data)
        self.create_subscription(Image, TOPICS["depth"], lambda m: self.cb("depth", m), qos_profile_sensor_data)
        self.create_subscription(CameraInfo, f"{a.ns}/depth/camera_info", lambda m: self.cb("info", m), qos_profile_sensor_data)

    def cb(self, k, m):
        self.t[k].append(time.time()); self.last[k] = m


rclpy.init(); n = Check(); t0 = time.time()
while time.time() - t0 < a.seconds:
    rclpy.spin_once(n, timeout_sec=0.1)
res = {"topics": {k: TOPICS[k] if k != "info" else f"{a.ns}/depth/camera_info" for k in n.t}}
for k, ts in n.t.items():
    res[k] = {"msgs": len(ts), "hz": round((len(ts) - 1) / (ts[-1] - ts[0]), 2) if len(ts) > 1 else 0.0}
if "rgb" in n.last:
    m = n.last["rgb"]; ch = 4 if m.encoding in ("bgra8", "rgba8") else 3
    img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, ch)[..., :3]
    if m.encoding.startswith("bgr"): img = img[..., ::-1]
    np.save(os.path.join(a.out, "rgb.npy"), img); res["rgb"].update(encoding=m.encoding, size=[m.width, m.height], frame=m.header.frame_id)
if "depth" in n.last:
    m = n.last["depth"]; d = np.frombuffer(m.data, np.float32 if m.encoding == "32FC1" else np.uint16).reshape(m.height, m.width)
    d = d.astype(np.float32) / (1000.0 if m.encoding != "32FC1" else 1.0)
    np.save(os.path.join(a.out, "depth_m.npy"), d); ok = np.isfinite(d) & (d > 0)
    res["depth"].update(encoding=m.encoding, size=[m.width, m.height], valid_frac=round(float(ok.mean()), 3),
                        median_m=round(float(np.median(d[ok])), 3) if ok.any() else None)
if "info" in n.last:
    m = n.last["info"]; res["info"].update(K=[round(float(v), 2) for v in m.k], size=[m.width, m.height], frame=m.header.frame_id)
json.dump(res, open(os.path.join(a.out, "topics.json"), "w"), indent=1); print(json.dumps(res))
os.system(f"chown -R 1000:1000 {a.out}")
