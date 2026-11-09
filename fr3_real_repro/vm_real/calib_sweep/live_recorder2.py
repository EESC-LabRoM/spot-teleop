"""PASSIVE live calibration-data recorder (no robot commands, no ROS publishers except none). Watches the real arm's link poses
(dashboard SSE, read-only) and, whenever the arm is still for >= 1 s and in a pose that differs enough from the ones already saved,
stores both ZED left rectified images + depth + the link poses. Stop with SIGTERM. Output /tmp/live_calib/NNN/."""
import json, os, sys, threading, time, urllib.request, collections
import numpy as np, cv2, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
OUT = sys.argv[2] if len(sys.argv) > 2 else "/tmp/live_calib"; os.makedirs(OUT, exist_ok=True); MAXN = int(sys.argv[1]) if len(sys.argv) > 1 else 60   # usage: live_recorder.py [max_saves] [out_dir]
hist = collections.deque(maxlen=400); last_msg = {}; lock = threading.Lock()
def sse():
    while True:
        try:
            r = urllib.request.urlopen("http://127.0.0.1:8088/stream", timeout=10)
            while True:
                l = r.readline().decode()
                if not l: break
                if l.startswith("data: "):
                    m = json.loads(l[6:])
                    with lock: hist.append((m["t"], np.array(m["tcp"][:3]), np.array(m["links"]["fr3_link7"][:3]), m)); last_msg["m"] = m
        except Exception as e:
            time.sleep(1)
threading.Thread(target=sse, daemon=True).start()
rclpy.init(); node = Node("live_recorder"); img = {}; dep = {}; ci = {}
def mk(store, cam, rgb):
    def cb(m):
        store[cam] = m
    return cb
for cam in ("zed1", "zed2"):
    node.create_subscription(Image, f"/{cam}/zed_node/left/image_rect_color", mk(img, cam, True), qos_profile_sensor_data)
    node.create_subscription(Image, f"/{cam}/zed_node/depth/depth_registered", mk(dep, cam, False), qos_profile_sensor_data)
    node.create_subscription(CameraInfo, f"/{cam}/zed_node/left/camera_info", mk(ci, cam, False), qos_profile_sensor_data)
def q2R(q):
    x, y, z, w = q; return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)], [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
saved = []; n = 0; t_start = time.time(); last_log = 0
print("recorder up; waiting for the arm to be still and in a new pose", flush=True)
while rclpy.ok() and n < MAXN:
    rclpy.spin_once(node, timeout_sec=0.05)
    with lock: H = [h for h in hist if h[0] > (hist[-1][0] - 1.0)] if hist else []
    if len(H) < 15 or H[-1][0] - H[0][0] < 0.9: continue
    spread = max(np.linalg.norm(np.array([h[1] for h in H]).max(0) - np.array([h[1] for h in H]).min(0)), np.linalg.norm(np.array([h[2] for h in H]).max(0) - np.array([h[2] for h in H]).min(0)))
    if time.time() - last_log > 20:
        last_log = time.time(); print(f"{time.time()-t_start:5.0f}s saved {n}, still-spread {spread*1000:.1f} mm", flush=True)
    if spread > 0.0015: continue
    m = H[-1][3]; tcp = np.array(m["tcp"]); R = q2R(tcp[3:7]); p = tcp[:3]
    LK = [k for k in ("fr3_link3", "fr3_link4", "fr3_link5", "fr3_link6", "fr3_link7", "fr3_hand") if k in m["links"]]   # v2: ELBOW/forearm differences count too (the original compared only the TCP pose)
    diverse = all(np.linalg.norm(p - s[0]) > 0.06 or np.degrees(np.arccos(np.clip((np.trace(R.T @ s[1]) - 1) / 2, -1, 1))) > 12 or max(np.linalg.norm(np.array(m["links"][k][:3]) - np.array(s[2][k][:3])) for k in LK) > 0.05 for s in saved)
    if not diverse: continue
    if not all(c in img and c in dep and c in ci for c in ("zed1", "zed2")): continue
    # frames must be fresh (arm still for >= 1 s, so any frame of the last second shows the same pose)
    d = f"{OUT}/{n:03d}"; os.makedirs(d, exist_ok=True)
    for cam in ("zed1", "zed2"):
        mi, md, mc = img[cam], dep[cam], ci[cam]
        a = np.frombuffer(mi.data, np.uint8).reshape(mi.height, mi.width, -1)[:, :, :3]; cv2.imwrite(f"{d}/{cam}_left.png", a)
        np.savez_compressed(f"{d}/{cam}_depth.npz", depth=np.frombuffer(md.data, np.float32).reshape(md.height, md.width).copy(), stamp=md.header.stamp.sec + md.header.stamp.nanosec * 1e-9)
        json.dump(dict(K=list(mc.k), D=list(mc.d), model=mc.distortion_model, w=mc.width, h=mc.height, stamp=mi.header.stamp.sec + mi.header.stamp.nanosec * 1e-9), open(f"{d}/{cam}_info.json", "w"))
    json.dump(dict(t=m["t"], links=m["links"], tcp=m["tcp"]), open(f"{d}/robot.json", "w"))
    saved.append((p, R, {k: list(v) for k, v in m["links"].items()})); n += 1; print(f"saved #{n}: TCP {np.round(p,3).tolist()} (spread {spread*1000:.2f} mm)", flush=True)
print("done", n, flush=True)
