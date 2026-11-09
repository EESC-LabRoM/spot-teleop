#!/usr/bin/env python3
"""PASSIVE odometry anchor + logger for the ZED cameras (publishes nothing, commands nothing).
Run in zed-build (ROS domain 8):  python3 odom_anchor.py [--new]
 * First start (or --new): writes an IMMUTABLE anchor_<UTC>.json pairing each camera's current ZED odometry pose (camera_link in the odom frame)
   with the calibration that is live right now (TF fr3_link0 -> camera_link/optical, calibration file md5), the raw IMU gravity, and a reference
   image + depth + camera_info (ref_<UTC>/), so the calibration can be re-verified or re-anchored visually after an odometry reset.
   ANCHOR_CURRENT names the active anchor. Later starts only RESUME logging against it (they never overwrite an anchor).
 * Continuously: odom_log.jsonl (every 2 s), last_good.json (every 5 s, atomic), events.log (JUMP / GAP / RESUME / RESET-TO-ORIGIN, epoch counter).
   The anchor is valid only while the camera's epoch is unchanged; an event bumps the epoch = continuity lost, correction by odometry no longer trusted.
Correction rule (apply_anchor.py):  T_base_link_now = T_base_link_ref @ inv(O_ref) @ O_now ;  T_base_optical_now = T_base_link_now @ T_link_optical."""
import os, sys, json, time, math, hashlib, datetime
import numpy as np, rclpy, cv2
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo, Imu
from tf2_ros import Buffer, TransformListener

DIR = os.environ.get("ANCHOR_DIR", "/home/spot_ws/zed_ws/calib_anchor"); os.makedirs(DIR, exist_ok=True)
CAMS = ("zed1", "zed2")
CALIB = {"zed1": "/tmp/nvblox_real/calib/zed1_front_camera_extrinsic.json", "zed2": "/tmp/nvblox_real/calib/zed2_top_camera_extrinsic.json"}
JUMP_M, JUMP_DEG, GAP_S, LOG_MAX = 0.30, 15.0, 5.0, 40e6
NEW = "--new" in sys.argv
def q2R(q):
    x, y, z, w = q; return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)], [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
def ang_between(qa, qb): return math.degrees(2 * math.acos(min(1.0, abs(float(np.dot(qa, qb))))))
def now_utc(): return datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
def atomic(path, obj):
    tmp = path + ".tmp"; json.dump(obj, open(tmp, "w"), indent=1); os.replace(tmp, path)
def ev(msg):
    line = f"{datetime.datetime.now(datetime.timezone.utc).isoformat(timespec='seconds')} {msg}"; open(f"{DIR}/events.log", "a").write(line + "\n"); print(line, flush=True)

rclpy.init(); node = Node("claude_odom_anchor"); buf = Buffer(); TransformListener(buf, node)
be = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
S = {c: dict(odom=None, prev=None, t_rx=0.0, epoch=0, gap=False, img=None, dep=None, info=None, acc=[]) for c in CAMS}
def on_odom(c):
    def cb(m):
        p = m.pose.pose.position; q = m.pose.pose.orientation; cur = (np.array([p.x, p.y, p.z]), np.array([q.x, q.y, q.z, q.w]), m.child_frame_id, m.header.frame_id)
        s = S[c]; tnow = time.time()
        if s["gap"]: s["gap"] = False; ev(f"{c} RESUME after gap")
        if s["prev"] is not None:
            dp = float(np.linalg.norm(cur[0] - s["prev"][0])); da = ang_between(cur[1], s["prev"][1])
            if dp > JUMP_M or da > JUMP_DEG or (np.linalg.norm(s["prev"][0]) > 1.0 and np.linalg.norm(cur[0]) < 0.2):
                s["epoch"] += 1; ev(f"{c} JUMP/RESET: step {dp:.3f} m / {da:.1f} deg, was {np.round(s['prev'][0],2).tolist()} now {np.round(cur[0],2).tolist()} -> epoch {s['epoch']} (odometry continuity LOST)")
        s["prev"] = cur; s["odom"] = cur; s["t_rx"] = tnow
    return cb
for c in CAMS:
    node.create_subscription(Odometry, f"/{c}/zed_node/odom", on_odom(c), be)
    node.create_subscription(Image, f"/{c}/zed_node/left/image_rect_color", lambda m, c=c: S[c].__setitem__("img", m), be)
    node.create_subscription(Image, f"/{c}/zed_node/depth/depth_registered", lambda m, c=c: S[c].__setitem__("dep", m), be)
    node.create_subscription(CameraInfo, f"/{c}/zed_node/left/camera_info", lambda m, c=c: S[c].__setitem__("info", m), be)
    node.create_subscription(Imu, f"/{c}/zed_node/imu/data", lambda m, c=c: (S[c]["acc"].append([m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z, m.header.frame_id]), S[c]["acc"].__delitem__(slice(0, -200))), be)
def spin(sec):
    t0 = time.time()
    while time.time() - t0 < sec: rclpy.spin_once(node, timeout_sec=0.05)
def tf4(a, b):
    tr = buf.lookup_transform(a, b, rclpy.time.Time()).transform; T = np.eye(4); r = tr.rotation
    T[:3, :3] = q2R([r.x, r.y, r.z, r.w]); T[:3, 3] = [tr.translation.x, tr.translation.y, tr.translation.z]; return T

spin(12)                                                   # let odom/images/imu/TF arrive
cur_file = f"{DIR}/ANCHOR_CURRENT"
if NEW or not os.path.exists(cur_file):
    stamp = now_utc(); ref = {"anchor_id": stamp, "created_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(), "host_note": "paired with the calibration live at this moment", "cams": {}}
    os.makedirs(f"{DIR}/ref_{stamp}", exist_ok=True)
    # average odom over 3 s of messages (camera static) to get a stable reference
    samples = {c: [] for c in CAMS}; t0 = time.time()
    while time.time() - t0 < 3.0:
        rclpy.spin_once(node, timeout_sec=0.02)
        for c in CAMS:
            if S[c]["odom"] is not None and (not samples[c] or S[c]["t_rx"] != samples[c][-1][2]): samples[c].append((S[c]["odom"][0].copy(), S[c]["odom"][1].copy(), S[c]["t_rx"]))
    for c in CAMS:
        e = dict(ok=False)
        if S[c]["odom"] is None or not samples[c]: e["error"] = "no odom"; ref["cams"][c] = e; continue
        P = np.array([s[0] for s in samples[c]]); Q = np.array([s[1] for s in samples[c]]); q = Q[-1]; Q = Q * np.sign(Q @ q)[:, None]; qm = Q.mean(0); qm /= np.linalg.norm(qm)
        o = S[c]["odom"]; e.update(ok=True, odom_frame=o[3], odom_child=o[2], n_samples=len(P), odom_pos=P.mean(0).tolist(), odom_quat_xyzw=qm.tolist(), odom_pos_std_mm=(P.std(0) * 1000).round(2).tolist(),
                                   odom_pose_matrix=None)
        Om = np.eye(4); Om[:3, :3] = q2R(qm); Om[:3, 3] = P.mean(0); e["odom_pose_matrix"] = Om.tolist()
        try:
            e["T_base_link"] = tf4("fr3_link0", f"{c}_camera_link").tolist(); e["T_base_optical"] = tf4("fr3_link0", f"{c}_left_camera_optical_frame").tolist()
            e["T_link_optical"] = tf4(f"{c}_camera_link", f"{c}_left_camera_optical_frame").tolist()
        except Exception as ex: e["tf_error"] = str(ex)[:200]
        cf = CALIB[c]; e["calibration_file"] = cf; e["calibration_md5"] = hashlib.md5(open(cf, "rb").read()).hexdigest() if os.path.exists(cf) else None
        A = S[c]["acc"]
        if A: e["imu_accel_mean_m_s2"] = np.mean([a[:3] for a in A], 0).round(4).tolist(); e["imu_frame"] = A[-1][3]; e["imu_samples"] = len(A)
        d = f"{DIR}/ref_{stamp}"; m, md, mi = S[c]["img"], S[c]["dep"], S[c]["info"]
        if m is not None: cv2.imwrite(f"{d}/{c}_left.png", np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, -1)[:, :, :3]); e["ref_image"] = f"ref_{stamp}/{c}_left.png"
        if md is not None: np.savez_compressed(f"{d}/{c}_depth.npz", depth=np.frombuffer(bytes(md.data), np.float32).reshape(md.height, md.width).copy()); e["ref_depth"] = f"ref_{stamp}/{c}_depth.npz"
        if mi is not None: json.dump(dict(K=list(mi.k), D=list(mi.d), w=mi.width, h=mi.height), open(f"{d}/{c}_info.json", "w")); e["ref_info"] = f"ref_{stamp}/{c}_info.json"
        ref["cams"][c] = e
    fn = f"anchor_{stamp}.json"; atomic(f"{DIR}/{fn}", ref); open(cur_file, "w").write(fn + "\n"); os.chmod(f"{DIR}/{fn}", 0o444)
    ev(f"ANCHOR WRITTEN {fn}: " + "; ".join(f"{c} ok={ref['cams'][c].get('ok')} odom_pos={np.round(ref['cams'][c].get('odom_pos', [0,0,0]),3).tolist()} std_mm={ref['cams'][c].get('odom_pos_std_mm')}" for c in CAMS))
else: ev(f"RESUME logging against {open(cur_file).read().strip()} (anchor not touched)")
anchor_name = open(cur_file).read().strip()

def tick_log():
    rec = dict(t=time.time(), anchor=anchor_name, cams={}); lg = f"{DIR}/odom_log.jsonl"
    for c in CAMS:
        s = S[c]
        if s["odom"] is None: rec["cams"][c] = None; continue
        age = time.time() - s["t_rx"]
        if age > GAP_S and not s["gap"]: s["gap"] = True; s["epoch"] += 1; ev(f"{c} GAP: no odom for {age:.1f} s -> epoch {s['epoch']} (continuity LOST until verified)")
        rec["cams"][c] = dict(p=s["odom"][0].round(4).tolist(), q=s["odom"][1].round(5).tolist(), age_s=round(age, 2), epoch=s["epoch"])
    if not os.path.exists(lg) or os.path.getsize(lg) < LOG_MAX: open(lg, "a").write(json.dumps(rec) + "\n")
def tick_last():
    rec = dict(t=time.time(), utc=datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="seconds"), anchor=anchor_name, cams={})
    for c in CAMS:
        s = S[c]
        if s["odom"] is not None: rec["cams"][c] = dict(p=s["odom"][0].tolist(), q_xyzw=s["odom"][1].tolist(), age_s=round(time.time() - s["t_rx"], 2), epoch=s["epoch"], gap=s["gap"])
    atomic(f"{DIR}/last_good.json", rec)
node.create_timer(2.0, tick_log); node.create_timer(5.0, tick_last)
print("logging; Ctrl-C/SIGTERM to stop", flush=True); rclpy.spin(node)
