"""3D (fr3_link0) of a pixel in each ZED from its own depth (median of a 5x5 window), to compare the two cameras' calibration on the same object.
  python3 point_check.py zed1:u,v zed2:u,v ..."""
import sys, time, numpy as np, rclpy
from rclpy.node import Node; from rclpy.time import Time; from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo; from tf2_ros import Buffer, TransformListener
rclpy.init(); n = Node("point_check"); b = Buffer(); TransformListener(b, n); dep, inf = {}, {}
req = [(s.split(":")[0], [int(x) for x in s.split(":")[1].split(",")]) for s in sys.argv[1:]]
for c in {r[0] for r in req}:
    n.create_subscription(Image, f"/{c}/zed_node/depth/depth_registered", lambda m, c=c: dep.__setitem__(c, m), qos_profile_sensor_data)
    n.create_subscription(CameraInfo, f"/{c}/zed_node/depth/camera_info", lambda m, c=c: inf.__setitem__(c, m), qos_profile_sensor_data)
t0 = time.monotonic()
while time.monotonic() - t0 < 4: rclpy.spin_once(n, timeout_sec=0.05)
for c, (u, v) in req:
    m, K = dep[c], np.array(inf[c].k).reshape(3, 3); d = np.frombuffer(m.data, np.float32).reshape(m.height, m.width)
    z = float(np.nanmedian(d[v - 2:v + 3, u - 2:u + 3])); p = np.array([(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z])
    t = b.lookup_transform("fr3_link0", m.header.frame_id, Time()).transform; q = t.rotation; x, y, w_, zq = q.x, q.y, q.w, q.z
    R = np.array([[1-2*(y*y+zq*zq), 2*(x*y-zq*w_), 2*(x*zq+y*w_)], [2*(x*y+zq*w_), 1-2*(x*x+zq*zq), 2*(y*zq-x*w_)], [2*(x*zq-y*w_), 2*(y*zq+x*w_), 1-2*(x*x+y*y)]])
    pb = R @ p + [t.translation.x, t.translation.y, t.translation.z]
    print(f"{c} ({u},{v}): depth {z:.3f} m -> fr3_link0 {np.round(pb, 3).tolist()}")
