import json, time, numpy as np, rclpy
from sensor_msgs.msg import Image, CameraInfo
rclpy.init(); n = rclpy.create_node('grasp_lane_table_probe')
got = {}
n.create_subscription(Image, '/vm/cable/depth', lambda m: got.setdefault('d', m), 1)
n.create_subscription(CameraInfo, '/zed1/zed_node/depth/camera_info', lambda m: got.setdefault('i', m), 1)
t0 = time.time()
while (len(got) < 2) and time.time() - t0 < 20: rclpy.spin_once(n, timeout_sec=0.2)
n.destroy_node(); rclpy.shutdown()
if len(got) < 2: raise SystemExit(f'missing {set(["d","i"]) - set(got)}')
m, info = got['d'], got['i']
D = np.frombuffer(m.data, np.float32).reshape(m.height, m.width)
K = np.array(info.k).reshape(3, 3); s = m.width / info.width
fx, fy, cx, cy = K[0, 0] * s, K[1, 1] * s, K[0, 2] * s, K[1, 2] * s
v, u = np.nonzero(np.isfinite(D) & (D > 0.2) & (D < 3.0)); z = D[v, u].astype(float)
Pc = np.stack([(u - cx) * z / fx, (v - cy) * z / fy, z], 1)
T = np.array(json.loads(json.load(open('/tmp/z1.json'))['T_base_cam']) if isinstance(json.load(open('/tmp/z1.json'))['T_base_cam'], str) else json.load(open('/tmp/z1.json'))['T_base_cam'], float)
P = Pc @ T[:3, :3].T + T[:3, 3]
np.save('/tmp/zed1_table_pts.npy', P.astype(np.float32))
print(f'frame {m.width}x{m.height} enc {m.encoding}, {len(P)} pts')
T_ = P[(P[:, 2] > -0.08) & (P[:, 2] < 0.09) & (P[:, 0] > 0.05) & (P[:, 0] < 1.0) & (P[:, 1] > -0.75) & (P[:, 1] < 0.5)]
xs = np.arange(0.10, 0.96, 0.05); ys = np.arange(-0.70, 0.46, 0.05)
print('ZED1 table height (cm, p30 per 5 cm cell, fr3_link0); rows y, cols x; robot base at (0,0); drill ~(0.47,-0.32)')
print('  y\\x  ' + ' '.join(f'{x:5.2f}' for x in xs))
for y in ys[::-1]:
    row = []
    for x in xs:
        c = T_[(np.abs(T_[:, 0] - x) < 0.025) & (np.abs(T_[:, 1] - y) < 0.025)]
        row.append(f'{np.percentile(c[:, 2], 30) * 100:5.1f}' if len(c) >= 30 else '   . ')
    print(f'{y:+6.2f} ' + ' '.join(row))
h, e = np.histogram(T_[:, 2], bins=np.arange(-0.06, 0.08, 0.005))
print('z histogram (cm: count):', ' '.join(f'{(e[i]+0.0025)*100:+.1f}:{h[i]}' for i in range(len(h)) if h[i] > len(T_) * 0.01))
