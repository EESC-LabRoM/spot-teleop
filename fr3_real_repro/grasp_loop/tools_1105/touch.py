"""Touch test: fingertip height (FK) vs map surface height at the same xy. Runs in vm-viewer.
python3 /tmp/touch.py   (reads /tmp/now.txt = joint_states echo)"""
import json, re, urllib.request, numpy as np
src = open('/tmp/grasp_plan.py').read(); pre = src[:src.index("g = json.load(")].replace("a = ap.parse_args()", "a = ap.parse_args(['x'])"); exec(pre)
v = [float(x) for x in re.findall(r'-?\d+\.\d+(?:e-?\d+)?', open('/tmp/now.txt').read())]
q = np.array(v[:7]); width = (v[7] + v[8]) * 1000 if len(v) >= 9 else float('nan')
H, P = fk(q, pts=True)
tcp = H[:3, 3] + H[:3, 2] * 0.1034
print(f'gripper width {width:.0f} mm   tilt from straight-down {np.degrees(np.arccos(max(-1, min(1, -H[2, 2])))):.1f} deg')
print('fingertip centre (FK, fr3_link0):', np.round(tcp, 4))
b = urllib.request.urlopen('http://127.0.0.1:18089/mesh.bin', timeout=10).read()
h = np.frombuffer(b, np.uint32, 3); V = np.frombuffer(b, np.float32, int(h[1]) * 3, 12).reshape(-1, 3).astype(float)
for r in (0.03, 0.05, 0.08):
    m = (np.linalg.norm(V[:, :2] - tcp[:2], axis=1) < r) & (V[:, 2] < tcp[2] + 0.03) & (V[:, 2] > tcp[2] - 0.15)
    if m.sum():
        z = V[m, 2]; print(f'map surface within {r*100:.0f} cm xy (below tip+3cm): n={m.sum():4d}  z p10/p50/p90 = {np.percentile(z,10):+.4f} {np.percentile(z,50):+.4f} {np.percentile(z,90):+.4f}  -> tip - map p50 = {(tcp[2]-np.percentile(z,50))*100:+.1f} cm')
    else:
        print(f'map surface within {r*100:.0f} cm xy: none')
