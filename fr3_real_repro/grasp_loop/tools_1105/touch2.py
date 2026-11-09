import re, numpy as np
src = open('/tmp/grasp_plan.py').read(); pre = src[:src.index("g = json.load(")].replace("a = ap.parse_args()", "a = ap.parse_args(['x'])"); exec(pre)
v = [float(x) for x in re.findall(r'-?\d+\.\d+(?:e-?\d+)?', open('/tmp/now.txt').read())]; q = np.array(v[:7])
H, P = fk(q, pts=True); A, Y = H[:3, 2], H[:3, 1]; tcp = H[:3, 3] + A * 0.1034
tips = [tcp + Y * 0.04, tcp - Y * 0.04]
print('width mm %.0f  tilt from straight-down %.1f deg' % ((v[7] + v[8]) * 1000, np.degrees(np.arccos(max(-1, min(1, -H[2, 2]))))))
print('TCP', np.round(tcp, 4), ' tipA', np.round(tips[0], 4), ' tipB', np.round(tips[1], 4), ' hand origin z %.4f' % H[2, 3])
low = min(tips, key=lambda t: t[2])
for name in ('zed1', 'zed2'):
    Pc = np.load(f'/tmp/{name}_table_pts.npy').astype(float)
    for r in (0.03, 0.05):
        m = (np.linalg.norm(Pc[:, :2] - low[:2], axis=1) < r) & (Pc[:, 2] < low[2] + 0.05) & (Pc[:, 2] > low[2] - 0.10)
        if m.sum() > 20:
            z = Pc[m, 2]; print(f'{name}: surface within {r*100:.0f} cm of the lower tip xy: n={m.sum()} z p20/p50 = {np.percentile(z,20)*100:+.1f} {np.percentile(z,50)*100:+.1f} cm  -> lower tip {low[2]*100:+.1f} cm')
            break
    else:
        print(name, 'no surface near the tip xy')
