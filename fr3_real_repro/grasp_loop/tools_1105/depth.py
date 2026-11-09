import json, re, numpy as np
src = open('/tmp/grasp_plan.py').read(); pre = src[:src.index("g = json.load(")].replace("a = ap.parse_args()", "a = ap.parse_args(['x'])"); exec(pre)
q = np.array([float(x) for x in re.findall(r'-?\d+\.\d+(?:e-?\d+)?', open('/tmp/now.txt').read())[:7]])
C = np.array(json.load(open('/tmp/gin.json'))['cloud'], float)
H = fk(q); A, Y, X = H[:3, 2], H[:3, 1], H[:3, 0]; tcp0 = H[:3, 3] + A * 0.1034
print(' d(cm) along approach | tipA z  tipB z | top-tipHigh(cm) | pts between pads | finger-body hits | palm hits')
for d in np.arange(0, 0.071, 0.005):
    tcp = tcp0 + A * d; r = C - tcp; ry, ra, rx = r @ Y, r @ A, r @ X
    between = (np.abs(ry) < 0.035) & (np.abs(rx) < 0.011) & (ra > -0.045) & (ra < 0.0)
    fingers = (np.abs(np.abs(ry) - 0.045) < 0.01) & (np.abs(rx) < 0.012) & (ra > -0.05) & (ra < 0.005)
    palm = (ra < -0.05) & (ra > -0.11) & (np.abs(ry) < 0.105) & (np.abs(rx) < 0.035)
    ta, tb = tcp + Y * 0.04, tcp - Y * 0.04
    print(f'  {d*100:4.1f}               | {ta[2]:.3f}  {tb[2]:.3f} | {(0.2847-max(ta[2],tb[2]))*100:5.1f}          | {between.sum():5d}            | {fingers.sum():4d}             | {palm.sum():4d}')
