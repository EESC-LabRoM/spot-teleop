import urllib.request, numpy as np
b = urllib.request.urlopen('http://127.0.0.1:18089/mesh.bin', timeout=10).read()
h = np.frombuffer(b, np.uint32, 3); V = np.frombuffer(b, np.float32, int(h[1]) * 3, 12).reshape(-1, 3).astype(float)
V = V[(V[:, 2] > -0.15) & (V[:, 2] < 0.12)]          # near-table surfaces only
xs = np.arange(0.20, 0.86, 0.05); ys = np.arange(-0.60, 0.41, 0.05)
print('map surface z (cm, median of lowest-ish points) per 5 cm cell; rows = y, cols = x; drill ~ x0.47 y-0.32')
print('   y\\x ' + ' '.join(f'{x:5.2f}' for x in xs))
for y in ys[::-1]:
    row = []
    for x in xs:
        m = (np.abs(V[:, 0] - x) < 0.025) & (np.abs(V[:, 1] - y) < 0.025)
        row.append(f'{np.percentile(V[m, 2], 30) * 100:5.1f}' if m.sum() >= 5 else '   . ')
    print(f'{y:+6.2f} ' + ' '.join(row))
