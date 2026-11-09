import numpy as np
Z = np.load("/s/esdf_live.npz"); E, O, V = Z["esdf"], Z["origin"], float(Z["voxel"]); R = np.load("/s/objects_raw.npz")
def col(x, y):
    i, j = int((x - O[0]) // V), int((y - O[1]) // V); return E[i, j, :]
def zc(k): return O[2] + (k + 0.5) * V
def crossing(e):
    # highest z where the ESDF changes sign from <=0 (below) to >0 (above), interpolated; unobserved (-1000) ignored
    best = None
    for k in range(len(e) - 1):
        a, b = e[k], e[k + 1]
        if a > -999 and b > -999 and a <= 0 < b: best = zc(k) + (0 - a) / (b - a) * V
    return best
def prof(name, x, y, raw_top, raw_surf):
    e = col(x, y); ks = [k for k in range(len(e)) if -0.2 < zc(k) < 0.32]
    print(f"{name} at ({x:.2f},{y:.2f}): raw top z {raw_top:.3f} (surface points z up to), map zero-crossing z {crossing(e)}")
    print("   z(cm):  " + " ".join(f"{100*zc(k):5.1f}" for k in ks)); print("   ESDF cm:" + " ".join(f"{100*e[k]:5.1f}" if e[k] > -999 else "  unk" for k in ks))
# table columns away from objects: where does the map put the table top vs the raw cloud (42 mm)?
tc = [crossing(col(x, y)) for x in (0.25, 0.30, 0.45, 0.60) for y in (-0.10, 0.0, 0.10, 0.25)]; tc = [t for t in tc if t is not None]
print(f"TABLE top in the map (zero crossing over {len(tc)} empty columns): median {1000*np.median(tc):.0f} mm, range {1000*min(tc):.0f}..{1000*max(tc):.0f} mm   (raw cloud table plane: 42 mm)")
for nm, key in (("bottle", "o1"), ("drill", "o2"), ("box", "o3"), ("panel", "o4")):
    p = R[key]; c = np.median(p[:, :2], 0); top = np.percentile(p[:, 2], 99); prof(nm, c[0], c[1], top, None)
