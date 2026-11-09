import base64, struct, numpy as np
def load(p):
    d = base64.b64decode(open(p).read().strip()); ver, nv, nt = struct.unpack_from("<3I", d, 0)
    V = np.frombuffer(d, "<f4", nv * 3, 12).reshape(-1, 3).astype(np.float64); C = np.frombuffer(d, "u1", nv * 4, 12 + nv * 12).reshape(-1, 4)[:, :3].astype(np.float64)
    T = np.frombuffer(d, "<u4", nt * 3, 12 + nv * 16).reshape(-1, 3).astype(np.int64); return V, C, T
for nm, p in (("720p", "/tmp/claude_mesh_z1_720.b64"), ("1080p", "/tmp/claude_mesh_z1_1080.b64")):
    V, C, T = load(p); P = V[T]; cen = P.mean(1); area = 0.5 * np.linalg.norm(np.cross(P[:, 1] - P[:, 0], P[:, 2] - P[:, 0]), axis=1); col = C[T].mean(1)
    box = (cen[:, 0] > 0.40) & (cen[:, 0] < 0.65) & (cen[:, 1] > -0.45) & (cen[:, 1] < -0.15) & (cen[:, 2] > 0.05)
    yellow = box & (col[:, 0] > 120) & (col[:, 1] > 80) & (col[:, 2] < 90) & (col[:, 0] - col[:, 2] > 60)
    zs = cen[box, 2] if box.any() else np.array([0])
    print(f"{nm}: triangles above the table in the drill box {box.sum():4d}, surface area {1e4*area[box].sum():6.0f} cm2, yellow (drill body) area {1e4*area[yellow].sum():5.0f} cm2 ({yellow.sum()} tris), height range of those triangles {100*zs.min():.0f}..{100*zs.max():.0f} cm")
