import base64, io, json, struct, os, numpy as np, cv2
OUT = "/tmp/claude_out"; os.makedirs(OUT, exist_ok=True)
H, W = 360, 640; vv, uu = np.mgrid[0:H, 0:W]
K = np.array(json.load(open("/work/data/live_all/live_calib_1023/000/zed1_info.json"))["K"]).reshape(3, 3)
T = np.array(json.load(open("/work/calib_1031/zed1_front_camera_extrinsic.json"))["T_base_cam"])
def frame(p):
    A = np.load(io.BytesIO(base64.b64decode(open(p).read().strip())))
    return np.ascontiguousarray(A["zed1_img"]), A["zed1_dep"].astype(np.float32)
def mesh(p):
    d = base64.b64decode(open(p).read().strip()); ver, nv, nt = struct.unpack_from("<3I", d, 0)
    V = np.frombuffer(d, "<f4", nv * 3, 12).reshape(-1, 3).astype(np.float64); Tr = np.frombuffer(d, "<u4", nt * 3, 12 + nv * 12 + nv * 4).reshape(-1, 3)
    return V, Tr
imA, dA = frame("/tmp/claude_fA_b64.txt"); imB, dB = frame("/tmp/claude_fB_b64.txt")
VA, TA = mesh("/tmp/claude_mesh_z1_720.b64"); VB, TB = mesh("/tmp/claude_mesh_z1_1080.b64")
def cloud(d):
    ok = np.isfinite(d) & (d > 0.3) & (d < 3.5); X = np.stack([(uu - K[0, 2]) * d / K[0, 0], (vv - K[1, 2]) * d / K[1, 1], d], -1).reshape(-1, 3)[ok.reshape(-1)]
    return X @ T[:3, :3].T + T[:3, 3]
PA = cloud(dA)
def plane(z):
    h, e = np.histogram(z, bins=np.arange(-0.12, 0.1001, 0.004)); return 0.5 * (e[np.argmax(h)] + e[np.argmax(h) + 1]) * 1000
def win(P): return P[(P[:, 0] > 0.35) & (P[:, 0] < 0.85) & (abs(P[:, 1]) < 0.40) & (P[:, 2] > -0.12) & (P[:, 2] < 0.10)]
mA, mB = plane(win(VA)[:, 2]), plane(win(VB)[:, 2]); print(f"mesh table plane: 720p map {mA:.0f} mm, 1080p map {mB:.0f} mm; vertices {len(VA)} vs {len(VB)}")
def turbo(d, lo=0.3, hi=2.0):
    n = np.clip((np.nan_to_num(d, nan=0) - lo) / (hi - lo), 0, 1); c = cv2.applyColorMap((n * 255).astype(np.uint8), cv2.COLORMAP_TURBO); c[~(np.isfinite(d) & (d > 0.3))] = 30; return c
def label(img, s, y=22): cv2.rectangle(img, (0, 0), (img.shape[1], y + 6), (0, 0, 0), -1); cv2.putText(img, s, (6, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA); return img

def label(img, s, y=22): cv2.rectangle(img, (0, 0), (img.shape[1], y + 6), (0, 0, 0), -1); cv2.putText(img, s, (6, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA); return img
f1 = np.vstack([np.hstack([label(imA[:, :, :3].copy(), "zed1 RGB, 720p grab (published 640x360)"), label(imB[:, :, :3].copy(), "zed1 RGB, 1080p grab (published 640x360): narrower view")]),
                np.hstack([label(turbo(dA), "zed1 depth, 720p (0.3-2.0 m)"), label(turbo(dB), "zed1 depth, 1080p (same colour scale)")])])
cv2.imwrite(f"{OUT}/fig1_zed1_rgb_depth_720_vs_1080.jpg", f1, [cv2.IMWRITE_JPEG_QUALITY, 90]); print("fig1", f1.shape)
def render(V, Tr, yaw, pitch, size=(560, 440), scale=520, c0=(0.55, 0.0, 0.08), zr=(-0.08, 0.30)):
    cy, sy, cp, sp = np.cos(yaw), np.sin(yaw), np.cos(pitch), np.sin(pitch)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]); Rx = np.array([[1, 0, 0], [0, cp, -sp], [0, sp, cp]]); R = Rx @ Rz
    keep = (V[:, 0] > 0.2) & (V[:, 0] < 1.1) & (abs(V[:, 1]) < 0.7) & (V[:, 2] > -0.1) & (V[:, 2] < 0.6)
    tri = Tr[keep[Tr].all(1)]; img = np.full((size[1], size[0], 3), 245, np.uint8)
    P = (V - np.array(c0)) @ R.T; Pp = np.stack([P[:, 0] * scale + size[0] / 2, size[1] / 2 - P[:, 1] * scale * (1 if pitch == 0 else 1)], 1)
    # tilted: use depth = P[:,2] after rotation toward the viewer; screen y uses P[:,1]
    n = np.cross(V[tri[:, 1]] - V[tri[:, 0]], V[tri[:, 2]] - V[tri[:, 0]]); n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-9)
    n[n[:, 2] < 0] *= -1; shade = 0.45 + 0.55 * np.clip(n @ (np.array([0.3, -0.4, 0.85]) / np.linalg.norm([0.3, -0.4, 0.85])), 0, 1)
    zc = V[tri].mean(1)[:, 2]; cc = cv2.applyColorMap((np.clip((zc - zr[0]) / (zr[1] - zr[0]), 0, 1) * 255).astype(np.uint8)[:, None], cv2.COLORMAP_TURBO)[:, 0, :].astype(np.float32) * shade[:, None]
    order = np.argsort(P[tri].mean(1)[:, 2])
    for k in order: cv2.fillConvexPoly(img, Pp[tri[k]].astype(np.int32), cc[k].tolist(), cv2.LINE_8)
    return img

views = {}
for nm, V, Tr in (("A", VA, TA), ("B", VB, TB)):
    views[nm] = (render(V, Tr, np.radians(-30), np.radians(50)), render(V, Tr, 0, 0, scale=520, c0=(0.62, 0.0, 0.0)))
def side(Vs, cols, title, bound=(0.30, 0.90), zlim=(-0.12, 0.12)):
    img = np.full((440, 560, 3), 255, np.uint8); s = 420 / (zlim[1] - zlim[0])
    for z in np.arange(-0.12, 0.121, 0.02): y = int(440 - (z - zlim[0]) * s - 10); cv2.line(img, (0, y), (560, y), (225, 225, 225), 1); cv2.putText(img, f"{z*1000:+.0f}", (2, y - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (120, 120, 120), 1)
    for V, c in zip(Vs, cols):
        m = (V[:, 0] > bound[0]) & (V[:, 0] < bound[1]) & (abs(V[:, 1]) < 0.4) & (V[:, 2] > zlim[0]) & (V[:, 2] < zlim[1]); S = V[m][::max(1, m.sum() // 6000)]
        for p in S: cv2.circle(img, (int((p[0] - bound[0]) / (bound[1] - bound[0]) * 540 + 10), int(440 - (p[2] - zlim[0]) * s - 10)), 1, c, -1)
    cv2.putText(img, title, (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1, cv2.LINE_AA); cv2.putText(img, "x (m) right, z in mm up", (330, 432), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1); return img
sm = side([VA, VB], [(220, 100, 0), (0, 0, 220)], "mesh side view: blue 720p, red 1080p")
txt = np.full((440, 560, 3), 255, np.uint8)
for i, l in enumerate(["Both maps: nvblox, zed1 ONLY, Spot params (2 cm voxel,", "6 voxel truncation, max weight 100, min weight 0.0001),", "same scene, settled ~110 s.", "", f"table plane height in the mesh: {mA:.0f} mm (720p) vs {mB:.0f} mm (1080p)", f"vertices: {len(VA)} (720p) vs {len(VB)} (1080p)", "", "Same table height -> no depth bias at 1080p.", "1080p grab = narrower field of view (zoom ~1.3x),", "so it sees less of the table edges / robot base."]): cv2.putText(txt, l, (10, 40 + 30 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
g = lambda im, s: label(im.copy(), s)
f2 = np.vstack([np.hstack([g(views["A"][0], f"nvblox map, zed1 only, 720p ({len(VA)} verts)"), g(views["B"][0], f"nvblox map, zed1 only, 1080p ({len(VB)} verts)"), sm]),
                np.hstack([g(views["A"][1], "top view 720p (colour = height, -8..30 cm)"), g(views["B"][1], "top view 1080p (same scale)"), txt])])
cv2.imwrite(f"{OUT}/fig2_nvblox_map_zed1_only_720_vs_1080.jpg", f2, [cv2.IMWRITE_JPEG_QUALITY, 90]); print("fig2", f2.shape)
