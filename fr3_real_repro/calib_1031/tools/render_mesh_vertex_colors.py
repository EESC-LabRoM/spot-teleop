import base64, struct, json, os, numpy as np, cv2
OUT = "/tmp/claude_out3"; os.makedirs(OUT, exist_ok=True)
T1 = np.array(json.load(open("/work/calib_1031/zed1_front_camera_extrinsic.json"))["T_base_cam"]); ZED1 = T1[:3, 3]
def load(p):
    d = base64.b64decode(open(p).read().strip()); ver, nv, nt = struct.unpack_from("<3I", d, 0)
    V = np.frombuffer(d, "<f4", nv * 3, 12).reshape(-1, 3).astype(np.float64); C = np.frombuffer(d, "u1", nv * 4, 12 + nv * 12).reshape(-1, 4)[:, :3].astype(np.float64)
    T = np.frombuffer(d, "<u4", nt * 3, 12 + nv * 16).reshape(-1, 3).astype(np.int64); return V, C, T
MESH = {"720p": load("/tmp/claude_mesh_z1_720.b64"), "1080p": load("/tmp/claude_mesh_z1_1080.b64")}
def vnormals(V, T):
    n = np.cross(V[T[:, 1]] - V[T[:, 0]], V[T[:, 2]] - V[T[:, 0]]); N = np.zeros_like(V)
    for k in range(3): np.add.at(N, T[:, k], n)
    return N / np.maximum(np.linalg.norm(N, axis=1, keepdims=True), 1e-12)
def render(name, eye, target, up, fov, W=640, H=480, crop=None):
    V, C, T = MESH[name]
    if crop is not None:
        lo, hi = np.array(crop[0]), np.array(crop[1]); keepv = ((V >= lo) & (V <= hi)).all(1); T = T[keepv[T].all(1)]
    N = vnormals(V, T) if False else vnormals(V, MESH[name][2])
    eye, target, up = map(np.array, (eye, target, up)); f = target - eye; f /= np.linalg.norm(f); r = np.cross(f, up); r /= np.linalg.norm(r); u = np.cross(r, f)
    Pc = (V - eye) @ np.stack([r, u, f]).T; fx = (H / 2) / np.tan(np.radians(fov) / 2)
    z = np.maximum(Pc[:, 2], 1e-6); px = W / 2 + fx * Pc[:, 0] / z; py = H / 2 - fx * Pc[:, 1] / z
    view = (eye - V); view /= np.maximum(np.linalg.norm(view, axis=1, keepdims=True), 1e-9)
    lam = np.abs((N * view).sum(1)); col = C * (0.35 + 0.65 * lam)[:, None]
    img = np.full((H, W, 3), 38.0); zb = np.full((H, W), 1e9)
    ok = (Pc[T, 2] > 0.05).all(1); T = T[ok]
    for a, b, c in T:
        xs = (px[a], px[b], px[c]); ys = (py[a], py[b], py[c])
        x0, x1 = int(max(0, np.floor(min(xs)))), int(min(W - 1, np.ceil(max(xs)))); y0, y1 = int(max(0, np.floor(min(ys)))), int(min(H - 1, np.ceil(max(ys))))
        if x1 < x0 or y1 < y0: continue
        X, Y = np.meshgrid(np.arange(x0, x1 + 1), np.arange(y0, y1 + 1))
        den = (ys[1] - ys[2]) * (xs[0] - xs[2]) + (xs[2] - xs[1]) * (ys[0] - ys[2])
        if abs(den) < 1e-9: continue
        w0 = ((ys[1] - ys[2]) * (X - xs[2]) + (xs[2] - xs[1]) * (Y - ys[2])) / den; w1 = ((ys[2] - ys[0]) * (X - xs[2]) + (xs[0] - xs[2]) * (Y - ys[2])) / den; w2 = 1 - w0 - w1
        m = (w0 >= -1e-4) & (w1 >= -1e-4) & (w2 >= -1e-4)
        if not m.any(): continue
        zz = w0 * z[a] + w1 * z[b] + w2 * z[c]; cur = zb[y0:y1 + 1, x0:x1 + 1]; s = m & (zz < cur)
        if not s.any(): continue
        cur[s] = zz[s]; sub = img[y0:y1 + 1, x0:x1 + 1]
        sub[s] = (w0[..., None] * col[a] + w1[..., None] * col[b] + w2[..., None] * col[c])[s]
    return np.clip(img, 0, 255).astype(np.uint8)[:, :, ::-1].copy()      # RGB -> BGR for cv2
def lab(img, s): cv2.rectangle(img, (0, 0), (img.shape[1], 26), (0, 0, 0), -1); cv2.putText(img, s, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA); return img
D = np.array([0.52, -0.30, 0.10])       # drill (top ~ 20 cm above the table plane)
toward = ZED1 - D; toward /= np.linalg.norm(toward)
VIEWS = [("zed1 viewpoint, whole table", ZED1 + np.array([0, 0, 0.02]), np.array([0.55, -0.05, 0.03]), (0, 0, 1), 75, None),
         ("drill from the zed1 side", D + 0.55 * toward, D, (0, 0, 1), 32, ((0.2, -0.7, -0.1), (0.9, 0.2, 0.5))),
         ("drill from above", D + np.array([0, 0, 0.65]), D, (1, 0, 0), 32, ((0.2, -0.7, -0.1), (0.9, 0.2, 0.5))),
         ("drill from the far side (+x)", D + np.array([0.6, 0.0, 0.08]), D, (0, 0, 1), 32, ((0.2, -0.7, -0.1), (0.9, 0.2, 0.5)))]
rows = []
for nm, eye, tgt, up, fov, crop in VIEWS:
    ims = [lab(render(k, eye, tgt, up, fov, crop=crop), f"{k} map: {nm}") for k in ("720p", "1080p")]; rows.append(np.hstack(ims)); print("rendered", nm, flush=True)
cv2.imwrite(f"{OUT}/mesh_whole_table.jpg", rows[0], [cv2.IMWRITE_JPEG_QUALITY, 92]); cv2.imwrite(f"{OUT}/mesh_drill.jpg", np.vstack(rows[1:]), [cv2.IMWRITE_JPEG_QUALITY, 92]); print("done")
