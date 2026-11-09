import base64, io, json, numpy as np
H, W = 360, 640; vv, uu = np.mgrid[0:H, 0:W]
K = {c: np.array(json.load(open(f"/work/data/live_all/live_calib_1023/000/{c}_info.json"))["K"]).reshape(3, 3) for c in ("zed1", "zed2")}
T = {"zed1": np.array(json.load(open("/work/calib_1031/zed1_front_camera_extrinsic.json"))["T_base_cam"]), "zed2": np.array(json.load(open("/work/calib_1031/zed2_top_camera_extrinsic.json"))["T_base_cam"])}
F = {"frame2 (720p depth, 21:13)": "/tmp/claude_frame2_b64.txt", "frame3 (1080p depth, 21:37)": "/tmp/claude_frame3_b64.txt", "frame4 (zed1 back to 720p, now)": "/tmp/claude_frame4_b64.txt"}
for nm, p in F.items():
    A = np.load(io.BytesIO(base64.b64decode(open(p).read().strip()))); print("==", nm)
    for c in ("zed1", "zed2"):
        d = A[f"{c}_dep"].astype(np.float32); ok = np.isfinite(d) & (d > 0.3) & (d < 3.5); k = K[c]
        X = np.stack([(uu - k[0, 2]) * d / k[0, 0], (vv - k[1, 2]) * d / k[1, 1], d], -1).reshape(-1, 3)[ok.reshape(-1)]
        P = X @ T[c][:3, :3].T + T[c][:3, 3]
        s = (P[:, 0] > 0.35) & (P[:, 0] < 0.85) & (abs(P[:, 1]) < 0.40) & (P[:, 2] > -0.12) & (P[:, 2] < 0.10)
        z = P[s, 2]
        if len(z) < 500: print(f"   {c}: only {len(z)} pts in the table window"); continue
        h, e = np.histogram(z, bins=np.arange(-0.12, 0.1001, 0.004)); z0 = 0.5 * (e[np.argmax(h)] + e[np.argmax(h) + 1]); t = z[abs(z - z0) < 0.02]
        print(f"   {c}: table plane peak z = {z0*1000:5.0f} mm, {len(t):6d} plane pts, robust sigma {1.4826*np.median(abs(t-np.median(t)))*1000:4.1f} mm, depth median {np.median(d[ok]):.3f} m")
