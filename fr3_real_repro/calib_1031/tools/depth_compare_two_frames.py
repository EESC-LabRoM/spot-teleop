import base64, io, numpy as np
A = np.load(io.BytesIO(base64.b64decode(open("/tmp/claude_frame2_b64.txt").read().strip())))   # 720p-derived depth (21:13)
B = np.load(io.BytesIO(base64.b64decode(open("/tmp/claude_frame_b64.txt").read().strip())))    # 1080p-derived depth (~21:37)
for c in ("zed1", "zed2"):
    da, db = A[f"{c}_dep"].astype(np.float32), B[f"{c}_dep"].astype(np.float32)
    ia, ib = A[f"{c}_img"][:, :, :3].astype(np.float32).mean(-1), B[f"{c}_img"][:, :, :3].astype(np.float32).mean(-1)
    ok = np.isfinite(da) & np.isfinite(db) & (da > 0.3) & (db > 0.3)
    # static pixels: image almost unchanged (lighting/auto-exposure aside) and not at depth edges
    static = ok & (np.abs(ia - ib) < 12)
    print(f"{c}: overall median depth 720p-frame {np.nanmedian(da[da>0.3]):.3f} m, 1080p-frame {np.nanmedian(db[db>0.3]):.3f} m; static pixels {static.sum()} ({100*static.mean():.0f}%)")
    r = db[static] / da[static]
    for lo, hi in ((0.3, 0.7), (0.7, 1.0), (1.0, 1.4), (1.4, 2.5)):
        s = static & (da >= lo) & (da < hi)
        if s.sum() > 200: print(f"   depth {lo:.1f}-{hi:.1f} m (old): n={s.sum():6d}  new/old median ratio {np.median(db[s]/da[s]):.3f}  median diff {1000*np.median(db[s]-da[s]):+.0f} mm")
