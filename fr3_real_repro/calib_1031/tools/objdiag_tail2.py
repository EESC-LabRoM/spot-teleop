import io, base64, re, json, sys, math
import numpy as np, cv2, torch
from scipy import ndimage
from scipy.spatial import cKDTree
now = np.load(io.BytesIO(base64.b64decode(B64)))
def parse_ply(b):
    t = base64.b64decode(b).decode(errors="ignore").splitlines(); n = int(re.search(r"element vertex (\d+)", "\n".join(t[:30])).group(1)); i = t.index("end_header")
    return np.loadtxt(t[i+1:i+1+n], usecols=(0, 1, 2))
MESH = {"human(2cm)": parse_ply(MH), "test(3cm)": parse_ply(MT), "curobo(5cm)": parse_ply(MC)}
CAMS = ("zed1", "zed2")
K = {c: np.array(json.load(open(f"/work/data/live_all/live_calib_1023/000/{c}_info.json"))["K"]).reshape(3, 3) for c in CAMS}
T = {"zed1": np.array(json.load(open("/work/calib_1031/zed1_front_camera_extrinsic.json"))["T_base_cam"]), "zed2": np.array(json.load(open("/work/calib_1031/zed2_top_camera_extrinsic.json"))["T_base_cam"])}
RGB = {c: np.ascontiguousarray(now[f"{c}_img"])[:, :, ::-1].copy() for c in CAMS}      # BGR -> RGB
DEP = {c: now[f"{c}_dep"].astype(np.float32) for c in CAMS}
H, W = 360, 640; vv, uu = np.mgrid[0:H, 0:W]
def cloud(c):
    d = DEP[c]; ok = np.isfinite(d) & (d > 0.3) & (d < 3.5); k = K[c]
    X = np.stack([(uu - k[0, 2]) * d / k[0, 0], (vv - k[1, 2]) * d / k[1, 1], d], -1)
    P = X.reshape(-1, 3) @ T[c][:3, :3].T + T[c][:3, 3]; return P.reshape(H, W, 3), ok
PB, OK = {}, {}
for c in CAMS: PB[c], OK[c] = cloud(c)
# table height: histogram peak of z in the table region (both cameras)
zz = np.concatenate([PB[c][OK[c]][:, 2] for c in CAMS]); xx = np.concatenate([PB[c][OK[c]][:, 0] for c in CAMS]); yy = np.concatenate([PB[c][OK[c]][:, 1] for c in CAMS])
sel = (xx > 0.3) & (xx < 0.9) & (abs(yy) < 0.45) & (zz > -0.04) & (zz < 0.08); hh, ee = np.histogram(zz[sel], bins=np.arange(-0.04, 0.0801, 0.004)); z_t = 0.5 * (ee[np.argmax(hh)] + ee[np.argmax(hh) + 1])
print(f"table height (histogram peak, both cameras, new calibration): {z_t*1000:.0f} mm")
for c in CAMS:
    Pc_ = PB[c][OK[c]]; s_ = (Pc_[:,0]>0.3)&(Pc_[:,0]<0.9)&(abs(Pc_[:,1])<0.45)&(Pc_[:,2]>-0.04)&(Pc_[:,2]<0.10); h_, e_ = np.histogram(Pc_[s_][:,2], bins=np.arange(-0.04,0.1001,0.004)); top=np.argsort(-h_)[:3]
    print(f"   {c}: histogram peaks of z (mm, count): " + ", ".join(f"{(e_[i]+0.002)*1000:.0f}:{h_[i]}" for i in sorted(top)))
print("pixel size at 0.9 m: zed1 %.1f mm/px, zed2 %.1f mm/px (published at 640x360, half of the 1280x720 grab)" % (900/K['zed1'][0,0], 900/K['zed2'][0,0]))
# --- object candidates per camera: height blobs over the table ---------------------------------------------------------------
from sam2.build_sam import build_sam2
from sam2.sam2_image_predictor import SAM2ImagePredictor
dev = "cuda"; pred = SAM2ImagePredictor(build_sam2("configs/sam2.1/sam2.1_hiera_b+.yaml", "/work/sam2/checkpoints/sam2.1_hiera_base_plus.pt", device=dev))
OBJ = {c: [] for c in CAMS}; VIZ = {}
for c in CAMS:
    P = PB[c]; h = P[..., 2] - z_t; ok = OK[c]
    cand = ok & (P[..., 0] > 0.25) & (P[..., 0] < 0.95) & (abs(P[..., 1]) < 0.5) & (h > 0.02) & (h < 0.35)
    cand = ndimage.binary_opening(cand, iterations=1); lab, n = ndimage.label(cand); pred.set_image(RGB[c]); viz = RGB[c].copy()
    for i in range(1, n + 1):
        m = lab == i; area = int(m.sum())
        if area < (60 if c == "zed1" else 150): continue
        dist = ndimage.distance_transform_edt(m); py, px = np.unravel_index(np.argmax(dist), dist.shape)
        pts = [[px, py]]; lbl = [1]
        ring = ndimage.binary_dilation(m, iterations=14) & ~ndimage.binary_dilation(m, iterations=6) & ok & (abs(h) < 0.012)
        ry, rx = np.nonzero(ring)
        if len(ry) >= 4:
            idx = np.linspace(0, len(ry) - 1, 4).astype(int); pts += [[rx[j], ry[j]] for j in idx]; lbl += [0] * 4
        masks, sc, _ = pred.predict(point_coords=np.array(pts, dtype=np.float32), point_labels=np.array(lbl), multimask_output=False)
        sm = masks[0].astype(bool); cc, ncc = ndimage.label(sm); sm = cc == cc[py, px] if cc[py, px] > 0 else sm
        if sm.sum() < 20 or sm.sum() > 6 * area + 200: sm = ndimage.binary_dilation(m, iterations=2)     # SAM spilled or failed: fall back to the blob
        cloud_f = P[sm & ok]; cloud_f = cloud_f[cloud_f[:, 2] - z_t > 0.008]
        er = ndimage.binary_erosion(sm, iterations=3); cloud_e = P[er & ok]; cloud_e = cloud_e[cloud_e[:, 2] - z_t > 0.008]
        if len(cloud_f) < 30: continue
        OBJ[c].append(dict(cam=c, blob_px=area, sam_px=int(sm.sum()), cloud=cloud_f, cloud_er=cloud_e, mask=sm, prompt=(px, py), score=float(sc[0])))
        cnts, _ = cv2.findContours(sm.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE); cv2.drawContours(viz, cnts, -1, (0, 255, 0), 1); cv2.circle(viz, (int(px), int(py)), 3, (255, 0, 0), -1)
    VIZ[c] = viz; print(f"{c}: {len(OBJ[c])} objects segmented (blobs >= min area); SAM masks drawn in green")
def ext(Pc, lo=1, hi=99): return np.percentile(Pc, lo, axis=0), np.percentile(Pc, hi, axis=0)
def fmt(a): return "[" + " ".join(f"{v*100:6.1f}" for v in a) + "]"
# --- match across the two cameras ---------------------------------------------------------------------------------------------
used = set(); pairs = []
for i, a in enumerate(OBJ["zed1"]):
    ca = a["cloud"].mean(0); best = None
    for j, b in enumerate(OBJ["zed2"]):
        if j in used: continue
        d = np.linalg.norm(ca[:2] - b["cloud"].mean(0)[:2])
        if d < 0.12 and (best is None or d < best[0]): best = (d, j)
    if best: used.add(best[1]); pairs.append((a, OBJ["zed2"][best[1]]))
    else: pairs.append((a, None))
for j, b in enumerate(OBJ["zed2"]):
    if j not in used: pairs.append((None, b))
print(f"\n{len(pairs)} object(s) after matching zed1 <-> zed2 ({sum(1 for a,b in pairs if a is not None and b is not None)} seen by both)")
def mesh_obj(M, Pc, rad):
    tree = cKDTree(M); near = set(np.unique(np.concatenate([tree.query_ball_point(p, 0.03 + rad) for p in Pc[::max(1, len(Pc)//60)]]).astype(int))) if len(Pc) else set()
    if not near: return None
    sub = np.array(sorted(near)); sub = sub[M[sub, 2] - z_t > 0.012]; 
    return M[sub] if len(sub) else None
rows = []
for n_, (a, b) in enumerate(pairs):
    print(f"\n=== object {n_+1}")
    parts = [x for x in (a, b) if x is not None]; allc = np.vstack([x["cloud"] for x in parts]); lo, hi = ext(allc); cen = allc.mean(0)
    print(f"  centre (x,y) = ({cen[0]*100:.1f}, {cen[1]*100:.1f}) cm, top height above table p99 = {(np.percentile(allc[:,2],99)-z_t)*100:.1f} cm")
    for x in parts:
        l, h_ = ext(x["cloud"]); l2, h2 = ext(x["cloud_er"]) if len(x["cloud_er"]) > 20 else (l, h_)
        print(f"  {x['cam']}: {len(x['cloud']):5d} pts (SAM {x['sam_px']} px, score {x['score']:.2f}) extent x/y/z cm = {fmt(h_-l)}   | edge-eroded(3px) extent {fmt(h2-l2)}  -> edge pixels add {fmt((h_-l)-(h2-l2))} cm")
    if a is not None and b is not None:
        ca, cb = a["cloud"], b["cloud"]; off = ca.mean(0) - cb.mean(0)
        ta = np.percentile(ca[:, 2], 95) - z_t; tb = np.percentile(cb[:, 2], 95) - z_t; print(f"  CROSS-CAMERA: centroid offset zed1-zed2 (dx,dy,dz) = {np.round(off*100,1).tolist()} cm; top height zed1 {ta*100:.1f} vs zed2 {tb*100:.1f} cm; xy extents zed1 {fmt((ext(ca)[1]-ext(ca)[0])[:2])} / zed2 {fmt((ext(cb)[1]-ext(cb)[0])[:2])}")
    union_ext = hi - lo
    for nm, M in MESH.items():
        mo = mesh_obj(M, allc, 0.05 if "5cm" in nm else (0.03 if "3cm" in nm else 0.02))
        if mo is None: print(f"  mesh {nm}: no vertices near this object"); continue
        ml, mh = ext(mo, 1, 99); print(f"  mesh {nm}: {len(mo):4d} vertices, extent x/y/z cm = {fmt(mh-ml)}   vs raw union {fmt(union_ext)}  -> inflation (mesh - raw) = {fmt((mh-ml)-union_ext)} cm; per-side min {fmt(lo-ml)} max {fmt(mh-hi)}")
        rows.append(((mh - ml) - union_ext)[:2].mean())
print(f"\nmean xy inflation of the meshes over the raw cameras' extent (all objects, both maps): {np.mean(rows)*100:.1f} cm" if rows else "no mesh comparison")
# --- top-down picture -----------------------------------------------------------------------------------------------------------
s = 0.004; x0, x1, y0, y1 = 0.25, 0.95, -0.45, 0.45; imgT = np.full((int((y1-y0)/s), int((x1-x0)/s), 3), 255, np.uint8)
def plot(Pp, col, r=1):
    for p in Pp:
        if x0 <= p[0] < x1 and y0 <= p[1] < y1: cv2.circle(imgT, (int((p[0]-x0)/s), int((y1-p[1])/s)), r, col, -1)
for nm, col, r in (("curobo(5cm)", (255, 160, 0), 3), ("test(3cm)", (255, 0, 255), 2), ("human(2cm)", (0, 170, 0), 1)):
    Mv = MESH[nm]; Mv = Mv[Mv[:, 2] - z_t > 0.012]; plot(Mv, col, r)
for x in OBJ["zed1"]: plot(x["cloud"][::3], (0, 0, 220), 1)
for x in OBJ["zed2"]: plot(x["cloud"][::3], (200, 0, 200), 1)
cv2.putText(imgT, "top view (x right, y up) red=zed1 raw, magenta=zed2 raw, green=mesh 2cm, orange=mesh 5cm", (4, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)
vw = [cv2.resize(VIZ[c], (480, 270)) for c in CAMS]; left = np.vstack(vw); imgT = cv2.resize(imgT, (int(imgT.shape[1] * 540 / imgT.shape[0]), 540)); canvas = np.hstack([left, imgT[:, :, ::-1][:, :, ::-1]])
ok_, buf = cv2.imencode(".jpg", canvas[:, :, ::-1] if False else canvas, [cv2.IMWRITE_JPEG_QUALITY, 88]); sys.stderr.write(base64.b64encode(buf.tobytes()).decode())
