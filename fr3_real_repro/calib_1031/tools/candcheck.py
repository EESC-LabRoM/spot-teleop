import time, json, glob, collections, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
rclpy.init(); n=Node("claude_candcheck_passive")
qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT); D={}
for c in ("zed1","zed2"):
    D[c]={}
    n.create_subscription(Image,f"/{c}/zed_node/depth/depth_registered",lambda m,c=c:D[c].__setitem__("d",m),qos)
    n.create_subscription(CameraInfo,f"/{c}/zed_node/depth/camera_info",lambda m,c=c:D[c].__setitem__("k",m),qos)
t0=time.time()
while time.time()-t0<25 and not all("d" in D[c] and "k" in D[c] for c in D): rclpy.spin_once(n,timeout_sec=0.2)
base="/tmp/nvblox_real/calib/"
cands={"zed1":{"1016":"zed1_front_camera_extrinsic.json.bak_1016","1020":"zed1_front_camera_extrinsic.json.bak_before_1023icp","1023(live)":"zed1_front_camera_extrinsic.json"},
       "zed2":{"1016":"zed2_top_camera_extrinsic.json.bak_1016","1020":"zed2_top_camera_extrinsic.json.bak_before_1023icp","1023(live)":"zed2_top_camera_extrinsic.json"}}
def pts(c,T):
    m=D[c]["d"]; k=D[c]["k"].k; dep=np.frombuffer(bytes(m.data),dtype=np.float32).reshape(m.height,m.width)
    v,u=np.mgrid[0:m.height:4,0:m.width:4]; z=dep[::4,::4]; ok=np.isfinite(z)&(z>0.3)&(z<4); z=z[ok]
    P=np.stack([(u[ok]-k[2])*z/k[0],(v[ok]-k[5])*z/k[4],z],1)@T[:3,:3].T+T[:3,3]
    s=(P[:,2]>-0.15)&(P[:,2]<0.20)&(P[:,0]>0.15)&(P[:,0]<0.9)&(np.abs(P[:,1])<0.6); return P[s]
def cells(Q):
    d=collections.defaultdict(list)
    for x,y,z in Q: d[(int(np.floor(x/0.05)),int(np.floor(y/0.05)))].append(z)
    return {k:np.percentile(v,10) for k,v in d.items() if len(v)>=15}
def load(f):
    d=json.load(open(base+f)); return np.array(d["T_base_cam"])
def tilt(Q):
    rng=np.random.default_rng(0); best=None
    for _ in range(300):
        s=Q[rng.choice(len(Q),3,replace=False)]; nv=np.cross(s[1]-s[0],s[2]-s[0]); nn=np.linalg.norm(nv)
        if nn<1e-9: continue
        nv/=nn
        if abs(nv[2])<0.9: continue
        i=(np.abs((Q-s[0])@nv)<0.01).sum()
        if best is None or i>best[0]: best=(i,nv)
    return np.degrees(np.arccos(abs(best[1][2])))
C={c:{} for c in cands}
for c in cands:
    for name,f in cands[c].items():
        try: Q=pts(c,load(f)); C[c][name]=(cells(Q),tilt(Q) if len(Q)>200 else float("nan"),np.median(Q[:,2][Q[:,2]<np.percentile(Q[:,2],40)]))
        except Exception as e: print(c,name,"ERR",e)
print("per-camera: floor-ish z (median of lowest 40% pts in slab), plane tilt")
for c in C:
    for nm,(cl,tl,zf) in C[c].items(): print(f"  {c} {nm:11s} floor z {zf*1000:6.1f} mm  tilt {tl:4.1f} deg  cells {len(cl)}")
print("zed2 minus zed1 floor in common 5cm cells (median mm / p10..p90) for each pair:")
for n1,(a,_,_) in C["zed1"].items():
    for n2,(b,_,_) in C["zed2"].items():
        cm=set(a)&set(b)
        if len(cm)<8: print(f"  zed1 {n1:11s} x zed2 {n2:11s}: only {len(cm)} common cells"); continue
        dz=np.array([b[k]-a[k] for k in cm])*1000; print(f"  zed1 {n1:11s} x zed2 {n2:11s}: n={len(cm):2d}  median {np.median(dz):6.1f}  p10..p90 {np.percentile(dz,10):6.1f}..{np.percentile(dz,90):6.1f}")
