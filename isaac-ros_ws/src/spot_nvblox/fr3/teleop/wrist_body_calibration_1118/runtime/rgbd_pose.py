"""Offline combined-pose candidate: metric wrist in one fixed camera/robot frame.

No ROS publishers, robot connections, motion classification or rotation-only lock.
"""
import math
import numpy as np
# Runtime geometry depends only on NumPy.

CAMERA_TO_ROBOT=np.array([[0.,0.,-1.],[1.,0.,0.],[0.,-1.,0.]])
HAND_TO_TCP=np.array([[0.,0.,1.],[0.,1.,0.],[-1.,0.,0.]])
PALM=[0,5,9,13,17]


def sites(pix,world):
    p=np.asarray(pix,float);w=np.asarray(world,float)
    if p.shape!=(21,2) or w.shape!=(21,3) or not np.isfinite(p).all() or not np.isfinite(w).all():
        raise ValueError("invalid hand landmarks")
    # Interior palm points reduce silhouette/background contamination.
    groups=[(0,5,9),(0,9,13),(0,13,17),(5,9,13,17)]
    xy=[p[list(g)].mean(0) for g in groups]
    wz=[w[list(g),2].mean() for g in groups]
    for k in [5,9,13,17]:
        for f in [.4,.7]:
            xy.append((1-f)*p[0]+f*p[k])
            wz.append((1-f)*w[0,2]+f*w[k,2])
    return np.array(xy),np.array(wz)


def patch_depth(depth,u,v,radius=2):
    h,w=depth.shape;u=int(round(u));v=int(round(v))
    if not (radius<=u<w-radius and radius<=v<h-radius):return None
    a=depth[v-radius:v+radius+1,u-radius:u+radius+1]
    a=np.sort(a[np.isfinite(a)&(a>.3)&(a<3.)])
    if len(a)<max(3,int(.3*(2*radius+1)**2)):return None
    # Use the dominant local depth cluster, not automatically the nearest one.
    groups=np.split(a,np.flatnonzero(np.diff(a)>.04)+1)
    best=max(groups,key=len)
    if len(best)<max(3,.4*(2*radius+1)**2) or np.ptp(best)>.08:return None
    return float(np.median(best))


def measure_wrist(pix,world,depth,K):
    """Estimate the same wrist landmark from current interior depths and model offsets.

    A robust shared wrist Z is inferred from interior palm measurements. X/Y are
    a consensus over wrist and MCP reprojections using that Z and current relative
    model geometry. Orientation never rotates the translation reference.
    """
    pix=np.asarray(pix,float);world=np.asarray(world,float);K=np.asarray(K,float)
    xy,wz=sites(pix,world)
    samples=[]
    for point,site_z in zip(xy,wz):
        z=patch_depth(depth,*point)
        if z is not None:samples.append(z+world[0,2]-site_z)
    a=np.array(samples)
    if len(a)<4:raise ValueError("insufficient current palm depth")
    # Fixed-width consensus prevents a chain of individually close background
    # samples from bridging foreground to background.
    groups=[a[np.abs(a-c)<=.035] for c in a]
    g=max(groups,key=lambda g:(len(g),-float(np.ptp(g))))
    if len(g)<max(4,math.ceil(.5*len(a))):raise ValueError("palm depth has no consensus")
    z=float(np.median(g))
    if not .3<z<3.:raise ValueError("wrist depth out of range")
    rel=world[PALM]-world[0]
    q=pix[PALM]
    zsites=z+rel[:,2]
    tx=(q[:,0]-K[0,2])/K[0,0]*zsites-rel[:,0]
    ty=(q[:,1]-K[1,2])/K[1,1]*zsites-rel[:,1]
    x,y=float(np.median(tx)),float(np.median(ty))
    residual=np.sqrt((tx-x)**2+(ty-y)**2)
    if np.median(residual)>.025:raise ValueError("RGB/model reprojection inconsistent")
    return np.array([x,y,z]),dict(depth_support=len(g),depth_candidates=len(a),
            depth_spread_m=round(float(np.ptp(g)),6),reprojection_residual_m=round(float(np.median(residual)),6))


class FixedFramePosition:
    """Causal metric position filter with a calibration anchor that never follows a shoulder.

    Same gain on every axis, no deadband and no rotation-dependent position rule.
    Gaps require two consistent current measurements but never reset the anchor.
    """
    def __init__(self,gain=1.,tau=.06,max_speed=1.5):
        if gain<=0 or tau<=0:raise ValueError("positive gain/tau required")
        self.gain=float(gain);self.tau=tau;self.max_speed=max_speed
        self.anchor=None;self.filtered=None;self.raw=None;self.stamp=None
        self.pending=None;self.pending_t=None;self.pending_n=0
        self.reason="waiting for position"
    def update(self,p,t):
        if self.stamp is not None and t<=self.stamp:
            self.reason="nonmonotonic position timestamp";return None
        if p is None:
            self.pending=None;self.pending_n=0;self.pending_t=None
            self.reason="position unavailable";return None
        p=np.asarray(p,float)
        if p.shape!=(3,) or not np.isfinite(p).all():
            self.reason="invalid position";return None
        recover=self.stamp is None or t-self.stamp>.25
        dt=0. if self.stamp is None else t-self.stamp
        jump=not recover and np.linalg.norm(p-self.raw)>.025+self.max_speed*dt
        if recover or jump:
            if self.pending is not None and t-self.pending_t<.1 and np.linalg.norm(p-self.pending)<.025:
                self.pending_n+=1
            else:self.pending=p.copy();self.pending_n=1
            self.pending_t=t
            if self.pending_n<(2 if recover else 3):
                self.reason="confirming position reacquisition" if recover else "confirming depth jump"
                return None
        self.pending=None;self.pending_n=0;self.pending_t=None
        if self.anchor is None:self.anchor=p.copy()
        if recover:self.filtered=p.copy()
        else:
            alpha=1-math.exp(-dt/self.tau)
            self.filtered+=alpha*(p-self.filtered)
        self.raw=p.copy();self.stamp=t;self.reason="accepted in fixed frame"
        return self.gain*(CAMERA_TO_ROBOT@(self.filtered-self.anchor))
