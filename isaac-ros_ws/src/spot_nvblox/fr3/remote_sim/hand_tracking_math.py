"""Same-frame depth fusion and adaptive position filtering, independent of ROS."""
import math
import numpy as np

# Upright camera facing the operator: toward camera -> robot forward, image up -> robot up.
CAMERA_TO_ROBOT = np.array([[0., 0., -1.], [1., 0., 0.], [0., -1., 0.]])


class OneEuroPosition:
    def __init__(self, min_cutoff=1.5, beta=5., derivative_cutoff=1.):
        self.min_cutoff, self.beta, self.derivative_cutoff = min_cutoff, beta, derivative_cutoff
        self.previous = self.filtered = self.derivative = self.stamp = None

    @staticmethod
    def alpha(cutoff, dt):
        return 1. / (1. + 1. / (2. * math.pi * cutoff * dt))

    def update(self, value, stamp):
        value = np.asarray(value, float)
        if value.shape != (3,) or not np.all(np.isfinite(value)):
            raise ValueError('posição inválida')
        if self.stamp is not None and stamp <= self.stamp:
            raise ValueError('timestamp repetido')
        if self.stamp is None or stamp-self.stamp > .65:
            self.filtered, self.previous, self.derivative = value.copy(), value.copy(), np.zeros(3)
        else:
            dt = stamp-self.stamp
            if np.linalg.norm(value-self.previous) > .03 + 2.*dt:
                raise ValueError('salto na profundidade da mão')
            a = self.alpha(self.derivative_cutoff, dt)
            self.derivative = a*(value-self.previous)/dt + (1-a)*self.derivative
            a = self.alpha(self.min_cutoff+self.beta*np.linalg.norm(self.derivative), dt)
            self.filtered = a*value+(1-a)*self.filtered
            self.previous = value.copy()
        self.stamp = stamp
        return self.filtered.copy()


def fused_wrist_depth(measurements, world_z, limits=(.3, 3.)):
    """Estimate wrist depth from current ZED samples at wrist and four knuckles.

    Each measurement is corrected by the model's relative knuckle-to-wrist Z.
    A consistent cluster avoids a wrist silhouette/background measurement. No
    historical depth, prediction or stale frame is used.
    """
    values=[]
    for i,depth in enumerate(measurements):
        if depth is not None and math.isfinite(depth):
            z=float(depth)+float(world_z[0])-float(world_z[i])
            if limits[0]<z<limits[1]:values.append((z,i))
    if not values:return None,0
    values.sort()
    groups=[]
    for v in values:
        if not groups or v[0]-groups[-1][-1][0]>.08:groups.append([])
        groups[-1].append(v)
    supported=[g for g in groups if len(g)>=2 and g[-1][0]-g[0][0]<.10]
    if supported:
        group=max(supported,key=lambda g:(len(g),-g[0][0]))
        return float(np.median([v[0] for v in group])),len(group)
    # A directly measured wrist remains valid even when all knuckles are occluded.
    for value,index in values:
        if index==0:return value,1
    return None,0


def palm_depth_sites(pixels,world_points):
    """Current wrist/MCP pixels plus interior palm sites, with matching model Z."""
    pixels=np.asarray(pixels,float);world_points=np.asarray(world_points,float)
    if pixels.shape!=(21,2) or world_points.shape!=(21,3):raise ValueError('invalid hand landmarks')
    ids=(0,5,9,13,17)
    sites=[pixels[i] for i in ids];z=[world_points[i,2] for i in ids]
    for group in ((0,5,9),(0,9,13),(0,13,17),(5,9,13,17)):
        sites.append(pixels[list(group)].mean(0));z.append(float(world_points[list(group),2].mean()))
    if not np.all(np.isfinite(np.r_[np.asarray(sites).reshape(-1),z])):raise ValueError('invalid hand depth sites')
    return np.asarray(sites),z


def finger_distance_signals(points):
    """Vote from four fingers using extension AND joint bend; thumb is optional.

    Normalized finger-chain geometry follows G1's grip idea. Current measured
    landmarks only: ambiguous/pointing poses do not create a gripper intent.
    """
    p=np.asarray(points,float)
    if p.shape!=(21,3) or not np.all(np.isfinite(p)):return None
    chains=((0,1,2,3,4),(0,5,6,7,8),(0,9,10,11,12),(0,13,14,15,16),(0,17,18,19,20))
    lengths=np.array([np.linalg.norm(np.diff(p[list(chain)],axis=0),axis=1).sum() for chain in chains])
    valid=(lengths>.035)&(lengths<.4)
    if np.count_nonzero(valid[1:])<3:return None
    extension=np.linalg.norm(p[[4,8,12,16,20]]-p[0],axis=1)/np.maximum(lengths,1e-9)
    closure=np.clip((.90-extension)/(.90-.65),0.,1.)
    opened=[];folded=[]
    for finger,chain in enumerate(chains[1:],1):
        _,mcp,pip,dip,tip=chain
        bones=np.diff(p[[mcp,pip,dip,tip]],axis=0)
        norms=np.linalg.norm(bones,axis=1)
        if not valid[finger] or np.any(norms<.003):continue
        pip_cos=float(bones[0]@bones[1]/(norms[0]*norms[1]))
        dip_cos=float(bones[1]@bones[2]/(norms[1]*norms[2]))
        reach=np.linalg.norm(p[tip]-p[mcp])/norms.sum()
        opened.append(bool(extension[finger]>.78 and reach>.8 and pip_cos>.65 and dip_cos>.5))
        folded.append(bool(extension[finger]<.75 and (pip_cos<.5 or reach<.65)))
    if len(opened)<3:return None
    gesture=None
    if sum(opened)>=3:gesture='Open_Palm'
    elif sum(folded)>=3 and not any(opened):gesture='Closed_Fist'
    grip=float(np.median(closure[2:][valid[2:]]))
    pinch=None
    if valid[0] and valid[2]:
        pinch=float(np.clip((.40-np.linalg.norm(p[4]-p[8])/lengths[2])/(.40-.08),0.,1.))
    return dict(grip=grip,pinch=pinch,gesture=gesture,open_fingers=sum(opened),closed_fingers=sum(folded))
