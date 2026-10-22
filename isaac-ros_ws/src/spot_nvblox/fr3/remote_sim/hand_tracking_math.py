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


def finger_distance_signals(points):
    """G1 main grip.py idea: normalized wrist-tip extension, median of three fingers.

    Uses the observed bone-chain lengths rather than G1/Dex5 dimensions. Pinch is
    diagnostic only for the Franka parallel gripper. Ambiguous geometry yields no
    gripper command. Reference: EESC-LabRoM/G1_teleop, main 181e736, dex_hand/grip.py.
    """
    p=np.asarray(points,float)
    if p.shape!=(21,3) or not np.all(np.isfinite(p)):return None
    chains=((0,1,2,3,4),(0,5,6,7,8),(0,9,10,11,12),(0,13,14,15,16),(0,17,18,19,20))
    lengths=np.array([np.linalg.norm(np.diff(p[list(chain)],axis=0),axis=1).sum() for chain in chains])
    if np.any(lengths<.04) or np.any(lengths>.35):return None
    extension=np.linalg.norm(p[[4,8,12,16,20]]-p[0],axis=1)/lengths
    closure=np.clip((.90-extension)/(.90-.65),0.,1.)
    grip=float(np.median(closure[2:]))
    pinch=float(np.clip((.40-np.linalg.norm(p[4]-p[8])/lengths[2])/(.40-.08),0.,1.))
    gesture=None
    if grip<.10 and closure[1]<.25:gesture='Open_Palm'
    elif grip>.85 and closure[1]>.65:gesture='Closed_Fist'
    return dict(grip=grip,pinch=pinch,gesture=gesture)
