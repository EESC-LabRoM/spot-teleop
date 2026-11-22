"""Robust current-frame palm fit and quaternion smoothing. No predicted poses."""
import itertools,math
import numpy as np


def angle(a,b):return 2*math.acos(float(np.clip(abs(a@b),0.,1.)))

def blend(a,b,t):
    dot=float(a@b)
    if dot<0:b=-b;dot=-dot
    dot=float(np.clip(dot,0.,1.))
    if dot>.9995:q=(1-t)*a+t*b
    else:
        theta=math.acos(dot);q=(math.sin((1-t)*theta)*a+math.sin(t*theta)*b)/math.sin(theta)
    return q/np.linalg.norm(q)


class AdaptivePalmOrientation:
    def __init__(self,min_cutoff=1.5,beta=1.5,max_speed=math.radians(120),timeout=.65):
        self.min_cutoff,self.beta,self.max_speed,self.timeout=min_cutoff,beta,max_speed,timeout
        self.q=self.raw=self.stamp=None;self.observed_stamp=None;self.pending=[];self.speed=0.

    def update(self,value,stamp):
        q=np.asarray(value,float)
        if q.shape!=(4,) or not np.all(np.isfinite(q)) or abs(np.linalg.norm(q)-1)>.01:raise ValueError('invalid palm orientation')
        q=q/np.linalg.norm(q)
        if not math.isfinite(stamp) or (self.observed_stamp is not None and stamp<=self.observed_stamp):raise ValueError('orientation timestamp replay')
        self.observed_stamp=stamp
        if self.q is None:self.q=q.copy();self.raw=q.copy();self.stamp=stamp;return q.copy()
        dt=stamp-self.stamp;lost=dt>self.timeout
        jump=angle(self.raw,q)>max(math.radians(45),2*self.max_speed*min(dt,.2))
        if lost or jump:
            if self.pending:
                gap=stamp-self.pending[-1][0]
                # Confirm coherent current motion at the measured capture cadence.
                # A fixed 15-degree / .35-second window could never reacquire
                # legitimate turns when the RGB-D transport ran at 2 Hz.
                allowed=max(math.radians(15),1.25*self.max_speed*gap)
                if gap>self.timeout or angle(q,self.pending[-1][1])>allowed:self.pending=[]
            self.pending.append((stamp,q.copy()))
            required=2 if lost else 3
            if len(self.pending)<required:raise ValueError('Confirmando orientacao atual da palma')
            if lost:
                self.q=q.copy();self.raw=q.copy();self.stamp=stamp;self.pending=[];self.speed=0.;return q.copy()
        self.pending=[]
        speed=angle(self.raw,q)/dt
        da=1/(1+1/(2*math.pi*dt));self.speed=da*speed+(1-da)*self.speed
        cutoff=self.min_cutoff+self.beta*self.speed
        alpha=1/(1+1/(2*math.pi*cutoff*dt))
        amount=min(alpha,self.max_speed*dt/max(angle(self.q,q),1e-9))
        self.q=blend(self.q,q,amount);self.raw=q.copy();self.stamp=stamp
        return self.q.copy()


class RobustPalmFrame:
    """Fit a per-person five-point palm shape; exclude one inconsistent MCP.

    The wrist must remain in the fit. Four current inliers and non-collinear
    support are required. The latched template supplies shape, never motion.

    The template is the median palm-frame shape of `bootstrap` consistent frames
    (one MediaPipe frame can be distorted, e.g. a hand at the image edge, and a
    bad template then either jitters by switching 4-point subsets or fails the
    size check for the rest of the session). While unlatched, the caller's seed
    rotation is the output. A template whose median fit over the last `window`
    frames exceeds `refit_mm` (failed fits count as bad) is dropped and rebuilt
    from current frames.
    """
    def __init__(self,bootstrap=5,refit_mm=5.,window=15,consistency_mm=6.):
        self.bootstrap,self.refit,self.window,self.consistency=bootstrap,refit_mm/1000.,window,consistency_mm/1000.
        self.template=None;self.shapes=[];self.fits=[];self.relatches=0

    def health(self,fit):
        self.fits=(self.fits+[math.inf if fit is None else fit])[-self.window:]
        if len(self.fits)>=self.window and float(np.median(self.fits))>self.refit:
            self.template=None;self.shapes=[];self.fits=[];self.relatches+=1

    def update(self,points,initial_rotation):
        p=np.asarray(points,float)
        if p.shape!=(5,3) or not np.all(np.isfinite(p)):raise ValueError('invalid current palm geometry')
        if self.template is None:
            if initial_rotation is None:raise ValueError('palm template needs a seed rotation')
            seed=np.asarray(initial_rotation,float)
            self.shapes=(self.shapes+[(p-p[0])@seed])[-self.bootstrap:]
            quality=dict(orientation_inliers=5,orientation_fit_mm=0.,palm_template=f'{len(self.shapes)}/{self.bootstrap}',palm_relatches=self.relatches)
            if len(self.shapes)>=self.bootstrap:
                median=np.median(self.shapes,axis=0)
                if max(float(np.sqrt(np.mean(np.sum((s-median)**2,axis=1)))) for s in self.shapes)<=self.consistency:
                    self.template=median;self.shapes=[];self.fits=[];quality['palm_template']='latched'
            return seed.copy(),quality
        try:rotation,quality=self.fit(p)
        except ValueError:
            self.health(None);raise
        self.health(quality['orientation_fit_mm']/1000.)
        quality.update(palm_template='latched' if self.template is not None else 'relatch',palm_relatches=self.relatches)
        return rotation,quality

    def fit(self,p):
        lengths=np.linalg.norm(p[1:]-p[0],axis=1)
        base=np.linalg.norm(self.template[1:]-self.template[0],axis=1)
        scale=float(np.median(lengths/base))
        if not .65<scale<1.5:raise ValueError('Palm size changed inconsistently')
        source=self.template*scale;best=None
        for subset in [tuple(range(5))]+[(0,)+x for x in itertools.combinations(range(1,5),3)]:
            ids=list(subset);a=source[ids];b=p[ids];ac=a-a.mean(0);bc=b-b.mean(0)
            u,sv,vt=np.linalg.svd(ac.T@bc)
            if sv[1]<.06*sv[0]:continue
            fix=np.eye(3);fix[2,2]=np.linalg.det(vt.T@u.T)
            rotation=vt.T@fix@u.T
            translation=np.mean(b,axis=0)-np.mean(a,axis=0)@rotation.T
            residual=np.linalg.norm(source@rotation.T+translation-p,axis=1)
            inliers=residual<.012
            if not inliers[0] or np.count_nonzero(inliers)<4:continue
            score=(int(inliers.sum()),-float(np.mean(residual[inliers]**2)))
            if best is None or score>best[0]:best=(score,rotation,residual,inliers)
        if best is None:raise ValueError('Palma inconsistente: aguardando pontos atuais estaveis')
        _,rotation,residual,inliers=best
        return rotation,dict(orientation_inliers=int(inliers.sum()),orientation_fit_mm=round(float(np.sqrt(np.mean(residual[inliers]**2)))*1000,2))
