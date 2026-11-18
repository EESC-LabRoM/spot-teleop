"""Robust current-frame palm fit and quaternion smoothing. No predicted poses."""
import itertools,math
import numpy as np


def finger_axis_yaw(palm_rotation_camera,camera_to_robot,min_horizontal=.5):
    """Yaw (rotation about the robot's vertical z, CCW seen from above) of the palm FINGER axis.

    palm_rotation_camera: 3x3 palm frame in the camera frame (column 0 = wrist -> knuckles, hand_frame.palm_rotation).
    Returns radians in (-pi, pi], or None when the finger axis is too close to vertical (horizontal part < min_horizontal).

    Benchmark vs Meta Quest (13 takes, per-take yaw offset fitted on Right-labeled frames): finger-axis yaw error
    5/17 deg (median/p90) on Right-labeled frames and 22/59 deg on Left-labeled ones, the least label-sensitive of the palm axes
    (palm normal 44/143, across-palm 82/148). No handedness gate is needed for this quantity.
    """
    f=np.asarray(camera_to_robot,float)@np.asarray(palm_rotation_camera,float)[:,0]
    horizontal=math.hypot(f[0],f[1])
    if horizontal<min_horizontal*np.linalg.norm(f):return None
    return math.atan2(f[1],f[0])


class YawFilter:
    """Smoothed, rate-limited, UNWRAPPED yaw for translation+yaw teleop.

    update(yaw, stamp) takes the raw finger-axis yaw (radians, or None when it is unavailable) and returns the filtered
    unwrapped yaw, or None when the sample was not used (no yaw, duplicate stamp, or a large jump that is not yet confirmed); the previous
    value then stays as `value` and `age(stamp)` says how stale it is. Unwrapping keeps q=Rz(value) continuous through +-180 deg.
    Jumps above `jump_deg` need `confirm` consecutive agreeing samples; the output moves at most max_rate_deg/s (and max_step_deg per
    update) so the tracker's OrientationFilter jump gate (45 deg) is never hit. After `hold_timeout` seconds without an accepted
    sample the next valid yaw re-seeds the filter (the caller publishes orientation_valid=false meanwhile, so the consumer re-anchors).
    """
    def __init__(self,tau=.12,max_rate_deg=100.,max_step_deg=40.,jump_deg=60.,confirm=2,hold_timeout=1.0):
        self.tau,self.max_rate,self.max_step=tau,math.radians(max_rate_deg),math.radians(max_step_deg)
        self.jump,self.confirm,self.hold_timeout=math.radians(jump_deg),confirm,hold_timeout
        self.reset()

    def reset(self):
        self.value=None;self.stamp=None;self.pending=None;self.pending_n=0

    @staticmethod
    def wrap(a):return (a+math.pi)%(2*math.pi)-math.pi

    def age(self,stamp):
        return None if self.stamp is None else stamp-self.stamp

    def update(self,yaw,stamp):
        if yaw is None:return None
        if self.stamp is not None and stamp<=self.stamp:return None  # duplicate/older stamp: hold, never raise (a raise would drop the gesture)
        if self.value is None or stamp-self.stamp>self.hold_timeout:
            self.value,self.stamp,self.pending,self.pending_n=float(yaw),stamp,None,0
            return self.value
        d=self.wrap(yaw-self.value)
        if abs(d)>self.jump:
            if self.pending is not None and abs(self.wrap(yaw-self.pending))<math.radians(20.):self.pending_n+=1
            else:self.pending,self.pending_n=yaw,1
            if self.pending_n<self.confirm:return None
        self.pending,self.pending_n=None,0
        dt=stamp-self.stamp
        step=(1-math.exp(-dt/self.tau))*d
        cap=min(self.max_rate*dt,self.max_step)
        self.value+=max(-cap,min(cap,step));self.stamp=stamp
        return self.value


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


RELATCH_MIN_HISTORY=6  # share rule looks at >= this many recent palm frames even when they span > relatch_window_s
PALM_MCP_MIN_M,PALM_MCP_MAX_M=.035,.15  # template latch plausibility: every wrist->MCP length (metric world points)


class RobustPalmFrame:
    """Fit a per-person five-point palm shape; exclude one inconsistent MCP.

    The wrist must remain in the fit. Four current inliers and non-collinear
    support are required. The latched template supplies shape, never motion.

    Re-latch (v4, 2026-11-14): the template is latched from the FIRST palm seen. A bad first palm (wrong depth, a fist,
    the other hand) made every later frame fail the scale check for the whole session ('Palm size changed inconsistently'
    on 88% of the 11-13 live frames; a restart gave 99% valid). Every template failure counts (scale OR shape fit, e.g. a template
    latched on the other hand). With stamps: the template is dropped when, over the last `relatch_window_s`, at least
    `relatch_frames` palm frames failed, they are >= `relatch_share` of the palm frames in that window, and the failures span
    >= `relatch_s`. A window share (v5.1) rather than a consecutive run, because a scale-poisoned template fails in flickering
    singles/short runs between passes (forensics lane 2f61d35a; z12 replay: 72 rejections, 1 re-latch with the consecutive rule).
    Without stamps: `relatch_frames` consecutive failures. The caller passes a fresh seed on the next frame (it does so whenever
    `template is None`). `relatches` counts re-latches for diagnostics.

    Gated (re)latch (v5.2): with latch_frames=K>1 a template is only latched after K consecutive plausible palms whose wrist->MCP
    lengths agree (median ratio to the candidates' median within latch_agree) with gaps <= latch_gap_s; until then
    'Confirmando palma k/K' is raised. Forensics (lane 2f61d35a): the 60%/1 s trigger also fires on healthy templates during a
    Left/Right label-flicker burst (z01 3, z08 2 spurious re-latches) and each reseed re-rolls a ~13%-bad latch lottery; gating the
    seed makes spurious triggers harmless and protects the FIRST latch too. The class default (1) keeps the original behaviour;
    the tracker uses PALM_LATCH_FRAMES.
    """
    def __init__(self,relatch_s=.5,relatch_frames=3,relatch_window_s=1.,relatch_share=.6,latch_frames=1,latch_agree=(.85,1.18),latch_gap_s=.5):
        self.template=None;self.relatch_s,self.relatch_frames=relatch_s,relatch_frames
        self.relatch_window_s,self.relatch_share=relatch_window_s,relatch_share
        self.latch_frames,self.latch_agree,self.latch_gap_s=latch_frames,latch_agree,latch_gap_s
        self.fail_n=0;self.history=[];self.relatches=0;self.candidates=[]

    def _latch(self,p,initial_rotation,stamp):
        """Collect consistent plausible palms; latch the template from the K-th. Returns the seed rotation or raises."""
        if initial_rotation is None:raise ValueError('palm template needs a seed rotation')
        reach=np.linalg.norm(p[1:]-p[0],axis=1)  # plausibility (franka_teleop lane 3e8e6aec): adult wrist->MCP lengths only
        if not np.all((PALM_MCP_MIN_M<reach)&(reach<PALM_MCP_MAX_M)):
            self.candidates=[];raise ValueError('Implausible palm geometry; not latching template')
        if self.candidates:
            last=self.candidates[-1][0];ref=np.median([c[1] for c in self.candidates],axis=0)
            ratio=float(np.median(reach/ref))
            stale=stamp is not None and last is not None and stamp-last>self.latch_gap_s
            if stale or not self.latch_agree[0]<ratio<self.latch_agree[1]:self.candidates=[]
        self.candidates.append((stamp,reach))
        if len(self.candidates)<self.latch_frames:
            raise ValueError(f'Confirmando palma {len(self.candidates)}/{self.latch_frames} antes de fixar o modelo')
        self.template=(p-p[0])@initial_rotation;self.fail_n=0;self.history=[];self.candidates=[]
        return np.asarray(initial_rotation).copy(),dict(orientation_inliers=5,orientation_fit_mm=0.)

    def _record(self,stamp,ok):
        """Palm-frame history for the share rule: the last relatch_window_s, but never fewer than the last RELATCH_MIN_HISTORY
        frames, so a slow palm rate (< ~3 Hz: < 3 frames per second) can still trigger (review note 3, lane 2f61d35a)."""
        if stamp is None:return
        h=self.history+[(stamp,ok)]
        recent=[(t,o) for t,o in h if stamp-t<=self.relatch_window_s]
        self.history=recent if len(recent)>=RELATCH_MIN_HISTORY else h[-RELATCH_MIN_HISTORY:]

    def _fail(self,stamp,message):
        """Record a template failure and drop the template when the failure share criterion holds. Raises always."""
        self.fail_n+=1;self._record(stamp,False)
        if stamp is None:trigger=self.fail_n>=self.relatch_frames
        else:
            fails=[t for t,o in self.history if not o]
            trigger=(len(fails)>=self.relatch_frames and len(fails)>=self.relatch_share*len(self.history)
                     and fails[-1]-fails[0]>=self.relatch_s)
        if trigger:
            self.template=None;self.fail_n=0;self.history=[];self.candidates=[];self.relatches+=1
            raise ValueError(message+'; template re-latched')
        raise ValueError(message)

    def update(self,points,initial_rotation,stamp=None):
        p=np.asarray(points,float)
        if p.shape!=(5,3) or not np.all(np.isfinite(p)):raise ValueError('invalid current palm geometry')
        if self.template is None:return self._latch(p,initial_rotation,stamp)
        lengths=np.linalg.norm(p[1:]-p[0],axis=1)
        base=np.linalg.norm(self.template[1:]-self.template[0],axis=1)
        scale=float(np.median(lengths/base))
        if not .65<scale<1.5:self._fail(stamp,'Palm size changed inconsistently')
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
        if best is None:self._fail(stamp,'Palma inconsistente: aguardando pontos atuais estaveis')
        self.fail_n=0;self._record(stamp,True)  # a fully fitted frame: clears the no-stamp run, counts as a pass in the window
        _,rotation,residual,inliers=best
        return rotation,dict(orientation_inliers=int(inliers.sum()),orientation_fit_mm=round(float(np.sqrt(np.mean(residual[inliers]**2)))*1000,2))
