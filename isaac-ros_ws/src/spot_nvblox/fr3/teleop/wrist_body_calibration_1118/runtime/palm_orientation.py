"""Offline RGB hand orientation experiment. Does not publish or command a robot."""
import math
from collections import deque
import numpy as np

CAMERA_TO_ROBOT = np.array([[0.,0.,-1.],[1.,0.,0.],[0.,-1.,0.]])
HAND_TO_TCP = np.array([[0.,0.,1.],[0.,1.,0.],[-1.,0.,0.]])
PALM = [0,5,9,13,17]


def palm_frame(world):
    """Anatomical frame in MediaPipe world-landmark axes, without depth pixels.

    +x wrist -> mean MCP, +y little -> index, +z x cross y.
    The world landmarks are model estimates, not metric ground truth.
    """
    p = np.asarray(world, float)
    if p.shape != (21,3) or not np.isfinite(p).all():
        raise ValueError("invalid landmarks")
    p = p[PALM]
    lengths = np.linalg.norm(p[1:]-p[0], axis=1)
    width = np.linalg.norm(p[1]-p[4])
    if np.any(lengths < .025) or np.any(lengths > .18) or not .025 < width < .16:
        raise ValueError("implausible palm size")
    x = p[1:].mean(0)-p[0]
    x /= np.linalg.norm(x)
    side = p[1]-p[4]
    side -= np.dot(side,x)*x
    if np.linalg.norm(side)/width < .35:
        raise ValueError("degenerate palm")
    y = side/np.linalg.norm(side)
    z = np.cross(x,y)
    if np.max(np.abs((p-p.mean(0))@z)) > .025:
        raise ValueError("nonplanar palm")
    return np.column_stack((x,y,z))


def matrix_quaternion(r):
    r=np.asarray(r,float);q=np.empty(4);tr=float(np.trace(r))
    if tr>0:
        s=math.sqrt(1+tr)*2
        q[:]=[(r[2,1]-r[1,2])/s,(r[0,2]-r[2,0])/s,(r[1,0]-r[0,1])/s,s/4]
    else:
        i=int(np.argmax(np.diag(r)));j=(i+1)%3;k=(i+2)%3
        s=math.sqrt(max(0.,1+r[i,i]-r[j,j]-r[k,k]))*2
        q[i]=s/4;q[j]=(r[j,i]+r[i,j])/s;q[k]=(r[k,i]+r[i,k])/s;q[3]=(r[k,j]-r[j,k])/s
    return q/np.linalg.norm(q)


def quaternion_matrix(q):
    x,y,z,w=np.asarray(q,float)/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def angular_distance(a,b):
    qa,qb=matrix_quaternion(a),matrix_quaternion(b)
    return 2*math.acos(float(np.clip(abs(qa@qb),0.,1.)))


def blend(a,b,fraction):
    qa,qb=matrix_quaternion(a),matrix_quaternion(b)
    dot=float(qa@qb)
    if dot<0:qb=-qb;dot=-dot
    dot=float(np.clip(dot,0.,1.));f=float(np.clip(fraction,0.,1.))
    if dot>.9995:q=(1-f)*qa+f*qb
    else:
        theta=math.acos(dot)
        q=(math.sin((1-f)*theta)*qa+math.sin(f*theta)*qb)/math.sin(theta)
    return quaternion_matrix(q)


class PalmFilter:
    """SO(3) smoothing with confirmation and explicit missing data.

    No held pose is returned when measurement is missing/rejected. Reacquisition
    after a long gap needs two consistent measurements.
    """
    def __init__(self,tau=.06,max_rate_deg=300.,gap=.25,jump_deg=70.):
        self.tau=tau; self.rate=math.radians(max_rate_deg)
        self.gap=gap; self.jump=math.radians(jump_deg)
        self.last=None; self.last_t=None; self.pending=None; self.pending_n=0
        self.pending_t=None; self.reason="waiting"
    def update(self,raw,t):
        if self.last_t is not None and t <= self.last_t:
            self.reason="nonmonotonic timestamp"; return None
        if raw is None:
            self.pending=None; self.pending_n=0; self.pending_t=None
            self.reason="no current palm"; return None
        raw=np.asarray(raw,float)
        if raw.shape!=(3,3) or not np.isfinite(raw).all() or not np.allclose(raw.T@raw,np.eye(3),atol=1e-5) or np.linalg.det(raw)<.99:
            self.reason="invalid rotation"; return None
        recovering=self.last is None or t-self.last_t>self.gap
        dt=0. if self.last_t is None else t-self.last_t
        distance=0. if self.last is None else angular_distance(self.last,raw)
        if recovering or distance>self.jump:
            if self.pending is not None and t-self.pending_t<=.1 and angular_distance(self.pending,raw)<math.radians(20.):
                self.pending_n+=1
            else:
                self.pending=raw.copy(); self.pending_n=1
            self.pending_t=t
            required=2 if recovering else 3
            if self.pending_n<required:
                self.reason="confirming reacquisition" if recovering else "confirming large rotation"
                return None
        self.pending=None; self.pending_n=0; self.pending_t=None
        if recovering:
            self.last=raw.copy()
        else:
            fraction=1-math.exp(-dt/self.tau)
            if distance>1e-10: fraction=min(fraction,self.rate*dt/distance)
            self.last=blend(self.last,raw,fraction)
        self.last_t=t; self.reason="accepted"
        return self.last.copy()


class SpotRoll:
    """Spot roll equations, EMA .2, 90deg offset, .2s delay and fist block.

    Uses capture time instead of wall time; gestures are saved classifications.
    Offline reconstruction, not a recording of ROS output.
    """
    def __init__(self,alpha=.2,offset_deg=90.,delay=.20):
        self.alpha=alpha; self.offset=math.radians(offset_deg); self.delay=delay
        self.s=None; self.c=None; self.queue=deque(); self.last=None
        self.gesture=0; self.until=-1.; self.output=None; self.raw_angle=None
    def update(self,world,t,gesture=""):
        prev=self.gesture
        if gesture=="Closed_Fist": self.gesture=1
        elif gesture=="Open_Palm": self.gesture=0
        if prev==0 and self.gesture==1: self.until=t+2.
        blocked=self.gesture==1 and t<self.until and self.last is not None
        if blocked: self.output=self.last
        if world is not None:
            p=np.asarray(world,float)
            normal=-np.cross(p[17]-p[0],p[5]-p[17])
            n=np.linalg.norm(normal)
            if n>1e-6:
                normal/=n
                raw=-math.atan2(normal[1],normal[0])+self.offset
                self.raw_angle=raw
                s,c=math.sin(raw),math.cos(raw)
                if self.s is None: self.s,self.c=s,c
                else:
                    self.s=self.alpha*s+(1-self.alpha)*self.s
                    self.c=self.alpha*c+(1-self.alpha)*self.c
                angle=math.atan2(self.s,self.c)
                q=np.array([-math.sin(angle/2),0.,0.,math.cos(angle/2)])
                self.last=(q,t,angle)
                if not blocked: self.queue.append(self.last)
        while self.queue and t-self.queue[0][1]>=self.delay:
            self.output=self.queue.popleft()
        return self.output, bool(blocked)
