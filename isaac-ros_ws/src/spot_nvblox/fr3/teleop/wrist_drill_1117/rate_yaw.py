"""Neutral-centered wrist pronation controls bounded table-facing yaw velocity."""
import math
import numpy as np
from scipy.spatial.transform import Rotation as R
from table_facing import TableFacing
class RateYaw:
    def __init__(self, hand, robot, rate_deg_s=45.):
        self.neutral=R.from_quat(hand)
        self.down=R.from_quat([1.,0.,0.,0.])
        self.project=TableFacing(robot,rate_deg_s)
        self.yaw=self.project.yaw
        self.rate=0.
        self.max_rate=math.radians(rate_deg_s)
    def pause(self):
        self.rate=0.
    def step(self, hand, commanded, dt):
        if not math.isfinite(dt) or not 0<dt<=.1:raise ValueError('invalid timestep')
        q=(self.neutral.inv()*R.from_quat(hand)).as_quat()
        a=0. if math.hypot(q[0],q[3])<1e-7 else math.atan2(math.sin(2*math.atan2(q[0],q[3])),math.cos(2*math.atan2(q[0],q[3])))
        desired=math.copysign(self.max_rate*min(1.,max(0.,(abs(a)-math.radians(5))/math.radians(25))),a)
        current=self.project._twist_yaw(commanded,self.yaw)
        if desired==0.:
            self.rate=0.;self.yaw=current
        else:
            self.rate+=float(np.clip(desired-self.rate,-math.radians(90)*dt,math.radians(90)*dt))
            self.yaw=float(np.clip(self.yaw+self.rate*dt,current-math.radians(5),current+math.radians(5)))
        return (R.from_rotvec([0.,0.,self.yaw])*self.down).as_quat()
