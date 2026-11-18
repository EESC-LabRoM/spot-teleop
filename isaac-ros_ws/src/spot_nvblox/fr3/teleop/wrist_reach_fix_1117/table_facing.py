"""Table-facing orientation projection. xyzw quaternions, world/base +Z up."""
import math
import numpy as np
from scipy.spatial.transform import Rotation

class TableFacing:
    def __init__(self, initial_quaternion, yaw_rate_deg_s=45.):
        self.down=Rotation.from_quat([1.,0.,0.,0.])
        self.rate=math.radians(yaw_rate_deg_s)
        self.yaw=self._twist_yaw(initial_quaternion,0.)
        self.last_quaternion=Rotation.from_quat(initial_quaternion).as_quat()

    def _twist_yaw(self, quaternion, fallback):
        # Swing-twist decomposition avoids Euler pitch singularities.
        q=(Rotation.from_quat(quaternion)*self.down.inv()).as_quat()
        if math.hypot(q[2],q[3])<1e-7:return fallback
        raw=2*math.atan2(q[2],q[3])
        return fallback+math.atan2(math.sin(raw-fallback),math.cos(raw-fallback))

    def step(self, operator_quaternion, dt):
        if not math.isfinite(dt) or dt<=0:raise ValueError('positive dt required')
        desired=self._twist_yaw(operator_quaternion,self.yaw)
        self.yaw+=float(np.clip(desired-self.yaw,-self.rate*dt,self.rate*dt))
        goal=Rotation.from_rotvec([0.,0.,self.yaw])*self.down
        previous=Rotation.from_quat(self.last_quaternion)
        step=(goal*previous.inv()).as_rotvec();angle=np.linalg.norm(step)
        if angle>self.rate*dt:step*=self.rate*dt/angle
        q=(Rotation.from_rotvec(step)*previous).as_quat()
        if np.dot(q,self.last_quaternion)<0:q=-q
        self.last_quaternion=q.copy()
        return q

def table_tilt_deg(quaternion):
    approach=Rotation.from_quat(quaternion).apply([0.,0.,1.])
    return float(np.degrees(np.arccos(np.clip(np.dot(approach,[0.,0.,-1.]),-1.,1.))))
