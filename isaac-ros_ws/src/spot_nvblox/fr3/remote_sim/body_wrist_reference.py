"""Current RGB-D body wrist support and a continuous shoulder-relative control frame."""
import numpy as np


def body_wrist_supported(visibility, wrist, shoulder=None, elbow_visibility=0., shoulder_visibility=0., arm_length=None, hand_support=0):
    if wrist is None or not np.all(np.isfinite(wrist)):return False
    if visibility>=.65:return True
    # A weak wrist needs independent, current body support; visibility alone is insufficient.
    if visibility<.35 or shoulder is None or shoulder_visibility<.7 or (elbow_visibility<.5 and hand_support<1):return False
    reach=np.linalg.norm(np.asarray(wrist)-shoulder)
    return arm_length is not None and .10<reach<1.15*arm_length


class ShoulderRelativeWrist:
    def __init__(self):self.shoulder=None;self.stamp=None
    def update(self,wrist,stamp,baseline,shoulder=None):
        baseline=np.asarray(baseline,float)
        if self.shoulder is None:self.shoulder=np.asarray(shoulder,float).copy() if shoulder is not None else baseline.copy()
        if shoulder is not None:
            shoulder=np.asarray(shoulder,float)
            if np.all(np.isfinite(shoulder)) and .3<shoulder[2]<3.:
                dt=max(0.,min(.2,stamp-self.stamp)) if self.stamp is not None else 0.
                delta=shoulder-self.shoulder;distance=np.linalg.norm(delta)
                # Bound changes of the reference independently of wrist movement.
                if distance>0:self.shoulder+=delta*min(1.,.08*dt/distance)
        self.stamp=stamp
        return np.asarray(wrist)-self.shoulder+baseline
