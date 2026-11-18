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
    """Wrist relative to a slowly following shoulder reference.

    The reference follows the measured shoulder at up to `rate` m/s. dt is the
    time since the previous SHOULDER sample: it used to be the time since the
    previous wrist frame (~33 ms at 30 Hz while Pose runs every ~0.33 s), so the
    reference crept at <1 cm/s and the arm drifted back for tens of seconds after
    the operator leaned or stepped. Pose shoulder depth is noisy and only ~3 Hz,
    so samples are averaged (`smoothing`, EMA weight of a new sample) and the
    reference ignores the smoothed shoulder within `deadband` m of it: sway and
    measurement noise no longer leak into the command, real body motion is still
    cancelled. mode='off' publishes the plain wrist.
    """
    def __init__(self,rate=.08,mode='follow',smoothing=.3,deadband=.02):
        if mode not in ('follow','off'):raise ValueError('shoulder mode must be follow or off')
        self.shoulder=None;self.stamp=None;self.rate=float(rate);self.mode=mode
        self.smoothing=float(smoothing);self.deadband=float(deadband);self.measured=None
    def update(self,wrist,stamp,baseline,shoulder=None):
        if self.mode=='off':return np.asarray(wrist)
        baseline=np.asarray(baseline,float)
        if self.shoulder is None:self.shoulder=np.asarray(shoulder,float).copy() if shoulder is not None else baseline.copy()
        if shoulder is not None:
            shoulder=np.asarray(shoulder,float)
            if np.all(np.isfinite(shoulder)) and .3<shoulder[2]<3.:
                dt=max(0.,min(.2,stamp-self.stamp)) if self.stamp is not None else 0.
                self.measured=shoulder.copy() if self.measured is None else self.measured+self.smoothing*(shoulder-self.measured)
                delta=self.measured-self.shoulder;distance=np.linalg.norm(delta)
                # Bound changes of the reference independently of wrist movement; ignore the deadband.
                if distance>self.deadband:self.shoulder+=delta*min(1.-self.deadband/distance,self.rate*dt/distance)
                self.stamp=stamp
        return np.asarray(wrist)-self.shoulder+baseline
