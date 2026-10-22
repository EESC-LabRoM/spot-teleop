"""Per-operator metric arm calibration; missing torso never invalidates a latched scale."""
import numpy as np

class ArmCalibration:
    def __init__(self, reach=.855, minimum=20, duration=2.5):
        self.reach,self.minimum,self.duration=reach,minimum,duration
        self.reset()

    def reset(self):
        self.samples=[];self.scale=None;self.length=None;self.last_stamp=-1.
        self.check_samples=[];self.check_length=None
        self.shoulder=None;self.wrist_reference=None;self.reference_stamp=None

    def observe(self, shoulder, elbow, wrist, stamp):
        points=np.asarray([shoulder,elbow,wrist],float)
        if points.shape!=(3,3) or not np.all(np.isfinite(points)) or stamp<=self.last_stamp:return False
        upper,fore=np.linalg.norm(points[1:]-points[:-1],axis=1)
        length=float(upper+fore)
        if not (.15<upper<.50 and .15<fore<.50 and .45<length<.95):return False
        if self.shoulder is not None and np.linalg.norm(points[0]-self.shoulder)>.35:
            if self.scale is not None or stamp-self.last_stamp<1.5:return False
            # Reacquire an initial calibration after a real change of framing;
            # missing landmarks alone do not clear accepted measurements.
            self.samples=[];self.shoulder=None
        if self.shoulder is None:self.shoulder=points[0].copy()
        self.wrist_reference=points[-1].copy();self.reference_stamp=stamp
        self.last_stamp=stamp
        if self.scale is not None:
            self.check_samples=(self.check_samples+[length])[-21:]
            if len(self.check_samples)>=5:self.check_length=float(np.median(self.check_samples))
            return True
        self.samples=(self.samples+[(stamp,length)])[-90:]
        if len(self.samples)>=self.minimum and stamp-self.samples[0][0]>=self.duration:
            values=np.sort([x[1] for x in self.samples]);best=np.array([])
            for start in range(len(values)):
                group=values[start:][values[start:]<=values[start]+.06]
                if len(group)>len(best):best=group
            # Require a dominant consensus, not merely a small convenient cluster.
            if len(best)>=self.minimum and len(best)>=.65*len(values):
                q1,median,q3=np.percentile(best,[25,50,75])
                if q3-q1<.035:
                    self.length=float(median);self.scale=self.reach/self.length
        return True

    def status(self):
        return dict(arm_calibrated=self.scale is not None,arm_samples=len(self.samples),
                    arm_required_samples=self.minimum,arm_length_m=self.length,arm_scale=self.scale,
                    arm_check_length_m=self.check_length)
