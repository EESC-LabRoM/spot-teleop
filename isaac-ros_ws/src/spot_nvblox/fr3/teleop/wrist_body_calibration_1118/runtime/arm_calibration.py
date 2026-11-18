"""Per-operator metric arm calibration; missing torso never invalidates a latched scale."""
import math
import numpy as np

class ArmCalibration:
    def __init__(self, reach=.855, minimum=20, duration=2.5):
        if not math.isfinite(reach) or reach<=0 or minimum<2 or not math.isfinite(duration) or duration<=0:raise ValueError("invalid arm calibration settings")
        self.reach,self.minimum,self.duration=reach,minimum,duration
        self.reset()

    def reset(self):
        self.samples=[];self.scale=None;self.length=None;self.last_stamp=-1.
        self.check_samples=[];self.check_length=None
        self.shoulder=None;self.wrist_reference=None;self.reference_stamp=None
        self.upper_length=self.fore_length=None;self.profile_restored=False

    def observe(self, shoulder, elbow, wrist, stamp):
        points=np.asarray([shoulder,elbow,wrist],float)
        if points.shape!=(3,3) or not np.all(np.isfinite(points)) or not math.isfinite(stamp) or stamp<=self.last_stamp:return False
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
        self.samples=(self.samples+[(stamp,length,float(upper),float(fore),points[0].copy())])[-90:]
        if len(self.samples)>=self.minimum and stamp-self.samples[0][0]>=self.duration:
            # Both bone segments must agree; equal totals can hide opposing depth errors.
            segments=np.array([[x[2],x[3]] for x in self.samples])
            groups=[np.flatnonzero((np.abs(segments-centre)<=.035).all(axis=1)) for centre in segments]
            selected=max(groups,key=len)
            if len(selected)>=self.minimum and len(selected)>=.65*len(segments):
                lengths=segments[selected].sum(axis=1)
                q1,median,q3=np.percentile(lengths,[25,50,75])
                if q3-q1<.035:
                    self.length=float(median);self.scale=self.reach/self.length
                    self.upper_length,self.fore_length=np.median(segments[selected],axis=0).tolist()
                    self.shoulder=np.median([self.samples[i][4] for i in selected],axis=0)
        return True

    @staticmethod
    def profile_lengths(profile):
        if not isinstance(profile,dict) or type(profile.get('version')) is not int or profile['version']!=1:
            raise ValueError('unsupported operator profile version')
        try:values=[float(profile[k]) for k in ('arm_length_m','arm_upper_m','arm_forearm_m')]
        except (KeyError,TypeError,ValueError,OverflowError) as error:raise ValueError('invalid operator profile length fields') from error
        length,upper,fore=values
        if not all(math.isfinite(x) for x in values) or not (.45<length<.95 and .15<upper<.5 and .15<fore<.5) or abs(upper+fore-length)>.01:
            raise ValueError('invalid operator profile segment lengths')
        return length,upper,fore

    def restore(self, profile, shoulder, stamp):
        """Explicit saved person profile, installed only with a fresh current shoulder.

        Camera axes/positions are never restored. Scale is derived again from
        the configured robot reach; a serialized gain is not trusted.
        """
        if self.scale is not None:return False
        length,upper,fore=self.profile_lengths(profile)
        point=np.asarray(shoulder,float)
        if point.shape!=(3,) or not np.isfinite(point).all() or not .3<point[2]<3. or not math.isfinite(stamp):
            raise ValueError('saved operator profile requires a current valid shoulder')
        self.length,self.upper_length,self.fore_length=length,upper,fore
        self.scale=self.reach/length;self.shoulder=point.copy();self.last_stamp=stamp
        self.profile_restored=True
        return True

    def status(self):
        return dict(arm_calibrated=self.scale is not None,arm_samples=len(self.samples),
                    arm_required_samples=self.minimum,arm_length_m=self.length,arm_scale=self.scale,
                    arm_check_length_m=self.check_length,arm_upper_m=self.upper_length,arm_forearm_m=self.fore_length,arm_profile_restored=self.profile_restored)
