"""Latched operator axes and a continuous shoulder reference, independent of ROS.

Arm length is estimated separately from current shoulder/elbow/wrist segments.
Axes are calibrated once, so raising an arm or losing a hip cannot rotate an
already engaged hand frame. A fresh shoulder may slowly translate the reference.
"""
import math
import numpy as np
from body_wrist_reference import ShoulderRelativeWrist


def proper_rotation(m):
    u, _, vt = np.linalg.svd(m)
    d = np.eye(3); d[2, 2] = np.linalg.det(u @ vt)
    return u @ d @ vt


class OperatorReference:
    def __init__(self, minimum=8, duration=1.0, shoulder_mode='follow'):
        self.minimum, self.duration = minimum, duration
        self.samples = []
        self.rotation = self.baseline = None
        self.shoulder = None
        self.torso_baselines = {}
        self.last_observation = self.used_observation = -math.inf
        self.last_project = None
        self.relative = ShoulderRelativeWrist(mode=shoulder_mode)
        self.reason = 'waiting for body reference'

    def observe(self, robot_from_camera, shoulder, stamp, torso=None):
        r, s = np.asarray(robot_from_camera, float), np.asarray(shoulder, float)
        if not math.isfinite(stamp) or stamp <= self.last_observation:
            return False
        if r.shape != (3, 3) or s.shape != (3,) or not np.isfinite(r).all() or not np.isfinite(s).all():
            return False
        if not np.allclose(r @ r.T, np.eye(3), atol=1e-3) or np.linalg.det(r) < .99 or not .3 < s[2] < 3.:
            return False
        if self.baseline is not None and np.linalg.norm(s-self.baseline) > .35:
            return False
        # Depth along the torso is not gravity: a hip depth error must not
        # pitch the forward axis into the vertical hand movement. Keep the
        # camera-facing heading, calibrating only shoulder-line image roll.
        side=r[1].copy();side[2]=0.
        if np.linalg.norm(side)<.5 or side[0]<.5:return False
        side/=np.linalg.norm(side);forward=np.array([0.,0.,-1.])
        r=np.stack([forward,side,np.cross(forward,side)])
        points={}
        for key,value in (torso or {}).items():
            q=np.asarray(value,float)
            if key in ('shoulder_l','shoulder_r','hip_l','hip_r') and q.shape==(3,) and np.isfinite(q).all() and .3<q[2]<3.:points[key]=q.copy()
        target=s.copy()
        if self.rotation is not None and self.torso_baselines:
            deltas=np.array([q-self.torso_baselines[k] for k,q in points.items() if k in self.torso_baselines])
            if len(deltas)<2:return False
            centre=np.median(deltas,axis=0);keep=np.linalg.norm(deltas-centre,axis=1)<.035
            if keep.sum()<2 or keep.mean()<.6:return False
            delta=np.median(deltas[keep],axis=0)
            if np.linalg.norm(delta)>.35:return False
            target=self.baseline+delta
        self.last_observation = float(stamp)
        self.shoulder = target
        if self.rotation is None:
            self.samples = (self.samples+[(stamp, r.copy(), s.copy(), points)])[-60:]
        return True

    def maybe_latch(self, scale_ready):
        if self.rotation is not None:
            return True
        if not scale_ready or len(self.samples) < self.minimum:
            return False
        if self.samples[-1][0]-self.samples[0][0] < self.duration:
            return False
        # Rotation matrices are averaged then projected back onto SO(3).
        mean = proper_rotation(np.mean([x[1] for x in self.samples], axis=0))
        angles = np.array([math.acos(np.clip((np.trace(r @ mean.T)-1)/2, -1, 1)) for _, r, *_ in self.samples])
        keep = angles < math.radians(10)
        if keep.sum() < self.minimum or keep.mean() < .7:
            self.reason = 'body axes unstable; calibration not latched'
            return False
        selected = [x for x, ok in zip(self.samples, keep) if ok]
        self.rotation = proper_rotation(np.mean([x[1] for x in selected], axis=0))
        self.baseline = np.median([x[2] for x in selected], axis=0)
        self.relative.shoulder = self.baseline.copy()
        self.shoulder=self.baseline.copy()
        for key in ('shoulder_l','shoulder_r','hip_l','hip_r'):
            points=[x[3][key] for x in selected if key in x[3]]
            if len(points)>=3:self.torso_baselines[key]=np.median(points,axis=0)
        self.reason = 'body axes latched'
        return True

    def project(self, camera_wrist, stamp, gain):
        if self.rotation is None:
            raise ValueError(self.reason)
        if not math.isfinite(stamp) or (self.last_project is not None and stamp <= self.last_project):
            raise ValueError('nonmonotonic body reference timestamp')
        wrist = np.asarray(camera_wrist, float)
        if wrist.shape != (3,) or not np.isfinite(wrist).all() or not math.isfinite(gain) or gain <= 0:
            raise ValueError('invalid body-reference input')
        fresh = self.last_observation > self.used_observation and 0 <= stamp-self.last_observation <= .5
        shoulder = self.shoulder if fresh else None
        if fresh:
            self.used_observation = self.last_observation
        control = self.relative.update(wrist, stamp, self.baseline, shoulder)
        self.last_project = float(stamp)
        # Subtract the reference before rotation: torso baseline has no lever arm.
        return gain * (self.rotation @ (control-self.baseline))

    def status(self, stamp):
        return dict(body_reference_ready=self.rotation is not None,
                    body_reference_samples=len(self.samples),
                    body_reference_age_s=None if not math.isfinite(self.last_observation) else max(0.,stamp-self.last_observation),
                    body_axes_latched=self.rotation is not None,
                    body_axes=None if self.rotation is None else self.rotation.tolist(),
                    body_axes_convention='camera_forward_shoulders_roll',reference_mode='body_relative_latched_axes')
