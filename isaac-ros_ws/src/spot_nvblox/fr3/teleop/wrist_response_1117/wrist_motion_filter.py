"""Cartesian acceleration bound; stale input and following guards still hold immediately."""
import math
import numpy as np
def bounded_step(previous_velocity, desired_step, dt, acceleration):
    if not math.isfinite(dt) or dt<=0 or not math.isfinite(acceleration) or acceleration<=0:
        raise ValueError('positive finite timestep and acceleration required')
    desired=np.asarray(desired_step,dtype=float)/dt
    previous=np.asarray(previous_velocity,dtype=float)
    if not np.all(np.isfinite(np.r_[previous,desired])):raise ValueError('finite motion required')
    dv=desired-previous;n=float(np.linalg.norm(dv));limit=acceleration*dt
    if n>limit:dv*=limit/n
    velocity=previous+dv
    return velocity*dt,velocity


def map_translation(delta, profile):
    delta=np.asarray(delta,dtype=float)
    if delta.shape!=(3,) or not np.isfinite(delta).all():raise ValueError('finite 3D displacement required')
    if profile=='legacy':return delta.copy()
    if profile=='height_to_reach':return np.array([-delta[2],delta[1],-delta[0]])
    raise ValueError('Unknown translation axes')
