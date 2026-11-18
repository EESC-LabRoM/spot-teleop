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
