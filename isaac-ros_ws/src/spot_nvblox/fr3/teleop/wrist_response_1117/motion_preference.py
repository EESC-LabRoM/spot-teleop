"""Smoothly prefer proximal stability at rest, freeing translation while moving."""
import math
import numpy as np
class MotionPreference:
    def __init__(self):
        self.previous=None;self.speed=0.;self.mix=0.
    def update(self, position, dt):
        p=np.asarray(position,dtype=float)
        if p.shape!=(3,) or not np.isfinite(p).all() or not math.isfinite(dt) or dt<=0:raise ValueError('current finite goal and positive dt required')
        dt=min(dt,.05)
        instantaneous=0. if self.previous is None else float(np.linalg.norm(p-self.previous))/dt
        self.previous=p.copy()
        self.speed+=(-math.expm1(-dt/.15))*(instantaneous-self.speed)
        desired=float(np.clip((.04-self.speed)/.025,0.,1.))
        self.mix+=(-math.expm1(-dt/.3))*(desired-self.mix)
        base=np.array([3.,3.,3.,2.,1.,1.,1.]);stable=np.array([30.,30.,30.,30.,1.,1.,.2])
        return base+self.mix*(stable-base)
def apply_preference(mpc,weights):
    for rollout in [o.rollout_fn for o in mpc.solver.optimizers]+[mpc.solver.safety_rollout,mpc.rollout_fn]:
        value=rollout.dynamics_model.null_space_weight.new_tensor(weights)
        rollout.dynamics_model.null_space_weight.copy_(value.reshape_as(rollout.dynamics_model.null_space_weight))
        if getattr(rollout,'bound_cost',None) is not None:rollout.bound_cost.update_vec_weight(value)
