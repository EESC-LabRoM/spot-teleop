"""Offline collision evidence for every pick/place stage; not connected to Y.

Input spheres must come from validated full-body kinematics at sufficiently
spaced trajectory samples. ESDF must supply signed distance at sphere centres,
with unknown cells flagged, and must already exclude only the selected target.
This module never treats unknown space as free or commands robot hardware.
"""
from dataclasses import dataclass
import math
import numpy as np

REQUIRED_STAGES=('S1_home_to_pre','S2_pre_to_grasp','S3_lift','S4_put_back','S5_grasp_to_pre','S6_pre_to_home')

@dataclass(frozen=True)
class Evidence:
    stages: dict  # stage -> iterable of N x 4 sphere rows (base x,y,z,r)
    max_joint_step: float
    selected_target: str
    map_target: str
    map_current: bool
    other_obstacles_present: bool
    carried_stages_include_object: bool


def validate(evidence, query, *, clearance_m, table_top_m, max_joint_step=.05):
    if not (math.isfinite(clearance_m) and clearance_m >= 0 and math.isfinite(table_top_m)):
        raise ValueError('Invalid limits')
    if not evidence.map_current or not evidence.selected_target or evidence.map_target != evidence.selected_target or not evidence.other_obstacles_present:
        return False, 'Map identity, freshness or non-target obstacles not verified'
    if not math.isfinite(evidence.max_joint_step) or not 0 <= evidence.max_joint_step <= max_joint_step:
        return False, 'Insufficient trajectory sampling'
    if not evidence.carried_stages_include_object:
        return False, 'Held-object geometry missing from carried stages'
    if set(evidence.stages) != set(REQUIRED_STAGES):
        return False, 'All six stages are required'
    for stage in REQUIRED_STAGES:
        samples=evidence.stages[stage]
        if not samples:
            return False, f'{stage}: no samples'
        for index, sample in enumerate(samples):
            spheres=np.asarray(sample,dtype=float)
            if spheres.ndim != 2 or spheres.shape[1] != 4 or not len(spheres) or not np.isfinite(spheres).all() or (spheres[:,3] <= 0).any():
                return False, f'{stage}/{index}: invalid spheres'
            if (spheres[:,2]-spheres[:,3] < table_top_m+clearance_m).any():
                return False, f'{stage}/{index}: table clearance'
            dist,known=query(spheres[:,:3]);dist=np.asarray(dist,dtype=float);known=np.asarray(known)
            if dist.shape != (len(spheres),) or known.shape != (len(spheres),) or known.dtype.kind != 'b' or not known.all() or not np.isfinite(dist).all():
                return False, f'{stage}/{index}: unknown ESDF'
            if (dist-spheres[:,3] < clearance_m).any():
                return False, f'{stage}/{index}: obstacle clearance'
    return True, 'All stages have sampled clearance evidence'
