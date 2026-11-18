"""Prefer FR3 joint7 for table-facing yaw; a soft MPC reference, never a joint command."""
import math
import numpy as np
def wrist_yaw_reference(joint_position, measured_wxyz, target_wxyz, lower, upper):
    a=np.asarray(measured_wxyz,dtype=float);b=np.asarray(target_wxyz,dtype=float)
    if a.shape!=(4,) or b.shape!=(4,) or not np.isfinite(np.r_[joint_position,a,b,lower,upper]).all():raise ValueError('finite wrist state required')
    if min(np.linalg.norm(a),np.linalg.norm(b))<1e-8 or upper-lower<=2*math.radians(5):raise ValueError("valid quaternion and joint interval required")
    a=a/np.linalg.norm(a);b=b/np.linalg.norm(b)
    # Joint7's world axis is hand+Z; only prefer it when facing downward.
    approach_z=1.-2.*(a[1]**2+a[2]**2)
    if approach_z>-.9:return float(joint_position)
    aw,ax,ay,az=a;bw,bx,by,bz=b
    rw=bw*aw+bx*ax+by*ay+bz*az
    rz=-bw*az-bx*ay+by*ax+bz*aw
    if math.hypot(rw,rz)<1e-7:return float(joint_position)
    error=2.*math.atan2(rz,rw)
    error=math.atan2(math.sin(error),math.cos(error))
    error=float(np.clip(error,-math.radians(30),math.radians(30)))
    margin=math.radians(5)
    return float(np.clip(joint_position+error/approach_z,lower+margin,upper-margin))
