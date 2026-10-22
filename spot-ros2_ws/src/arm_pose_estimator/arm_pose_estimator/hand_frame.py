"""RGB-D palm frame geometry. Axes are explicit; no Euler angles enter control.

Hand +x: wrist toward MCP centres; +y: little-finger MCP toward index MCP;
+z: x cross y. FR3 TCP +z points along fingers, +y spans the gripper jaws.
All point coordinates must be in one optical camera frame, in metres.
"""
import math
import numpy as np

HAND_TO_FR3_TCP = np.array([[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]])


def fingers_open(points):
    """Four extended fingers in metric 3D, independent of palm facing or thumb pose.

    This supplements the canned Open_Palm classifier; absence of a fist alone
    never means open. Unknown, curled or degenerate geometry returns False.
    """
    p = np.asarray(points, dtype=float)
    if p.shape != (21, 3) or not np.all(np.isfinite(p)): return False
    for mcp in (5, 9, 13, 17):
        segments = np.diff(p[mcp:mcp+4], axis=0)
        lengths = np.linalg.norm(segments, axis=1)
        if np.any(lengths < .002) or np.any(lengths > .10): return False
        direction = segments / lengths[:, None]
        if np.dot(direction[0], direction[1]) < .75 or np.dot(direction[1], direction[2]) < .75:
            return False
        if np.linalg.norm(p[mcp+3]-p[0]) < 1.15*np.linalg.norm(p[mcp+1]-p[0]):
            return False
    return True


def proper_rotation(matrix):
    a = np.asarray(matrix, dtype=float)
    if a.shape != (3, 3) or not np.all(np.isfinite(a)) or np.linalg.det(a) < .1:
        raise ValueError('invalid body rotation')
    u, s, vt = np.linalg.svd(a)
    if s[-1] < .1: raise ValueError('degenerate body rotation')
    return u @ vt


def palm_rotation(points):
    """points: wrist, index MCP, middle MCP, ring MCP, little MCP (0,5,9,13,17)."""
    p = np.asarray(points, dtype=float)
    if p.shape != (5, 3) or not np.all(np.isfinite(p)): raise ValueError('invalid palm points')
    if np.any(p[:, 2] < .15) or np.any(p[:, 2] > 6.): raise ValueError('palm depth out of range')
    lengths = np.linalg.norm(p[1:] - p[0], axis=1)
    width = np.linalg.norm(p[1] - p[4])
    if np.any(lengths < .025) or np.any(lengths > .18) or not .025 < width < .16:
        raise ValueError('palm depth/size inconsistent')
    x = np.mean(p[1:], axis=0) - p[0]
    x /= np.linalg.norm(x)
    y = p[1] - p[4]
    side = y - np.dot(y, x) * x
    if np.linalg.norm(side) / width < .35: raise ValueError('palm axes collinear')
    y = side / np.linalg.norm(side)
    z = np.cross(x, y)
    rotation = np.column_stack((x, y, z))
    # A palm should be approximately planar. Reject background depth on one MCP.
    residual = np.abs((p - np.mean(p, axis=0)) @ z)
    if np.max(residual) > .025: raise ValueError('inconsistent palm plane')
    return rotation


def matrix_quaternion(matrix):
    """Stable conversion including half turns; returns ROS xyzw."""
    r = proper_rotation(matrix)
    q = np.empty(4)
    tr = np.trace(r)
    if tr > 0:
        s = math.sqrt(1. + tr) * 2
        q[:] = [(r[2,1]-r[1,2])/s, (r[0,2]-r[2,0])/s, (r[1,0]-r[0,1])/s, s/4]
    else:
        i = int(np.argmax(np.diag(r))); j=(i+1)%3; k=(i+2)%3
        s = math.sqrt(max(0., 1.+r[i,i]-r[j,j]-r[k,k]))*2
        q[i]=s/4; q[j]=(r[j,i]+r[i,j])/s; q[k]=(r[k,i]+r[i,k])/s; q[3]=(r[k,j]-r[j,k])/s
    return q / np.linalg.norm(q)


def quaternion_matrix(q):
    q=np.asarray(q,dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)) or np.linalg.norm(q)<1e-8: raise ValueError('invalid quaternion')
    x,y,z,w=q/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def tool_orientation(points, body_in_camera):
    return matrix_quaternion(proper_rotation(body_in_camera).T @ palm_rotation(points) @ HAND_TO_FR3_TCP)


def hand_roi(landmarks, width, height):
    """Square crop around the operator's right hand, expressed in original pixels."""
    wrist = np.array([landmarks[16].x*width, landmarks[16].y*height])
    index = np.array([landmarks[20].x*width, landmarks[20].y*height])
    elbow = np.array([landmarks[14].x*width, landmarks[14].y*height])
    if not np.all(np.isfinite(np.r_[wrist,index,elbow])): raise ValueError('invalid hand ROI')
    finger_delta = index-wrist
    # Pose fingertips are approximate; prevent one bad landmark from moving the crop away.
    if np.linalg.norm(finger_delta) > .25*min(width,height): finger_delta *= 0.
    center = wrist + .5*finger_delta
    side = int(np.clip(max(128., 3*np.linalg.norm(finger_delta), np.linalg.norm(wrist-elbow)),
                       1., min(320,width,height)))
    x = int(np.clip(center[0]-side/2, 0, width-side))
    y = int(np.clip(center[1]-side/2, 0, height-side))
    return x,y,side,side


def robust_depth(depth, u, v, radius=2, limits=(.1,10.)):
    """Reject out-of-range/background pixels and patches with no supported depth cluster."""
    if not (0 <= u < depth.shape[1] and 0 <= v < depth.shape[0]): return None
    patch = depth[max(0,v-radius):v+radius+1,max(0,u-radius):u+radius+1]
    values = np.sort(patch[np.isfinite(patch) & (patch > limits[0]) & (patch < limits[1])])
    required = max(3, int(math.ceil(.15*patch.size)))
    if len(values) < required: return None
    # At a silhouette, a median spanning foreground and background is not a measurement.
    # Prefer a supported foreground cluster rather than averaging across a depth edge.
    groups = np.split(values, np.flatnonzero(np.diff(values) > .08)+1)
    supported = [g for g in groups if len(g) >= required]
    if not supported: return None
    group = supported[0]
    if np.percentile(group,90)-np.percentile(group,10) > .12: return None
    return float(np.median(group))


class GestureOrientationLock:
    """Spot's grasp orientation lock extended to SO(3), with a jump-free release."""
    def __init__(self, duration=2.):
        self.duration, self.until = duration, -1.
        self.gesture, self.held, self.last = None, None, None
        self.offset = np.eye(3)

    def update(self, raw, gesture, stamp, anchor=None):
        rotation = quaternion_matrix(raw)
        if ((gesture == 'Closed_Fist' and self.gesture != 'Closed_Fist') or
                (gesture == 'Open_Palm' and self.gesture == 'Closed_Fist')):
            self.held = np.asarray(anchor if anchor is not None else
                                   (self.last if self.last is not None else raw)).copy()
            self.until = stamp+(self.duration if gesture == 'Closed_Fist' else .35)
        if gesture in ('Closed_Fist','Open_Palm'): self.gesture = gesture
        if self.held is not None:
            if stamp < self.until:
                self.last = self.held.copy()
                return self.last.copy()
            # Rebase the changed fist geometry rather than steering toward a new neutral.
            # Right multiplication preserves subsequent spatial hand rotations in base axes.
            self.offset = rotation.T @ quaternion_matrix(self.held)
            self.held = None
        self.last = matrix_quaternion(rotation @ self.offset)
        return self.last.copy()


class OrientationFilter:
    def __init__(self, alpha=.35, max_speed=math.radians(120), timeout=.35):
        self.alpha, self.max_speed, self.timeout = alpha, max_speed, timeout
        self.q, self.stamp, self.raw_q = None, None, None

    def hold(self, stamp):
        """Intentional grasp clutch with a fresh detected hand, not a tracking fallback."""
        if self.q is None or self.stamp is None or stamp <= self.stamp:
            raise ValueError('invalid orientation hold')
        self.stamp, self.raw_q = stamp, self.q.copy()
        return self.q.copy()

    def update(self, q, stamp):
        q = np.asarray(q, dtype=float)
        if q.shape != (4,) or not np.all(np.isfinite(q)) or abs(np.linalg.norm(q)-1)>1e-4:
            raise ValueError('invalid orientation sample')
        if self.stamp is not None and stamp <= self.stamp: raise ValueError('orientation timestamp replay')
        if self.q is None or stamp-self.stamp > self.timeout:
            self.q, self.stamp, self.raw_q = q.copy(), stamp, q.copy()
            return q.copy()
        dt = stamp-self.stamp
        # Gate physical frame-to-frame motion, not distance to a lagging EMA output.
        # Comparing to the EMA falsely rejected continuous pitch/yaw at low camera rates.
        raw_angle=2*math.acos(np.clip(abs(float(np.dot(self.raw_q,q))),-1.,1.))
        if raw_angle > max(math.radians(45), 2*self.max_speed*dt): raise ValueError('hand orientation jump')
        dot = float(np.dot(self.q, q))
        if dot < 0: q, dot = -q, -dot
        angle=2*math.acos(np.clip(dot,-1.,1.))
        # Same response time at 5 Hz and 30 Hz, retaining an explicit angular speed cap.
        alpha = 1-(1-self.alpha)**(dt*30.)
        t=min(alpha, self.max_speed*dt/max(angle,1e-9))
        if dot > .9995: filtered=(1-t)*self.q+t*q
        else:
            theta=math.acos(np.clip(dot,-1.,1.))
            filtered=(math.sin((1-t)*theta)*self.q+math.sin(t*theta)*q)/math.sin(theta)
        self.q=filtered/np.linalg.norm(filtered);self.stamp=stamp;self.raw_q=q.copy()
        return self.q.copy()
