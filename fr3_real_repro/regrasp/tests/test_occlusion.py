"""occlusion.py: synthetic look-at cameras vs the real FK silhouette.

Camera 1 sits on the far side of an arm link from the drill box (must read
occluded); camera 2 sits on the far side of the box from the arm (arm behind
the box: clear); camera 3 looks from the side (clear). No live data needed.

Run: docker exec vm-viewer bash -c
  'source /opt/ros/humble/setup.bash && python3 <this file>'
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, '/home/nexus/spot-teleop/fr3_real_repro/grasp_loop')

from occlusion import CamModel, arm_points, occludes
from grasp_plan_lib import Q_HOME, fk


def check(cond, msg):
    if not cond:
        raise AssertionError(msg)
    print(f"ok: {msg}")


def look_at(cam_pos, target):
    z = np.asarray(target, float) - np.asarray(cam_pos, float)
    z /= np.linalg.norm(z)
    x = np.cross([0, 0, 1], z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return CamModel(476.0, 476.0, 320.0, 180.0, 640, 360,
                    np.column_stack([x, y, z]), np.asarray(cam_pos, float))


q = np.array(Q_HOME, float)
P = arm_points(q, fk)
check(P.shape[1] == 3 and len(P) > 40, f"silhouette sampled ({len(P)} pts)")
elbow = P[len(P) // 3]
box_c = np.array([0.45, -0.20, 0.15])
lo, hi = box_c - 0.06, box_c + 0.06

d = elbow - box_c
d /= np.linalg.norm(d)
cam1 = look_at(elbow + d * 0.6, box_c)      # ray: cam -> elbow -> box
check(occludes(q, cam1, lo, hi, fk), "arm between cam and box reads occluded")
cam2 = look_at(box_c - d * 0.6, box_c)      # ray: cam -> box -> elbow
check(not occludes(q, cam2, lo, hi, fk), "arm behind box reads clear")
side = np.cross(d, [0, 0, 1])
side /= np.linalg.norm(side)
cam3 = look_at(box_c + side * 0.8 + [0, 0, 0.3], box_c)
check(not occludes(q, cam3, lo, hi, fk), "side view reads clear")
far = (np.array([2.0, 2.0, 2.0]), np.array([2.1, 2.1, 2.1]))
check(not occludes(q, cam1, *far, fk), "box out of view reads clear")
print("5/5 occlusion checks passed")
