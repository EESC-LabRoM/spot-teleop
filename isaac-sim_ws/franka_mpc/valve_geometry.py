"""HiveBoard ball-valve geometry in the Franka base frame (fr3_link0, table top z = 0), shared by valve_clouds.py and newton_valve.py.
Placement = isaaclab-hiveboard FR3 ball-valve task (valve root 0.70 m ahead, 0.06 left, 0.554 above the base, yaw 180 deg) but 0.60 m ahead (our table is 0.8 m long),
see hiveboard_scene.py. Valve frame: lever axis = +x (towards the robot), lever at theta = 0 points along +y (world -y), theta in [-90, 0] deg swings it down.
Meshes (assets/hiveboard/Valves/Lever Valve/Ball Valve/Ball_Valve.urdf): lever link 'alavanca_pivot' and body 'valvula_esfera' at the joint origin (identity)."""
import os
import numpy as np
from scipy.spatial.transform import Rotation as R

HERE = os.path.dirname(os.path.abspath(__file__))
HB = os.path.join(HERE, "assets", "hiveboard")
VALVE_DIR = os.path.join(HB, "Valves", "Lever Valve", "Ball Valve")
FWD, LEFT, UP = 0.60, 0.06, 0.554
POS = np.array([FWD, LEFT, UP]); YAW = 180.0
RV = R.from_euler("z", YAW, degrees=True).as_matrix()                      # valve frame -> base frame rotation
LEVER_VIS = ((-0.0468898, 0.0175836, -0.0114654), (1.5707964, 0.0, 1.5707964))   # lever mesh in its link frame (URDF visual/collision origin)
BODY_VIS = ((-0.0400277, 0.0, 0.0), (1.5707964, 0.0, 1.5707964))
PANEL_IN_VALVE = np.array([-0.04, 0.0, 0.0])                 # honeycomb panel origin in the valve frame (mm mesh, scale 0.001)

def to_base(P_valve):                                         # points in the valve frame -> base frame
    return np.asarray(P_valve) @ RV.T + POS

def link_mesh(fname, vis):
    import trimesh
    m = trimesh.load(os.path.join(VALVE_DIR, "meshes", fname), force="mesh", process=False)
    V = m.vertices @ R.from_euler("xyz", vis[1]).as_matrix().T + np.array(vis[0]); return V, np.asarray(m.faces)

def panel_mesh():                                             # honeycomb panel faces/vertices in the valve frame (7 hexagon bars, URDF visuals, mm -> m)
    import re, trimesh
    txt = open(os.path.join(HB, "Honeycomb", "Honeycomb_Panel.urdf")).read(); Vs, Fs, n = [], [], 0
    for xyz, rpy, fn in re.findall(r'<visual>\s*<origin xyz="([^"]+)" rpy="([^"]+)" />.*?filename="([^"]+)"', txt, re.S):
        m = trimesh.load(os.path.join(HB, "Honeycomb", fn), force="mesh", process=False)
        V = (m.vertices * 0.001) @ R.from_euler("xyz", [float(x) for x in rpy.split()]).as_matrix().T + np.array([float(x) for x in xyz.split()])
        Vs.append(V + PANEL_IN_VALVE); Fs.append(np.asarray(m.faces) + n); n += len(V)
    return np.vstack(Vs), np.vstack(Fs)

def hinge():                                                  # lever hinge in the base frame: point (valve origin) and axis (valve +x)
    return POS.copy(), RV[:, 0].copy()

def rot_about_hinge(theta):                                   # rigid map of lever-attached points when the lever is at theta (rad): p' = c + Rm (p - c)
    c, ax = hinge(); return c, R.from_rotvec(ax * theta).as_matrix()
