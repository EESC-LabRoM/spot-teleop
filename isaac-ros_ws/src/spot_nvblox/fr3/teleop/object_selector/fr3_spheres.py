"""Offline FR3 collision-sphere kinematics (numpy only, no ROS/robot access).

Reads the same URDF and sphere file the live cuRobo node resolves
(hub-curobo node_config/fr3.yml -> robot/fr3_description/fr3.urdf,
spheres/fr3_mesh.yml, collision_sphere_buffer 0.004) and returns world
(fr3_link0) spheres for any joint vector. Optional held-object spheres are
attached in fr3_hand, as the live attached_spheres/<run>.json files are.
"""
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml

_HERE = Path(__file__).resolve().parent
URDF = _HERE / 'robot/fr3.urdf'            # copies of hub-curobo content (fr3.yml)
SPHERES = _HERE / 'robot/fr3_mesh.yml'
# node_config/fr3.yml collision_link_names (link0 removed in the live node)
COLLISION_LINKS = ('fr3_link1', 'fr3_link2', 'fr3_link3', 'fr3_link4', 'fr3_link5',
                   'fr3_link6', 'fr3_link7', 'fr3_hand', 'fr3_leftfinger', 'fr3_rightfinger')
ARM_JOINTS = tuple(f'fr3_joint{i}' for i in range(1, 8))
SPHERE_BUFFER = 0.004
FINGER_OPEN = 0.04      # lock_joints in node_config/fr3.yml


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _axis_rot(a, th):
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K


class FR3Spheres:
    def __init__(self, urdf=URDF, spheres=SPHERES, buffer=SPHERE_BUFFER, links=COLLISION_LINKS):
        root = ET.parse(urdf).getroot()
        self.joints = {}
        for j in root.findall('joint'):
            o = j.find('origin')
            xyz = np.array([float(v) for v in (o.get('xyz', '0 0 0') if o is not None else '0 0 0').split()])
            rpy = [float(v) for v in (o.get('rpy', '0 0 0') if o is not None else '0 0 0').split()]
            ax = j.find('axis')
            T = np.eye(4)
            T[:3, :3], T[:3, 3] = _rpy(*rpy), xyz
            self.joints[j.find('child').get('link')] = dict(
                name=j.get('name'), type=j.get('type'), parent=j.find('parent').get('link'), T=T,
                axis=np.array([float(v) for v in ax.get('xyz').split()]) if ax is not None else None)
        data = yaml.safe_load(Path(spheres).read_text())['collision_spheres']
        self.link_spheres = {}
        for link in links:
            rows = [[*s['center'], s['radius'] + buffer] for s in data[link] if s['radius'] > 0]
            self.link_spheres[link] = np.array(rows, float)
        self.links = links

    def link_poses(self, q, finger=FINGER_OPEN):
        q = np.asarray(q, float).reshape(-1)
        if q.shape != (7,) or not np.isfinite(q).all():
            raise ValueError('q must be 7 finite joints')
        val = dict(zip(ARM_JOINTS, q))
        val['fr3_finger_joint1'] = val['fr3_finger_joint2'] = finger
        cache = {'fr3_link0': np.eye(4)}

        def pose(link):
            if link in cache:
                return cache[link]
            j = self.joints[link]
            T = pose(j['parent']) @ j['T']
            if j['type'] in ('revolute', 'continuous'):
                M = np.eye(4); M[:3, :3] = _axis_rot(j['axis'], val[j['name']]); T = T @ M
            elif j['type'] == 'prismatic':
                M = np.eye(4); M[:3, 3] = j['axis'] * val[j['name']]; T = T @ M
            cache[link] = T
            return T
        for link in self.links + ('fr3_hand_tcp',):
            pose(link)
        return cache

    def spheres(self, q, held=None, finger=FINGER_OPEN):
        """World spheres (N x 4) and a parallel list of link labels.
        held: optional N x 4 sphere rows in fr3_hand (x, y, z, r)."""
        P = self.link_poses(q, finger)
        out, labels = [], []
        for link in self.links:
            S = self.link_spheres[link]
            T = P[link]
            out.append(np.c_[S[:, :3] @ T[:3, :3].T + T[:3, 3], S[:, 3]])
            labels += [link] * len(S)
        if held is not None and len(held):
            H = np.asarray(held, float)
            T = P['fr3_hand']
            out.append(np.c_[H[:, :3] @ T[:3, :3].T + T[:3, 3], H[:, 3]])
            labels += ['held'] * len(H)
        return np.vstack(out), labels
