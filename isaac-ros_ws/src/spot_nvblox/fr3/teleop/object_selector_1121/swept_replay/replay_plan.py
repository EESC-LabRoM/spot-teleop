"""Offline swept replay of a staged grasp plan with full-body spheres.

Samples each stage the way the direct-JTC executor moves it (joint-linear
between consecutive waypoints, from start_q or Q_HOME), and reports per stage
the worst sphere clearance against a mesh-vertex obstacle set, table
clearance, the held object on carried stages, and the TCP deviation from the
straight Cartesian line the planner intended. Read-only: no ROS, no robot.

Mesh vertices give no unknown-space information, so every result here is
`known_space_verified=False` and the swept_esdf contract stays failed.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE / 'ref'))
from fr3_spheres import FR3Spheres  # noqa: E402

Q_HOME = np.array([0.0, -0.7853981634, 0.0, -2.3561944902, 0.0, 1.5707963268, 0.7853981634])
STAGES = ('S1_home_to_pre', 'S2_pre_to_grasp', 'S3_lift', 'S4_put_back', 'S5_grasp_to_pre', 'S6_pre_to_home')
CARRIED = ('S3_lift', 'S4_put_back')          # object in hand between close and release
LINE_STAGES = ('S2_pre_to_grasp', 'S3_lift', 'S4_put_back', 'S5_grasp_to_pre')


def sample_stage(prev, qs, max_step):
    out = []
    for q in qs:
        q = np.asarray(q, float)
        n = max(2, int(np.ceil(np.abs(q - prev).max() / max_step)) + 1)
        out += [prev + (q - prev) * t for t in np.linspace(0, 1, n)]
        prev = q
    return out, prev


def replay(plan, mesh_V, target_cloud, held, *, max_step=0.01, target_margin=0.03, obstacle_floor=0.015):
    k = FR3Spheres()
    table_z = float(plan['table_z'])
    V = np.asarray(mesh_V, float)
    obst = V[V[:, 2] > table_z + obstacle_floor]
    tree = cKDTree(obst) if len(obst) else None
    cloud = np.asarray(target_cloud, float)
    box = np.array([cloud.min(0) - target_margin, cloud.max(0) + target_margin])
    in_box = np.all((obst > box[0]) & (obst < box[1]), 1) if len(obst) else np.zeros(0, bool)
    ctree = cKDTree(cloud)
    prev = np.asarray(plan.get('start_q') or Q_HOME, float)
    rep = {}
    for name in STAGES:
        samples, prev_next = sample_stage(prev, plan['stages'][name]['q'], max_step)
        hld = held if name in CARRIED else None
        worst = dict(obst=(9.0, None), table=(9.0, None), target=(9.0, None))
        tcp = []
        for q in samples:
            S, lab = k.spheres(q, hld)
            tcp.append(k.link_poses(q)['fr3_hand_tcp'][:3, 3])
            lab = np.array(lab)
            if tree is not None:
                d, i = tree.query(S[:, :3])
                c = d - S[:, 3]
                j = int(np.argmin(c))
                if c[j] < worst['obst'][0]:
                    worst['obst'] = (float(c[j]), str(lab[j]), bool(in_box[i[j]]))
            tc = S[:, 2] - S[:, 3] - table_z
            j = int(np.argmin(tc))
            if tc[j] < worst['table'][0]:
                worst['table'] = (float(tc[j]), str(lab[j]))
            # non-finger body vs target cloud (fingers and held model are meant to touch it)
            body = ~np.isin(lab, ('fr3_leftfinger', 'fr3_rightfinger', 'held'))
            d, _ = ctree.query(S[body, :3])
            c = d - S[body, 3]
            j = int(np.argmin(c))
            if c[j] < worst['target'][0]:
                worst['target'] = (float(c[j]), str(lab[body][j]))
        tcp = np.array(tcp)
        dev = None
        if name in LINE_STAGES:
            a, b = tcp[0], tcp[-1]
            u = (b - a) / max(np.linalg.norm(b - a), 1e-9)
            r = tcp - a
            dev = float(np.max(np.linalg.norm(r - np.outer(r @ u, u), axis=1)))
        rep[name] = dict(samples=len(samples), held=hld is not None,
                         min_obstacle_clear_m=round(worst['obst'][0], 4), obstacle_link=worst['obst'][1],
                         obstacle_vertex_in_target_box=worst['obst'][2] if len(worst['obst']) > 2 else None,
                         min_table_clear_m=round(worst['table'][0], 4), table_link=worst['table'][1],
                         min_body_to_target_m=round(worst['target'][0], 4), target_link=worst['target'][1],
                         tcp_line_dev_m=None if dev is None else round(dev, 4),
                         planner_min_obstacle_m=plan['checks'][name]['min_obstacle_m'],
                         planner_min_table_m=plan['checks'][name]['min_table_clear_m'])
        prev = prev_next
    return dict(run=plan.get('run'), table_z=table_z, obstacle_vertices=int(len(obst)),
                obstacle_vertices_in_target_box=int(in_box.sum()), max_joint_step=max_step,
                sphere_buffer=0.004, known_space_verified=False, stages=rep)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--plan', required=True)
    ap.add_argument('--cloud', required=True)
    ap.add_argument('--held', required=True)
    ap.add_argument('--mesh', required=True, help='Nx3 .npy of map vertices')
    ap.add_argument('--out')
    a = ap.parse_args()
    held = np.array(json.load(open(a.held))['spheres'], float)
    r = replay(json.load(open(a.plan)), np.load(a.mesh), np.load(a.cloud), held)
    s = json.dumps(r, indent=1)
    if a.out:
        Path(a.out).write_text(s)
    print(s)


if __name__ == '__main__':
    main()
