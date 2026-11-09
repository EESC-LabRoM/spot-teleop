"""Render the chosen grasp against its point cloud (matplotlib, headless).

Pure function for the viewer server: render_grasp(run_dir) -> (png_bytes, meta).
Prefers the re-anchored cloud/pick (what the live run actually used), falls
back to the S3/S4 originals. Raises FileNotFoundError / KeyError with a plain
message when the run lacks artifacts.
"""
import json
import os

import numpy as np


def _load(run_dir):
    ra_json = os.path.join(run_dir, 'reanchor.json')
    if os.path.exists(ra_json):
        ra = json.load(open(ra_json))
        pick = np.array(ra['pick_shifted_m'], float)
        cloud_p = os.path.join(run_dir, 'reanchor', 'cloud.npy')
        if not os.path.exists(cloud_p):
            cloud_p = os.path.join(run_dir, 'cloud.npy')
        src = 'reanchor'
    else:
        pk = json.load(open(os.path.join(run_dir, 'pick.json')))
        pick = np.array(pk['pos'], float)
        cloud_p = os.path.join(run_dir, 'cloud.npy')
        src = 's3/s4'
    cloud = np.load(cloud_p)
    quat = np.array(json.load(open(os.path.join(run_dir, 'pick.json')))['quat_xyzw'],
                    float)
    try:
        cent = np.array(json.load(open(os.path.join(run_dir, 'cloud.json')))['centroid'],
                        float)
    except FileNotFoundError:
        cent = cloud[:, :3].mean(axis=0)
    return cloud[:, :3].astype(float), pick, quat, cent, src


def _rmat(q):
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


# GraspGenX emits a canonical grasp frame (closing axis +X); the Panda hand
# closes along +Y (URDF axis "0 1 0"). Right-multiply by Rz(+90) to pose the
# Panda mesh (end2end/robots/franka_panda.yaml grasp_to_tool_transform,
# zero translation). Without this the fingers render/execute 90 deg off.
_RZ90 = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])


_ASSETS = ('/home/nexus/spot-teleop/isaac-sim_ws/curobo-isaac6/src/curobo/'
           'content/assets/robot/fr3_description')
_meshes = {}  # kind -> (N,3,3) triangles in its own link frame


def _load_meshes():
    """FR3 hand + finger triangles (lazy; needs trimesh + pycollada)."""
    if _meshes:
        return _meshes
    import trimesh
    base = _ASSETS + '/meshes/robot_ee/franka_hand_white'
    hand = trimesh.load(base + '/collision/hand.stl')
    fing = trimesh.load(base + '/visual/finger.dae')
    if isinstance(fing, trimesh.Scene):
        fing = fing.to_mesh()
    _meshes['hand'] = np.asarray(hand.triangles, float)
    _meshes['finger'] = np.asarray(fing.triangles, float)
    return _meshes


def _gripper_tris(hand_pos, hand_rot, width=0.08):
    """Hand + both fingers posed at the grasp, width in metres (open state).

    Finger kinematics from fr3.urdf: prismatic joints at (0,0,0.0584) in the
    hand frame, left along +Y, right rotated pi about Z (so -Y); q = width/2.
    """
    m = _load_meshes()
    q = width / 2.0
    rz = np.array([[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]])
    parts = []
    hand_T = m['hand'] @ hand_rot.T + hand_pos
    parts.append(('hand', hand_T))
    fl = m['finger'] + np.array([0.0, q, 0.0584])
    parts.append(('finger', fl @ hand_rot.T + hand_pos))
    fr = (m['finger'] + np.array([0.0, q, 0.0])) @ rz.T + np.array([0.0, 0.0, 0.0584])
    parts.append(('finger', fr @ hand_rot.T + hand_pos))
    return parts


def load_grasp(run_dir, max_points=4000):
    """Grasp dataset for viewers (dict of plain lists; JSON-safe).

    Points downsampled by stride to max_points; gripper posed open 80 mm.
    """
    cloud, pick, quat, cent, src = _load(run_dir)
    tcp = pick - np.array([0.0, 0.0, 0.103])  # fr3_hand -> fr3_hand_tcp
    rg = _rmat(quat)
    rg_tool = rg @ _RZ90  # canonical grasp frame -> panda_hand frame
    if len(cloud) > max_points:
        cloud = cloud[::int(len(cloud) / max_points + 1)]
    dmin = float(np.linalg.norm(cloud - tcp, axis=1).min())
    gripper = []
    try:
        for kind, tris in _gripper_tris(pick, rg_tool, width=0.08):
            gripper.append(dict(kind=kind,
                                tris=np.round(tris.reshape(-1, 3), 4).tolist()))
    except Exception as e:  # trimesh missing: viewer draws axes only
        gripper = []
    return dict(
        run=os.path.basename(run_dir.rstrip('/')), source=src,
        n_points=int(len(cloud)),
        cloud=np.round(cloud, 4).tolist(),
        centroid=[round(float(v), 4) for v in cent],
        pick=[round(float(v), 4) for v in pick],
        quat=[round(float(v), 4) for v in (quat / np.linalg.norm(quat))],
        R=np.round(rg, 4).tolist(),
        tcp=[round(float(v), 4) for v in tcp],
        tcp_to_cloud_mm=round(dmin * 1000, 1),
        gripper=gripper,
    )


def render_grasp(run_dir):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    cloud, pick, quat, cent, src = _load(run_dir)
    tcp = pick - np.array([0.0, 0.0, 0.103])  # fr3_hand -> fr3_hand_tcp
    rg = _rmat(quat)
    rg_tool = rg @ _RZ90  # canonical grasp frame -> panda_hand frame
    dmin = float(np.linalg.norm(cloud - tcp, axis=1).min())

    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    try:
        gripper = _gripper_tris(pick, rg_tool, width=0.08)
    except Exception as e:  # trimesh/pycollada missing: axes only, note it
        gripper = []
        gripper_note = f'gripper mesh unavailable ({e})'
    else:
        gripper_note = 'gripper open 80mm (descend state)'
    fig = plt.figure(figsize=(11, 5))
    for col, (elev, azim, title) in ((121, (18, -65, '3/4 view (m)')),
                                     (122, (4, -90, 'side view: X-Z plane (m)'))):
        ax = fig.add_subplot(col, projection='3d')
        ax.scatter(cloud[:, 0], cloud[:, 1], cloud[:, 2], c=cloud[:, 2],
                   s=3, cmap='viridis', alpha=0.7)
        for kind, tris in gripper:
            poly = Poly3DCollection(
                tris, facecolor='#cfd4d9' if kind == 'hand' else '#e8b86d',
                edgecolor='#5a626b', linewidths=0.15, alpha=0.92)
            ax.add_collection3d(poly)
        ax.scatter([cent[0]], [cent[1]], [cent[2]], c='white', s=60,
                   edgecolors='k', label='grasp centroid')
        ax.scatter([pick[0]], [pick[1]], [pick[2]], c='magenta', s=120,
                   marker='X', label='pick (hand)')
        ax.scatter([tcp[0]], [tcp[1]], [tcp[2]], c='red', s=120, marker='o',
                   label='TCP')
        for i, c in ((0, 'r'), (1, 'g'), (2, 'b')):
            ax.quiver(*tcp, *(rg[:, i] * 0.06), color=c, linewidth=2)
        ax.quiver(*tcp, *(rg[:, 2] * 0.10), color='orange', linewidth=2,
                  label='approach')
        f0, f1 = tcp - rg[:, 0] * 0.04, tcp + rg[:, 0] * 0.04
        ax.plot([f0[0], f1[0]], [f0[1], f1[1]], [f0[2], f1[2]], 'c-', lw=3,
                label='finger span guess (8cm)')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_box_aspect((0.25, 0.25, 0.42))
        ax.set_xlim(0.36, 0.64)
        ax.set_ylim(-0.36, -0.06)
        ax.set_zlim(0.0, 0.44)
        ax.legend(fontsize=7, loc='upper left')
        ax.set_title(title)
    fig.tight_layout()
    import io
    buf = io.BytesIO()
    fig.savefig(buf, format='png', dpi=110)
    plt.close(fig)
    meta = dict(source=src, gripper=gripper_note, n_points=int(len(cloud)),
                pick_m=[round(float(v), 4) for v in pick],
                tcp_m=[round(float(v), 4) for v in tcp],
                tcp_to_cloud_mm=round(dmin * 1000, 1),
                cloud_xyz_min=np.round(cloud.min(axis=0), 3).tolist(),
                cloud_xyz_max=np.round(cloud.max(axis=0), 3).tolist())
    return buf.getvalue(), meta
