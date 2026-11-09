"""Regression test for the agnostic grasp re-ranker (drill grasp loop).

Covers: centered beats off-center at lower confidence; bilateral (straddle)
beats edge contact; dense beats sparse; shallow graze (TCP above top) loses;
side grasps are gated out; intended ranking wins on a synthetic box.

Pure numpy, no ROS. Run in a container that mounts the repo, e.g.:

  docker exec vm-viewer python3 \\
    /home/nexus/spot-teleop/fr3_real_repro/grasp_loop/tests/test_grasp_rank.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from grasp_loop import density_gate, grasp_metrics, rank_grasps, rank_score, tcp_of_pose


def _box_cloud(cx=0.5, cy=-0.2, z0=0.05, z1=0.20, sx=0.10, sy=0.06, n=4000,
               seed=0):
    rng = np.random.default_rng(seed)
    return np.column_stack([rng.uniform(cx - sx / 2, cx + sx / 2, n),
                            rng.uniform(cy - sy / 2, cy + sy / 2, n),
                            rng.uniform(z0, z1, n)])


def _top_down(x, y, z_hand, yaw=0.0):
    """Top-down hand pose: approach +tool-z pointing down (-world-z)."""
    c, s = np.cos(yaw), np.sin(yaw)
    R = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, -1.0]])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z_hand]
    return T


def test_tcp_and_wrap_geometry():
    top = 0.20
    T = _top_down(0.5, -0.2, top + 0.103 - 0.030)  # TCP 30 mm below top
    tcp = tcp_of_pose(T)
    assert abs(tcp[2] - (top - 0.030)) < 1e-9, f'tcp z wrong: {tcp[2]}'
    m = grasp_metrics(_box_cloud(), T, [0.5, -0.2], top)
    assert abs(m['wrap'] - 0.030) < 1e-9, f'wrap wrong: {m["wrap"]}'
    assert abs(m['vert'] - 1.0) < 1e-9, f'vert wrong: {m["vert"]}'
    assert m['off_xy'] < 1e-9, f'off wrong: {m["off_xy"]}'


def test_centered_beats_offcenter_despite_lower_conf():
    Pc = _box_cloud()
    top, C = 0.20, [0.5, -0.2]
    good = _top_down(0.5, -0.2, top + 0.103 - 0.030)          # centered, conf .70
    bad = _top_down(0.5 + 0.09, -0.2, top + 0.103 - 0.030)    # 90 mm off, conf .90
    mg = grasp_metrics(Pc, good, C, top)
    mb = grasp_metrics(Pc, bad, C, top)
    sg, _ = rank_score(0.70, mg, 0.05, len(Pc))
    sb, _ = rank_score(0.90, mb, 0.05, len(Pc))
    assert sg > sb, f'centered {sg:.4f} must beat off-center {sb:.4f}'


def test_straddle_beats_edge():
    Pc = _box_cloud()
    top, C = 0.20, [0.5, -0.2]
    mid = _top_down(0.5, -0.2, top + 0.103 - 0.030)            # TCP in the box
    edge = _top_down(0.5 + 0.049, -0.2, top + 0.103 - 0.030)   # TCP at +x face
    mm = grasp_metrics(Pc, mid, C, top)
    me = grasp_metrics(Pc, edge, C, top)
    assert mm['bilateral'] > 0.7, f'mid bilateral too low: {mm["bilateral"]}'
    assert me['bilateral'] < 0.4, f'edge bilateral too high: {me["bilateral"]}'
    sm, _ = rank_score(0.7, mm, 0.05, len(Pc))
    se, _ = rank_score(0.7, me, 0.05, len(Pc))
    assert sm > se, 'straddle must beat edge at equal confidence'


def test_side_grasps_gated():
    Pc = _box_cloud()
    top = 0.20
    side = np.eye(4)  # approach +x (horizontal): vert = -R[2,2] = -1
    side[:3, 3] = [0.5, -0.2, 0.15]
    downs = []
    for k in range(4):
        T = _top_down(0.5 + 0.01 * k, -0.2, top + 0.103 - 0.030)
        downs.append(np.concatenate([T.reshape(-1), [0.5]]))
    G = np.vstack([np.concatenate([side.reshape(-1), [0.99]])] + downs)
    ranked = rank_grasps(G, Pc, [0.5, -0.2], [0.10, 0.06], top)
    idxs = [i for i, _, _, _ in ranked]
    assert 0 not in idxs, 'side grasp (conf 0.99) must be gated out'
    assert len(ranked) == 4, f'expected 4 survivors, got {len(ranked)}'


def test_shallow_graze_loses():
    Pc = _box_cloud()
    top, C = 0.20, [0.5, -0.2]
    deep = _top_down(0.5, -0.2, top + 0.103 - 0.030)
    shallow = _top_down(0.5, -0.2, top + 0.103 + 0.010)  # TCP 10 mm above top
    md = grasp_metrics(Pc, deep, C, top)
    ms = grasp_metrics(Pc, shallow, C, top)
    sd, _ = rank_score(0.7, md, 0.05, len(Pc))
    ss, _ = rank_score(0.7, ms, 0.05, len(Pc))
    assert sd > ss, '30 mm wrap must beat a fingertip graze'


def test_full_ranking_orders_sensibly():
    Pc = _box_cloud()
    top = 0.20
    T_best = _top_down(0.5, -0.2, top + 0.103 - 0.030)
    T_off = _top_down(0.5 + 0.08, -0.2, top + 0.103 - 0.030)
    T_air = _top_down(0.5, -0.2 + 0.20, top + 0.103 - 0.030)  # outside cloud
    G = np.vstack([np.concatenate([T.reshape(-1), [c]]) for T, c in
                   [(T_air, 0.95), (T_off, 0.90), (T_best, 0.70)]])
    ranked = rank_grasps(G, Pc, [0.5, -0.2], [0.10, 0.06], top)
    assert ranked[0][0] == 2, f'centered grasp must win, got idx {ranked[0][0]}'
    assert ranked[-1][0] == 0, 'air grasp must rank last'
    _, _, _, b = ranked[0]
    for k in ('center_w', 'density_w', 'bilateral_w', 'wrap_w', 'contact_w'):
        assert k in b, f'breakdown missing {k}'


def _tilted_grasp(x, y, z_hand, tilt_deg=30.0):
    """Hand pose leaning tilt_deg off vertical: tool-z is NOT world -z."""
    t = np.radians(tilt_deg)
    R = np.array([[np.cos(t), 0.0, np.sin(t)],
                  [0.0, 1.0, 0.0],
                  [-np.sin(t), 0.0, -np.cos(t)]])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z_hand]
    return T


def test_density_gate_measures_at_tool_tcp():
    T = _tilted_grasp(0.5, -0.2, 0.30, tilt_deg=30.0)
    tcp = tcp_of_pose(T)  # true fingertip, 51 mm sideways of hand-z projection
    rng = np.random.default_rng(1)
    Pc = tcp + rng.normal(0, 0.008, (2000, 3))  # dense exactly at the TCP
    rows = []
    for k in range(5):  # five near-identical tilted candidates (gate needs 5)
        Tk = T.copy()
        Tk[:3, 3] += [0.002 * k, 0.0, 0.0]
        rows.append(np.concatenate([Tk.reshape(-1), [0.9 - 0.01 * k]]))
    G = np.vstack(rows)
    keep, starved, scored = density_gate(G, Pc, list(range(5)))
    assert keep == list(range(5)), f'true-TCP grasps must survive, got {keep}'
    assert not starved, f'gate must not starve on dense TCP (nnears={[n for _, n in scored]})'
    assert all(n >= 150 for _, n in scored), 'too few pts at the TCP'


if __name__ == '__main__':
    test_tcp_and_wrap_geometry()
    test_centered_beats_offcenter_despite_lower_conf()
    test_straddle_beats_edge()
    test_side_grasps_gated()
    test_shallow_graze_loses()
    test_full_ranking_orders_sensibly()
    test_density_gate_measures_at_tool_tcp()
    print('test_grasp_rank: 7/7 PASS')
