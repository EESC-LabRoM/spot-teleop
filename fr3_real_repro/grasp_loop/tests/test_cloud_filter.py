"""Focused regression test for the S3 cloud filters (drill grasp loop).

Covers: mask erosion trims silhouette bleed; floor cut drops table points by
known geometry while keeping the standing object; lost-tracker clouds (mostly
below the floor) fail fast instead of producing a junk grasp.

Pure numpy, no ROS. The repo host has no numpy, so run it in a container that
mounts the repo, e.g.:

  docker exec vm-viewer python3 \\
    /home/nexus/spot-teleop/fr3_real_repro/grasp_loop/tests/test_cloud_filter.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from grasp_loop import Fail, erode_mask_np, floor_cut


def test_erode_trims_border_only():
    M = np.zeros((60, 60), np.uint8)
    M[10:50, 10:50] = 255  # 40x40 block
    E = erode_mask_np(M, 2)
    assert E[20, 20] == 255, 'interior must survive'
    assert E[10, 25] == 0 and E[11, 25] == 0, 'top border must trim 2px'
    assert E[49, 25] == 0 and E[48, 25] == 0, 'bottom border must trim 2px'
    assert E[25, 10] == 0 and E[25, 49] == 0, 'side borders must trim'
    assert int((E > 0).sum()) == 36 * 36, '40x40 - 2px/side = 36x36'


def test_erode_empty_stays_empty():
    M = np.zeros((30, 30), np.uint8)
    assert int((erode_mask_np(M, 2) > 0).sum()) == 0


def test_floor_cut_keeps_drill_drops_table():
    rng = np.random.default_rng(0)
    table = np.column_stack([rng.uniform(0.4, 0.7, 4000),
                             rng.uniform(-0.35, -0.15, 4000),
                             rng.normal(0.002, 0.006, 4000)])
    drill = np.column_stack([rng.normal(0.60, 0.02, 3000),
                             rng.normal(-0.25, 0.02, 3000),
                             rng.uniform(0.03, 0.25, 3000)])
    P = np.vstack([table, drill])
    kept, frac = floor_cut(P)
    assert 0.50 < frac < 0.65, f'table fraction wrong: {frac}'
    assert (kept[:, 2] >= 0.015).all(), 'no below-floor points may remain'
    assert (kept[:, 2] > 0.10).mean() > 0.6, 'drill body must dominate kept'
    assert len(kept) > 2500, 'must keep the drill points'


def test_floor_cut_lost_tracker_fails():
    rng = np.random.default_rng(1)
    P = np.column_stack([rng.uniform(0.4, 0.7, 5000),
                         rng.uniform(-0.35, -0.15, 5000),
                         rng.normal(0.002, 0.006, 5000)])  # all table
    try:
        floor_cut(P)
    except Fail as e:
        assert 'lost the object' in str(e), f'wrong message: {e}'
    else:
        raise AssertionError('all-table cloud must Fail, not pass silently')


if __name__ == '__main__':
    test_erode_trims_border_only()
    test_erode_empty_stays_empty()
    test_floor_cut_keeps_drill_drops_table()
    test_floor_cut_lost_tracker_fails()
    print('test_cloud_filter: 4/4 PASS')
