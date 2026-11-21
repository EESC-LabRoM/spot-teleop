"""Offline bottle tests on the near-simultaneous twocam capture (63.7 ms)."""
import unittest
from pathlib import Path

import cv2
import numpy as np

import ball_grasp as bg
import bottle_keepout as bk

TC = Path(__file__).resolve().parent.parent / 'twocam'
TABLE_Z = 0.0287
CAMS = {c: bk.Camera.from_npz(TC / f'zed{c}.npz') for c in (1, 2)}
BALL_M = {c: cv2.imread(str(TC / f'zed{c}_tennis_ball_mask.png'), 0) > 0 for c in (1, 2)}
BOT_M = {c: cv2.imread(str(TC / f'zed{c}_sprite_mask.png'), 0) > 0 for c in (1, 2)}
BALL_P = {c: np.load(TC / f'zed{c}_ball_cluster.npy').astype(float) for c in (1, 2)}
FITS = [bg.fit_ball(BALL_P[c], TABLE_Z) for c in (1, 2)]
BOX_LO, BOX_HI = [0.55, -0.5, TABLE_Z], [1.05, 0.1, 0.36]


class Projection(unittest.TestCase):
    def test_own_cloud_lands_on_own_mask(self):
        for c in (1, 2):
            uv, _ = CAMS[c].project(BALL_P[c])
            u = np.round(uv).astype(int)
            self.assertGreater(BALL_M[c][u[:, 1], u[:, 0]].mean(), 0.99)

    def test_cross_camera_reprojection_shows_inconsistency(self):
        for c, o in ((1, 2), (2, 1)):
            uv, _ = CAMS[c].project(BALL_P[o])
            u = np.round(uv).astype(int)
            self.assertLess(BALL_M[c][u[:, 1], u[:, 0]].mean(), 0.75)


class BallControl(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.k, cls.rows = bk.calibrate_dilation(BALL_M, CAMS, FITS, TABLE_Z)

    def test_raw_hull_is_not_conservative(self):
        k0 = dict(self.rows)[0]
        self.assertLess(max(k0), 0.65)          # 2-view hull cuts both sphere hypotheses

    def test_calibrated_dilation(self):
        self.assertEqual(self.k, 8)
        self.assertEqual(dict(self.rows)[8], [1.0, 1.0])

    def test_silhouette_width_is_too_coarse_for_diameter(self):
        # control: ITF ball 65.4-68.6 mm; per-camera width estimate is +-15 mm off
        mid = np.mean([f.center for f in FITS], 0)
        est = []
        for c in (1, 2):
            m = BALL_M[c]
            ys = np.nonzero(m)[0]
            w = np.array([m[y].sum() for y in range(ys.min(), ys.max() + 1)])
            est.append(np.quantile(w, .9) * np.linalg.norm(mid - CAMS[c].t) / CAMS[c].K[0, 0])
        self.assertGreater(max(abs(e - 0.067) for e in est), 0.003)
        self.assertGreater(max(est) - min(est), 0.010)


class BottleKeepout(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.box = bk.bottle_keepout(BOT_M, CAMS, BALL_M, FITS, TABLE_Z, BOX_LO, BOX_HI)
        cls.H0 = bk.visual_hull(BOT_M, CAMS, 0, BOX_LO, BOX_HI)
        cls.Hk = bk.visual_hull(BOT_M, CAMS, cls.box['k_applied_px'], BOX_LO, BOX_HI)

    def test_keepout_contains_raw_hull_and_covers_both_masks(self):
        self.assertEqual(bk.contained_fraction(self.Hk, self.H0), 1.0)
        for c in (1, 2):
            self.assertGreater(bk.reprojection_coverage(self.Hk, BOT_M[c], CAMS[c]), 0.99)

    def test_keepout_cuboid_is_bounded_and_on_table(self):
        c, d = np.array(self.box['center']), np.array(self.box['dims'])
        lo, hi = c - d / 2, c + d / 2
        self.assertTrue(TABLE_Z - 0.002 <= lo[2] <= TABLE_Z, lo)
        self.assertEqual(self.box['k_applied_px'], self.box['k_calibrated_px'] + 1)
        self.assertTrue(np.all(d[:2] < 0.25) and d[2] < 0.30, self.box)
        self.assertTrue(np.all(self.Hk.min(0) >= lo - 1e-9) and np.all(self.Hk.max(0) <= hi + 1e-9))
        # bottle stands well clear of the ball (separate obstacles)
        ball_mid = np.mean([f.center for f in FITS], 0)
        self.assertGreater(np.linalg.norm(c[:2] - ball_mid[:2]), 0.25)

    def test_keepout_excludes_depth_smear_far_points(self):
        # the ZED1 depth cloud spans 243 mm; the silhouette keep-out is
        # tighter in x and does not chase smear beyond it
        smear = np.load(TC / 'zed1_sprite_cloud.npy').astype(float)
        self.assertGreater(np.ptp(smear[:, 0]), self.box['dims'][0])

    def test_hull_requires_two_cameras_and_valid_masks(self):
        with self.assertRaises(ValueError):
            bk.visual_hull({1: BOT_M[1]}, {1: CAMS[1]}, 0, BOX_LO, BOX_HI)
        with self.assertRaises(ValueError):
            bk.visual_hull({1: BOT_M[1], 2: np.zeros_like(BOT_M[2])}, CAMS, 0, BOX_LO, BOX_HI)


class BottleGrasp(unittest.TestCase):
    def test_real_evidence_refuses_grasp(self):
        axis = bk.visual_hull(BOT_M, CAMS, 0, BOX_LO, BOX_HI)[:, :2].mean(0)
        tops = {c: bk.top_ray_height(BOT_M[c], CAMS[c], axis) for c in (1, 2)}
        self.assertGreater(abs(tops[1] - tops[2]), 0.015)          # ~23 mm
        lat = bg.derive_budget({'1': FITS[0], '2': FITS[1]}, 0.0056).lateral_m
        v = bk.assess_bottle_grasp(diameter_bounds_m=(0.055, 0.090), lateral_budget_m=lat,
                                   top_z_by_cam=tops)
        self.assertFalse(v['ok'])
        self.assertEqual(len(v['reasons']), 3, v)

    def test_even_ideal_geometry_needs_depth_points(self):
        v = bk.assess_bottle_grasp(diameter_bounds_m=(0.06, 0.062), lateral_budget_m=0.003,
                                   top_z_by_cam={1: 0.25, 2: 0.251})
        self.assertFalse(v['ok'])
        self.assertIn('transparent', v['reasons'][0])

    def test_unknown_diameter_refused(self):
        v = bk.assess_bottle_grasp(diameter_bounds_m=(float('nan'), 0.06), lateral_budget_m=0.003,
                                   top_z_by_cam={1: 0.25, 2: 0.25}, depth_pts_between_pads=100)
        self.assertFalse(v['ok'])


if __name__ == '__main__':
    unittest.main()
