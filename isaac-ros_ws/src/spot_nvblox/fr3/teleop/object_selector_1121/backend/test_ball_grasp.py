"""Replay tests on the real 11-21 ball/bottle clouds (offline only)."""
import unittest
from pathlib import Path

import numpy as np

import ball_grasp as bg

D = Path(__file__).resolve().parent.parent
TABLE_Z = 0.029                      # plan.json table_z 0.0289-0.0291 on 11-21
Z1 = np.load(D / 'tennis_ball_cluster.npy').astype(float)          # cleaned ZED1
Z1_RAW = np.load(D / 'tennis_ball_snapshot_cloud.npy').astype(float)
Z2_RAW = np.load(D / 'zed2/tennis_ball_snapshot_cloud.npy').astype(float)
SPRITE1 = np.load(D / 'sprite_snapshot_cloud.npy').astype(float)
SPRITE2 = np.load(D / 'zed2/sprite_snapshot_cloud.npy').astype(float)


def clean_zed2():
    """Same table cut the ZED1 cluster saw (bottom ~0.038) + radius filter."""
    P = Z2_RAW[Z2_RAW[:, 2] > 0.04]
    return P[np.linalg.norm(P - np.median(P, 0), axis=1) < 0.06]


FIT1 = bg.fit_ball(Z1, TABLE_Z)
TINY = bg.ErrorBudget(0.003, 0.003)   # hypothetical, for "validator is not always-false"


def centered(z, **kw):
    return bg.HandPose.top_down([FIT1.center[0], FIT1.center[1], z], **kw)


class Fit(unittest.TestCase):
    def test_cleaned_zed1_cloud_is_a_tennis_ball(self):
        self.assertTrue(FIT1.ok, FIT1.reasons)
        self.assertAlmostEqual(FIT1.radius, 0.0349, delta=0.001)
        self.assertLess(FIT1.rms, 0.002)

    def test_uncleaned_clouds_with_table_points_refused(self):
        for P in (Z1_RAW, Z2_RAW):
            f = bg.fit_ball(P, TABLE_Z)
            self.assertFalse(f.ok)

    def test_resting_offset_is_reported_not_hidden(self):
        # fitted bottom ~10 mm below the planner table estimate: real z
        # misregistration or fit bias; it feeds the vertical budget.
        self.assertAlmostEqual(FIT1.table_offset, -0.0096, delta=0.001)

    def test_malformed_inputs(self):
        for P in (np.zeros((10, 3)), np.full((500, 3), np.nan), np.zeros((500, 2))):
            self.assertFalse(bg.fit_ball(P, TABLE_Z).ok)
        self.assertFalse(bg.fit_ball(Z1, float('nan')).ok)
        self.assertFalse(bg.fit_ball(Z1 + [0, 0, 0.05], TABLE_Z).ok)   # floating ball


class GraspRules(unittest.TestCase):
    def test_reported_graspgen_tip_15mm_pinches_above_equator(self):
        ok, reasons, m = bg.check_grasp(centered(FIT1.top_observed - 0.015), FIT1, Z1, TABLE_Z, TINY)
        self.assertFalse(ok)
        self.assertTrue(any('equator' in r for r in reasons))

    def test_drill_tip_min_alone_is_insufficient_for_ball(self):
        z = FIT1.top_observed - 0.032          # passes TIP_MIN 3 cm
        ok, reasons, m = bg.check_grasp(centered(z), FIT1, Z1, TABLE_Z, TINY)
        self.assertGreaterEqual(m['tip_below_top_mm'], 30)
        self.assertFalse(ok)
        self.assertTrue(any('equator' in r for r in reasons))

    def test_centered_equator_grasp_passes_only_under_tiny_hypothetical_budget(self):
        lo, hi = bg.feasible_tcp_window(FIT1, TABLE_Z, TINY)
        self.assertLess(lo, hi)
        ok, reasons, m = bg.check_grasp(centered((lo + hi) / 2), FIT1, Z1, TABLE_Z, TINY)
        self.assertTrue(ok, reasons)
        self.assertGreaterEqual(m['pts_between_pads'], bg.MIN_PTS_BETWEEN_PADS)

    def test_tilt_offset_and_table_refusals(self):
        z = bg.feasible_tcp_window(FIT1, TABLE_Z, TINY)[0] + 0.004
        bad = [centered(z, tilt=np.radians(20)),
               bg.HandPose.top_down(FIT1.center + [0, 0.004, 0] - [0, 0, FIT1.center[2] - z]),
               bg.HandPose.top_down(FIT1.center + [0.010, 0, 0] - [0, 0, FIT1.center[2] - z]),
               centered(TABLE_Z + 0.004)]
        for p in bad:
            self.assertFalse(bg.check_grasp(p, FIT1, Z1, TABLE_Z, TINY)[0])

    def test_invalid_fit_blocks_every_pose(self):
        bad = bg.fit_ball(Z1_RAW, TABLE_Z)
        self.assertFalse(bg.check_grasp(centered(0.045), bad, Z1_RAW, TABLE_Z, TINY)[0])

    def test_graspgen_row_convention_round_trips(self):
        want = centered(0.045)
        Rh = np.c_[want.X, want.Y, want.A]
        Rm = np.eye(4)
        Rm[:3, :3] = Rh @ bg.Rz(-np.pi / 2)
        depth = 0.01
        Rm[:3, 3] = want.tcp - want.A * (depth + bg.TCP_OFFSET)
        got = bg.HandPose.from_graspgen(Rm.ravel(), depth)
        for a, b in ((got.tcp, want.tcp), (got.X, want.X), (got.Y, want.Y), (got.A, want.A)):
            np.testing.assert_allclose(a, b, atol=1e-9)
        rows = bg.replay_graspgen([Rm.ravel()], FIT1, Z1, TABLE_Z, TINY, depths=(depth, 0.04))
        self.assertEqual([r['ok'] for r in rows], [True, False])

    def test_zero_budget_not_allowed(self):
        for a, b in ((0, 0.003), (0.003, 0), (float('nan'), 0.003)):
            with self.assertRaises(ValueError):
                bg.ErrorBudget(a, b)


class EvidenceBudget(unittest.TestCase):
    def setUp(self):
        self.fit2 = bg.fit_ball(clean_zed2(), TABLE_Z)

    def test_two_camera_disagreement_exceeds_gripper_capacity(self):
        self.assertTrue(self.fit2.ok, self.fit2.reasons)
        b = bg.derive_budget({'zed1': FIT1, 'zed2': self.fit2}, broker_drift_m=0.0056)
        cap = bg.lateral_budget_capacity(FIT1)
        self.assertAlmostEqual(cap, 0.0051, delta=0.0003)        # 79.9 mm open vs 69.8 mm ball
        self.assertGreater(b.lateral_m, 0.015)                    # ~16.6 mm measured
        self.assertGreater(b.lateral_m, cap)

    def test_end_to_end_record_refuses_on_real_evidence(self):
        fits = {'zed1': FIT1, 'zed2': self.fit2}
        lo, hi = bg.feasible_tcp_window(FIT1, TABLE_Z, TINY)
        best = centered((lo + hi) / 2)
        rec = bg.validate_ball_binding('ball_run', best, fits, 'zed1', Z1, TABLE_Z, 0.0056)
        self.assertFalse(rec['ok'])
        self.assertTrue(any('exceeds gripper capacity' in r for r in rec['reasons']))
        self.assertTrue(any('no vertical TCP window' in r for r in rec['reasons']))
        # exhaustive top-down sweep: nothing passes with the evidence budget
        b = bg.derive_budget(fits, 0.0056)
        for z in np.arange(0.02, 0.09, 0.001):
            for dx in np.arange(-0.01, 0.011, 0.002):
                for dy in np.arange(-0.01, 0.011, 0.002):
                    p = bg.HandPose.top_down([FIT1.center[0] + dx, FIT1.center[1] + dy, z])
                    self.assertFalse(bg.check_grasp(p, FIT1, Z1, TABLE_Z, b)[0])

    def test_single_camera_cannot_bound_registration(self):
        with self.assertRaises(ValueError):
            bg.derive_budget({'zed1': FIT1}, 0.001)
        rec = bg.validate_ball_binding('r', centered(0.045), {'zed1': FIT1}, 'zed1', Z1, TABLE_Z, 0.001)
        self.assertFalse(rec['ok'])

    def test_unmeasured_broker_drift_refused(self):
        with self.assertRaises(ValueError):
            bg.derive_budget({'zed1': FIT1, 'zed2': self.fit2}, float('nan'))


class HeldAndSprite(unittest.TestCase):
    def test_held_width_window(self):
        self.assertFalse(bg.held_width_ok(0.000216, FIT1))   # 11-21 drill slip width
        self.assertFalse(bg.held_width_ok(0.0799, FIT1))     # still open
        self.assertTrue(bg.held_width_ok(0.066, FIT1))
        self.assertFalse(bg.held_width_ok(float('nan'), FIT1))

    def test_held_model_single_sphere(self):
        s = bg.held_model(FIT1, np.eye(4))
        self.assertEqual(len(s), 1)
        self.assertAlmostEqual(s[0][3], FIT1.radius + 0.012, places=3)

    def test_transparent_bottle_never_ok(self):
        for P in (SPRITE1, SPRITE2):
            a = bg.assess_sprite_cloud(P)
            self.assertFalse(a['ok'])
            self.assertGreater(len(a['reasons']), 1, a)     # fragmentation evidence, not only the prior


class GraspGenReplay(unittest.TestCase):
    """Real 11-21 GraspGen pool (200x17, copied from the VM by the peer lane)."""
    @classmethod
    def setUpClass(cls):
        cls.G = np.load(D / 'tennis_ball_snapshot_grasps.npy')[:, :16].astype(float)
        cls.fit2 = bg.fit_ball(clean_zed2(), TABLE_Z)

    def test_pool_is_homogeneous_poses(self):
        self.assertEqual(self.G.shape, (200, 16))
        np.testing.assert_allclose(self.G[:, 12:16], np.tile([0, 0, 0, 1], (200, 1)), atol=1e-6)

    def test_zero_pass_with_evidence_budget(self):
        b = bg.derive_budget({'zed1': FIT1, 'zed2': self.fit2}, 0.0056)
        rows = bg.replay_graspgen(self.G, FIT1, Z1, TABLE_Z, b)
        self.assertEqual(len(rows), 800)
        self.assertEqual(sum(r['ok'] for r in rows), 0)

    def test_hypothetical_3mm_budget_leaves_one_marginal_candidate(self):
        rows = bg.replay_graspgen(self.G, FIT1, Z1, TABLE_Z, TINY)
        ok = [r for r in rows if r['ok']]
        self.assertEqual([(r['idx'], r['depth']) for r in ok], [(14, 0.03)])
        self.assertLess(ok[0]['side_clear_mm'], 1.5)       # ~1 mm: marginal even then
        strict = sum(1 for row in self.G if -bg.HandPose.from_graspgen(row, 0).A[2] >= bg.VERT_MIN)
        self.assertEqual(strict, 9)                         # only 9/200 meet 15 deg tilt


class TwoCamNearSimultaneous(unittest.TestCase):
    """Peer capture twocam/ (63.7 ms spread). Narrows, does not prove, the cause."""
    T = 0.0287

    @classmethod
    def setUpClass(cls):
        tc = D / 'twocam'
        cls.P = {c: np.load(tc / f'zed{c}_ball_cluster.npy').astype(float) for c in (1, 2)}
        cls.F = {c: bg.fit_ball(cls.P[c], cls.T) for c in (1, 2)}
        cls.cam = {c: np.load(tc / f'zed{c}.npz')['camera'][:3] for c in (1, 2)}

    def test_both_fits_valid_same_radius_centres_16mm_apart(self):
        self.assertTrue(self.F[1].ok and self.F[2].ok)
        self.assertAlmostEqual(self.F[1].radius, self.F[2].radius, delta=0.0005)
        dxy = np.linalg.norm(self.F[1].center[:2] - self.F[2].center[:2])
        self.assertAlmostEqual(dxy, 0.0166, delta=0.001)

    def test_fixed_radius_keeps_the_offset(self):
        for r in (0.0327, 0.0343):        # ITF min / max radius
            c1 = bg.fit_center_fixed_radius(self.P[1], r, self.F[1].center)
            c2 = bg.fit_center_fixed_radius(self.P[2], r, self.F[2].center)
            self.assertGreater(np.linalg.norm(c1[:2] - c2[:2]), 0.015)

    def test_offset_is_across_rays_and_not_a_depth_scale(self):
        d = bg.diagnose_two_camera(self.F[1], self.F[2], self.cam[1], self.cam[2])
        self.assertLess(abs(d['a_along_mm']), 5)
        self.assertLess(abs(d['b_along_mm']), 5)
        self.assertGreater(d['a_across_mm'], 15)
        self.assertGreater(d['b_across_mm'], 15)
        self.assertGreater(d['residual_after_depth_scale_mm'], 15)

    def test_still_zero_graspgen_passes(self):
        b = bg.derive_budget({'zed1': self.F[1], 'zed2': self.F[2]}, 0.0056)
        G = np.load(D / 'tennis_ball_snapshot_grasps.npy')[:, :16].astype(float)
        self.assertEqual(sum(r['ok'] for r in bg.replay_graspgen(G, self.F[1], self.P[1], self.T, b)), 0)


if __name__ == '__main__':
    unittest.main()
