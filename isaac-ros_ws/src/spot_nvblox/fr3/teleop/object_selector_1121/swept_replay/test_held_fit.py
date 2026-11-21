"""Offline tests for the tighter held-sphere candidates (no ROS/robot)."""
import json
import unittest
from pathlib import Path

import numpy as np

import held_fit as hf

HERE = Path(__file__).resolve().parent
RUNS = ('20261121_053830_drill_loop_regen', '20261121_050322_drill_oneshot_regen')
RESULTS = HERE / 'held_fit_results.json'


class Frame(unittest.TestCase):
    def test_grasp_frame_matches_plan(self):
        for run in RUNS:
            plan, _, _ = hf.load_run(run)
            T = hf.hand_T(hf.grasp_q(plan))
            np.testing.assert_allclose(T[:3, 3], plan['grasp_pos'], atol=2e-4)
            np.testing.assert_allclose(T[:3, :3], plan['hand_R'], atol=1e-4)

    def test_world_hand_round_trip(self):
        plan, _, old = hf.load_run(RUNS[0])
        T = hf.hand_T(hf.grasp_q(plan))
        np.testing.assert_allclose(hf.to_hand(hf.to_world(old, T), T), old, atol=1e-12)


class LiveBaseline(unittest.TestCase):
    def test_live_v3_crosses_table_and_matches_recorded_coverage(self):
        expect = {RUNS[0]: (-34.7, 0.995), RUNS[1]: (-14.8, 0.982)}
        for run, (floor, cov) in expect.items():
            plan, cloud, old = hf.load_run(run)
            m = hf.evaluate(old, plan, cloud, samples=20000)
            self.assertAlmostEqual(m['grasp_floor_mm'], floor, delta=0.2)
            self.assertAlmostEqual(m['coverage_inside'], cov, delta=0.003)   # live 'coverage' = inside fraction
            ok, why = hf.passes(m)
            self.assertFalse(ok)
            self.assertTrue(any('below table' in r for r in why))


class FlatBaseGeometry(unittest.TestCase):
    def test_need_never_exceeds_height_above_table(self):
        P = np.array([[0, 0, 0.0291 + h] for h in (0.0, 0.002, 0.008, 0.03)])
        for pen in (0.0, 0.01):
            need = hf.need_clearance(P, 0.0291, pen)
            self.assertTrue(np.all(need <= P[:, 2] - (0.0291 - pen) + 1e-12))
            self.assertTrue(np.all(need <= hf.MARGIN))

    def test_sphere_on_table_reaches_little_footprint_near_table(self):
        # a sphere resting on the plane covers radius sqrt(2 r h - h^2) at height h:
        # with the 0.12 m cap, a 50 mm half-footprint needs h >= ~10.9 mm
        r, w = 0.12, 0.05
        h = r - np.sqrt(r * r - w * w)
        self.assertGreater(h, 0.010)


class SyntheticFit(unittest.TestCase):
    def test_box_on_table_respects_floor_and_coverage(self):
        table = 0.03
        rng = np.random.default_rng(1)
        g = rng.uniform([-0.03, -0.03, table + 0.005], [0.03, 0.03, table + 0.15], (4000, 3))
        face = rng.integers(0, 3, len(g))
        g[face == 0, 0] = np.sign(g[face == 0, 0]) * 0.03                  # side shell
        g[face == 1, 1] = np.sign(g[face == 1, 1]) * 0.03
        g[face == 2, 2] = table + 0.15                                      # top
        plan, _, _ = hf.load_run(RUNS[0])
        T = hf.hand_T(hf.grasp_q(plan))
        cloud = g + T[:3, 3] * [1, 1, 0]                                    # under the hand
        plan = dict(plan, table_z=table)
        S, info = hf.fit(plan, cloud, 4, 0.0, time_limit=30)
        self.assertIsNotNone(S, info)
        m = hf.evaluate(S, plan, cloud, samples=20000)
        self.assertGreaterEqual(m['grasp_floor_mm'], -hf.TABLE_TOL * 1e3)
        self.assertGreaterEqual(m['coverage_inside'], 0.95)
        self.assertGreaterEqual(m['lift_floor_end_mm'], hf.ACTIVATION * 1e3)


@unittest.skipUnless(RESULTS.exists(), 'run held_fit.py first')
class SavedCandidates(unittest.TestCase):
    """Re-evaluates every saved candidate from scratch (no trust in the JSON numbers)."""
    @classmethod
    def setUpClass(cls):
        cls.res = json.loads(RESULTS.read_text())

    def test_every_candidate_respects_table_and_lift(self):
        for run, rows in self.res.items():
            plan, cloud, _ = hf.load_run(run)
            for r in rows:
                if 'spheres_hand' not in r:
                    continue
                pen = r['floor_pen_mm'] / 1e3
                m = hf.evaluate(np.array(r['spheres_hand']), plan, cloud, samples=20000, floor_pen=pen)
                self.assertGreaterEqual(m['grasp_floor_mm'], -(pen + hf.TABLE_TOL) * 1e3, (run, r['model']))
                self.assertGreaterEqual(m['lift_floor_end_mm'], hf.ACTIVATION * 1e3, (run, r['model']))
                self.assertEqual(m['coverage_inside'], r['coverage_inside'])

    def test_table_crossing_never_reported_as_pass(self):
        for rows in self.res.values():
            for r in rows:
                if r.get('grasp_floor_mm', 0) < -hf.TABLE_TOL * 1e3:
                    self.assertFalse(r['pass'])

    def test_no_candidate_is_marked_validated(self):
        for rows in self.res.values():
            for r in rows:
                self.assertNotIn('validated', json.dumps(r.get('fit', {})).lower())


if __name__ == '__main__':
    unittest.main()
