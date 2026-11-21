import json
import sys
import unittest
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE / 'ref'))
import grasp_plan_lib as gp  # noqa: E402
from fr3_spheres import FR3Spheres  # noqa: E402
from replay_plan import replay, sample_stage  # noqa: E402


class TestFR3Spheres(unittest.TestCase):
    k = FR3Spheres()

    def test_hand_matches_planner_fk(self):
        rng = np.random.default_rng(0)
        for q in [gp.Q_HOME] + [rng.uniform(gp.QLO, gp.QHI) for _ in range(25)]:
            np.testing.assert_allclose(self.k.link_poses(q)['fr3_hand'], gp.fk(q), atol=1e-9)

    def test_sphere_count_and_buffer(self):
        S, lab = self.k.spheres(gp.Q_HOME)
        self.assertEqual(len(S), 59)
        self.assertIn('fr3_leftfinger', lab)
        self.assertTrue((S[:, 3] >= 0.004).all())

    def test_held_spheres_follow_hand(self):
        held = np.array([[0, 0, 0.2, 0.05]])
        S, lab = self.k.spheres(gp.Q_HOME, held)
        T = self.k.link_poses(gp.Q_HOME)['fr3_hand']
        np.testing.assert_allclose(S[np.array(lab) == 'held'][0, :3], T[:3, :3] @ [0, 0, 0.2] + T[:3, 3])

    def test_rejects_bad_q(self):
        with self.assertRaises(ValueError):
            self.k.spheres([0] * 6)


class TestReplay(unittest.TestCase):
    def test_sampling_respects_step(self):
        s, end = sample_stage(np.zeros(7), [np.full(7, 0.3)], 0.01)
        self.assertLessEqual(np.abs(np.diff(s, axis=0)).max(), 0.01 + 1e-12)
        np.testing.assert_allclose(end, 0.3)

    def test_real_plan_flags_held_model_below_table(self):
        # 053830 drill plan: held model must breach the table at the grasp, arm stays clear
        L = HERE / 'ref/20261121_053830_drill_loop_regen'
        plan = json.loads((L / 'plan.json').read_text())
        held = np.array(json.loads((L / 'held.json').read_text())['spheres'])
        far = np.array([[5.0, 5.0, 5.0]])  # no obstacles: isolate table/held geometry
        r = replay(plan, far, np.load(L / 'cloud.npy'), held)
        self.assertFalse(r['known_space_verified'])
        self.assertLess(r['stages']['S3_lift']['min_table_clear_m'], 0)
        self.assertEqual(r['stages']['S3_lift']['table_link'], 'held')
        self.assertGreater(r['stages']['S2_pre_to_grasp']['min_table_clear_m'], 0.05)


if __name__ == '__main__':
    unittest.main()
