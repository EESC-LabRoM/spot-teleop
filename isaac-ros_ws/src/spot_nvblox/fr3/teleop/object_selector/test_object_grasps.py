import sys
import unittest
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE / 'ref'))
import grasp_plan_lib as gp  # noqa: E402
import object_grasps as og  # noqa: E402


def bottle(cx=0.60, cy=-0.20, top=0.26, table=0.029, n=1500, seed=0):
    """Visible (camera-facing) half of a bottle: label band dense, cap sparse."""
    rng = np.random.default_rng(seed)
    z = np.r_[rng.uniform(table + 0.07, table + 0.12, n), rng.uniform(top - 0.02, top, 30)]
    r = np.where(z > top - 0.05, 0.015, 0.032)
    face = np.arctan2(-0.599 - cy, 0.219 - cx)  # towards ZED1
    a = face + rng.uniform(-1.2, 1.2, len(z))
    return np.c_[cx + r * np.cos(a), cy + r * np.sin(a), z]


class Sprite(unittest.TestCase):
    def test_cap_estimate(self):
        cap, spread, probs = og.sprite_cap(bottle(), 0.029)
        self.assertLess(np.linalg.norm(cap[:2] - [0.60, -0.20]), 0.01)
        self.assertAlmostEqual(cap[2], 0.26, delta=0.01)
        self.assertEqual(probs, [])

    def test_rejects_bad_clouds(self):
        with self.assertRaises(ValueError):
            og.sprite_cap(bottle()[:10], 0.029)
        short = bottle(top=0.17)
        self.assertTrue(og.sprite_cap(short, 0.029)[2])
        no_cap = bottle()[:1500]
        with self.assertRaises(ValueError):
            og.sprite_cap(no_cap, 0.029)

    def test_ignores_see_through_points_behind(self):
        b = bottle()
        rng = np.random.default_rng(5)
        behind = np.c_[rng.uniform(0.62, 0.70, 600), rng.uniform(-0.15, -0.05, 600), rng.uniform(0.08, 0.15, 600)]
        cap, _, probs = og.sprite_cap(np.vstack([b, behind]), 0.029)
        self.assertLess(np.linalg.norm(cap[:2] - [0.60, -0.20]), 0.012)

    def test_plan_clear_scene(self):
        table = np.c_[np.random.default_rng(1).uniform(0.2, 0.9, (3000, 2)) * [1, 1] - [0, 0.5], np.full(3000, 0.029)]
        p = og.plan_sprite(gp, bottle(), table, gp.Q_HOME)
        self.assertEqual(p['problems'], [])
        self.assertEqual(list(p['stages']), ['S1_home_to_pre', 'S2_pre_to_grasp', 'S3_lift', 'S4_put_back',
                                             'S5_grasp_to_pre', 'S6_pre_to_home'])
        self.assertEqual(len(p['held_spheres_hand']), 4)

    def test_plan_flags_obstacle_next_to_bottle(self):
        rng = np.random.default_rng(2)
        wall = np.c_[np.full(2000, 0.675), rng.uniform(-0.35, -0.05, 2000), rng.uniform(0.05, 0.45, 2000)]
        p = og.plan_sprite(gp, bottle(), wall, gp.Q_HOME)
        self.assertTrue(p['problems'])


if __name__ == '__main__':
    unittest.main()
