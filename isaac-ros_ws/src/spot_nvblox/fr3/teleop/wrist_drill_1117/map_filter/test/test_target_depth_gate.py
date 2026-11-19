import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from drill_depth_filter import IntegrationPaused, apply_target_gate


class TargetDepthGateTests(unittest.TestCase):
    def setUp(self):
        # Three objects occupy distinct image regions; only the drill is targeted.
        self.depth = np.full((80, 120), 0.8, dtype=np.float32)
        self.drill = np.zeros_like(self.depth, dtype=np.uint8)
        self.drill[20:40, 20:45] = 255
        self.ball = (slice(45, 55), slice(70, 80))
        self.bottle = (slice(10, 30), slice(90, 100))
        self.screwdriver = (slice(55, 65), slice(30, 70))

    def test_no_target_passes_full_depth_and_does_not_empty_map(self):
        out, removed = apply_target_gate(self.depth, "NO_TARGET")
        np.testing.assert_array_equal(out, self.depth)
        self.assertFalse(removed.any())
        self.assertGreater(np.count_nonzero(out), 0)

    def test_active_drill_removes_only_drill_mask(self):
        out, removed = apply_target_gate(
            self.depth, "ACTIVE:drill", self.drill, dilate=0)
        np.testing.assert_array_equal(out[~removed], self.depth[~removed])
        self.assertTrue(np.all(out[self.drill > 0] == 0))
        for region in (self.ball, self.bottle, self.screwdriver):
            self.assertTrue(np.all(out[region] == self.depth[region]))
        self.assertGreater(np.count_nonzero(out), 0)
        self.assertEqual(int(removed.sum()), int(np.count_nonzero(self.drill)))

    def test_active_without_mask_pauses(self):
        with self.assertRaises(IntegrationPaused):
            apply_target_gate(self.depth, "ACTIVE:drill")

    def test_lost_target_pauses(self):
        with self.assertRaises(IntegrationPaused):
            apply_target_gate(self.depth, "LOST:drill")

    def test_unknown_state_pauses(self):
        with self.assertRaises(IntegrationPaused):
            apply_target_gate(self.depth, "")

    def test_empty_depth_pauses_instead_of_building_empty_map(self):
        with self.assertRaises(IntegrationPaused):
            apply_target_gate(np.zeros_like(self.depth), "NO_TARGET")


if __name__ == "__main__":
    unittest.main()
