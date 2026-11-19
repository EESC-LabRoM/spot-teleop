import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from drill_depth_filter import IntegrationPaused, apply_target_gate, scale_depth
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from state_logic import resolve_map_state


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


    def test_depth_scale_changes_only_valid_positive_ranges(self):
        depth = np.array([[0.0, np.nan, 1.2], [0.25, -1.0, np.inf]], dtype=np.float32)
        scaled = scale_depth(depth, 0.9755371809)
        self.assertEqual(scaled[0, 0], 0.0)
        self.assertTrue(np.isnan(scaled[0, 1]))
        self.assertAlmostEqual(float(scaled[0, 2]), 1.2 * 0.9755371809, places=6)
        self.assertAlmostEqual(float(scaled[1, 0]), 0.25 * 0.9755371809, places=6)
        self.assertEqual(scaled[1, 1], -1.0)
        self.assertTrue(np.isinf(scaled[1, 2]))

    def test_invalid_depth_scale_refused(self):
        for scale in (0.0, -1.0, float("nan"), float("inf")):
            with self.subTest(scale=scale), self.assertRaises(ValueError):
                scale_depth(self.depth, scale)

    def test_target_selection_and_tracker_state_are_distinct(self):
        self.assertEqual(resolve_map_state("", "LOST", 0.0), "NO_TARGET")
        self.assertEqual(resolve_map_state("drill", "TRACKING", 0.2), "ACTIVE:drill")
        self.assertEqual(resolve_map_state("drill", "LOST", 0.2), "LOST:drill")
        self.assertEqual(resolve_map_state("drill", "TRACKING", 4.0), "LOST:drill")
        self.assertEqual(resolve_map_state("drill", "UNKNOWN", 0.2), "UNKNOWN:UNKNOWN")


if __name__ == "__main__":
    unittest.main()
