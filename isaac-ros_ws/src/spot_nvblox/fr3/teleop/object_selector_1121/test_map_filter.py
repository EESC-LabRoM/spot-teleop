import unittest

import numpy as np

from map_filter import Identity, MaskFrame, Selection, Tracking, route_depth


class RouteDepthTests(unittest.TestCase):
    def setUp(self):
        self.depth = np.full((100, 100), 1.25, np.float32)
        self.mask = np.zeros((100, 100), np.uint8)
        self.mask[40:50, 40:50] = 255
        self.identity = Identity("tennis_ball", 2, "selection-uuid")
        self.selection = Selection(self.identity, stamp=10.0, selected_at=9.1)
        self.tracking = Tracking(self.identity, "TRACKING", stamp=9.8)
        self.mask_frame = MaskFrame(self.identity, stamp=9.9, pixels=self.mask)

    def route(self, **kwargs):
        args = dict(depth=self.depth, depth_stamp=9.9, now=10.0,
                    selection=self.selection, tracking=self.tracking,
                    mask=self.mask_frame, dilate_px=0)
        args.update(kwargs)
        return route_depth(**args)

    def test_explicit_fresh_no_selection_keeps_full_scene(self):
        result = self.route(selection=Selection(None, stamp=10.0, selected_at=None),
                            tracking=Tracking(None, "LOST", 10.0), mask=None)
        self.assertEqual(result.action, "full")
        np.testing.assert_array_equal(result.depth, self.depth)
        self.assertIsNone(result.excluded)

    def test_stale_no_selection_heartbeat_pauses_instead_of_leaking(self):
        result = self.route(selection=Selection(None, stamp=8.0, selected_at=None),
                            tracking=None, mask=None)
        self.assertEqual(result.action, "pause")
        self.assertIn("Selection state is stale", result.reason)

    def test_matching_fresh_target_mask_carves_only_target_pixels(self):
        result = self.route()
        self.assertEqual(result.action, "carved")
        self.assertTrue(np.all(result.depth[self.mask > 0] == 0))
        self.assertTrue(np.all(result.depth[self.mask == 0] == self.depth[self.mask == 0]))

    def test_lost_selected_target_pauses(self):
        result = self.route(tracking=Tracking(self.identity, "LOST", 9.8))
        self.assertEqual(result.action, "pause")
        self.assertIn("tracker state", result.reason)

    def test_old_generation_mask_pauses(self):
        old = Identity("drill", 1, "old-id")
        result = self.route(mask=MaskFrame(old, 9.9, self.mask))
        self.assertEqual(result.action, "pause")
        self.assertIn("identity differs", result.reason)

    def test_mask_from_before_selection_pauses(self):
        result = self.route(selection=Selection(self.identity, 10.0, 9.95),
                            tracking=Tracking(self.identity, "TRACKING", 9.8),
                            mask=MaskFrame(self.identity, 9.9, self.mask))
        self.assertEqual(result.action, "pause")
        self.assertIn("predates selection", result.reason)

    def test_unmatched_depth_and_mask_pause(self):
        result = self.route(mask=MaskFrame(self.identity, 9.8, self.mask))
        self.assertEqual(result.action, "pause")
        self.assertIn("not source-stamp matched", result.reason)

    def test_empty_and_oversized_mask_pause(self):
        empty = self.route(mask=MaskFrame(self.identity, 9.9, np.zeros_like(self.mask)))
        full = self.route(mask=MaskFrame(self.identity, 9.9, np.ones_like(self.mask)))
        self.assertEqual(empty.action, "pause")
        self.assertEqual(full.action, "pause")

    def test_stale_selection_identity_pauses_even_with_recent_mask(self):
        result = self.route(selection=Selection(self.identity, 8.0, 7.9))
        self.assertEqual(result.action, "pause")
        self.assertIn("Selection state is stale", result.reason)


if __name__ == "__main__":
    unittest.main()
