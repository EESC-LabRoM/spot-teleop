import unittest

import numpy as np

from spot_operation_ros2.object_exclusion_union import (
    MaskRejected, ObjectMaskSpec, union_object_masks,
)


class TestObjectMaskUnion(unittest.TestCase):
    def setUp(self):
        self.shape = (100, 200)
        self.specs = [
            ObjectMaskSpec("a", (.05, .1, .25, .4), .001, .1),
            ObjectMaskSpec("b", (.35, .1, .55, .4), .001, .1),
        ]
        self.masks = {}
        self.masks["a"] = np.zeros(self.shape, np.uint8)
        self.masks["a"][20:30, 20:30] = 1
        self.masks["b"] = np.zeros(self.shape, np.uint8)
        self.masks["b"][20:30, 90:100] = 1

    def test_union_keeps_both_ids_and_pixels(self):
        union, stats = union_object_masks(self.masks, self.specs, self.shape)
        self.assertEqual(set(np.unique(union)), {0, 255})
        self.assertEqual(int(np.count_nonzero(union)), 200)
        self.assertEqual(stats["_union"]["pixels"], 200)

    def test_missing_object_fails_closed(self):
        with self.assertRaisesRegex(MaskRejected, "missing"):
            union_object_masks({"a": self.masks["a"]}, self.specs, self.shape)

    def test_oversized_mask_fails_closed(self):
        masks = dict(self.masks)
        masks["a"] = np.ones(self.shape, np.uint8)
        with self.assertRaisesRegex(MaskRejected, "area fraction"):
            union_object_masks(masks, self.specs, self.shape)

    def test_mask_outside_prompt_roi_fails_closed(self):
        masks = dict(self.masks)
        masks["a"] = np.zeros(self.shape, np.uint8)
        masks["a"][70:80, 20:30] = 1
        with self.assertRaisesRegex(MaskRejected, "centroid escaped"):
            union_object_masks(masks, self.specs, self.shape)

    def test_two_ids_collapsing_onto_same_region_fails_closed(self):
        masks = dict(self.masks)
        masks["b"] = masks["a"].copy()
        overlap_specs = [
            self.specs[0],
            ObjectMaskSpec("b", self.specs[0].box_norm, .001, .1),
        ]
        with self.assertRaisesRegex(MaskRejected, "ambiguous overlap"):
            union_object_masks(masks, overlap_specs, self.shape)


if __name__ == "__main__":
    unittest.main()
