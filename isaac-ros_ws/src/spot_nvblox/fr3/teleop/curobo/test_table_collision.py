import unittest

from table_collision import build_world_config_data


PROPOSED_MESH_BOUNDS = [0.25, 0.95, -0.55, 0.60, 0.0294, 0.10]


class TableCollisionWorldTest(unittest.TestCase):
    def test_default_world_is_esdf_only(self):
        world = build_world_config_data([1.4, 1.6, 1.2], 0.02)
        self.assertIn("voxel", world)
        self.assertNotIn("cuboid", world)

    def test_table_overlay_keeps_esdf_and_covers_drill_pocket(self):
        world = build_world_config_data([1.4, 1.6, 1.2], 0.02, PROPOSED_MESH_BOUNDS)
        self.assertIn("voxel", world)
        self.assertIn("cuboid", world)
        table = world["cuboid"]["fr3_table"]
        self.assertEqual(table["dims"], [0.70, 1.15, 0.10])
        self.assertAlmostEqual(table["pose"][0], 0.60)
        self.assertAlmostEqual(table["pose"][1], 0.025)
        self.assertAlmostEqual(table["pose"][2] + table["dims"][2] / 2, 0.0294)
        self.assertTrue(0.472 >= 0.60 - 0.35 and 0.472 <= 0.60 + 0.35)
        self.assertTrue(-0.18 >= 0.025 - 0.575 and -0.18 <= 0.025 + 0.575)

    def test_rejects_nonfinite_or_invalid_bounds(self):
        for invalid in ([0.2, 0.1, -0.5, 0.5, 0.03, 0.1],
                        [0.1, 0.5, 0.2, -0.2, 0.03, 0.1],
                        [0.1, 0.5, -0.5, 0.5, float("nan"), 0.1]):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                build_world_config_data([1.4, 1.6, 1.2], 0.02, invalid)

    def test_rejects_invalid_voxel_dimensions(self):
        with self.assertRaises(ValueError):
            build_world_config_data([1.4, 0, 1.2], 0.02)
        with self.assertRaises(ValueError):
            build_world_config_data([1.4, 1.6, 1.2], 0)


if __name__ == "__main__":
    unittest.main()
