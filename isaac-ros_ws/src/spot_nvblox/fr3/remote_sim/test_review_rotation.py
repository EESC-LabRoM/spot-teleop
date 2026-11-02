import math
import unittest
import numpy as np
from review_rotation import hand_angle, tool_angle, direction
from teleop_mapping import _quat_mul


class SignedRotationTests(unittest.TestCase):
    def test_both_signs_with_tilted_anchor_and_quaternion_sign_flip(self):
        anchor = np.array([.2, .1, .3, math.sqrt(.86)])
        for profile, hand_axis, tool_axis in [('pitch', 1, 1), ('yaw', 2, 0)]:
            for degrees in [-60, -15, 15, 60]:
                for sign in [-1, 1]:
                    half = math.radians(degrees) / 2
                    h = np.r_[np.eye(3)[hand_axis] * math.sin(half), math.cos(half)]
                    t = np.r_[np.eye(3)[tool_axis] * math.sin(half), math.cos(half)]
                    self.assertAlmostEqual(hand_angle(anchor, sign * _quat_mul(h, anchor), profile), degrees)
                    self.assertAlmostEqual(tool_angle(anchor, sign * _quat_mul(anchor, t), profile), degrees)

    def test_labels_distinguish_neutral_and_both_directions(self):
        self.assertEqual(direction(-15), 'NEGATIVO')
        self.assertEqual(direction(15), 'POSITIVO')
        self.assertEqual(direction(1), 'NEUTRO')


if __name__ == '__main__':
    unittest.main()
