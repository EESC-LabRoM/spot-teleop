import math
import unittest
import numpy as np
from teleop_mapping import map_target, mapping_settings, palm_normal, rotation_delta, _quat_mul


CAMERA_TO_ROBOT = np.array([[0., 0., -1.], [1., 0., 0.], [0., -1., 0.]])
HAND_TO_FR3_TCP = np.array([[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]])


def axis_angle_quat(axis, angle_rad):
    axis = np.asarray(axis, float)
    axis = axis / np.linalg.norm(axis)
    half = 0.5 * angle_rad
    return np.r_[axis * math.sin(half), math.cos(half)]


def matrix_to_quat(r):
    q = np.empty(4)
    tr = float(np.trace(r))
    if tr > 0.0:
        s = math.sqrt(1.0 + tr) * 2.0
        q[:] = [(r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s, 0.25 * s]
    else:
        i = int(np.argmax(np.diag(r)))
        j = (i + 1) % 3
        k = (i + 2) % 3
        s = math.sqrt(max(0.0, 1.0 + r[i, i] - r[j, j] - r[k, k])) * 2.0
        q[i] = 0.25 * s
        q[j] = (r[j, i] + r[i, j]) / s
        q[k] = (r[k, i] + r[i, k]) / s
        q[3] = (r[k, j] - r[j, k]) / s
    return q / np.linalg.norm(q)


class TeleopMappingTests(unittest.TestCase):
    def setUp(self):
        self.anchor_p = np.array([0.40, 0.30, 0.50])
        self.anchor_q = axis_angle_quat([1.0, 0.0, 0.0], math.pi)  # tool pointing down
        self.hand_p0 = np.array([0.10, -0.05, 0.20])
        # Palm facing camera: palm +x up (+Z), palm +y left (-Y), palm +z toward camera (+X).
        r_palm_robot = np.column_stack(([0., 0., 1.], [0., -1., 0.], [1., 0., 0.]))
        self.hand_q0 = matrix_to_quat(r_palm_robot @ HAND_TO_FR3_TCP)

    def test_zero_hand_motion_returns_anchor_exactly(self):
        p_t, q_t = map_target(
            self.anchor_p, self.anchor_q,
            self.hand_p0, self.hand_q0,
            self.hand_p0.copy(), self.hand_q0.copy(),
            [1.0, 1.0, 1.0], 1.0, 0.0,
        )
        np.testing.assert_allclose(p_t, self.anchor_p, atol=1e-12)
        np.testing.assert_allclose(q_t, self.anchor_q, atol=1e-12)

    def test_hand_toward_camera_lowers_tcp_z(self):
        hand_p = self.hand_p0 + np.array([0.10, 0.0, 0.0])
        p_t, q_t = map_target(
            self.anchor_p, self.anchor_q,
            self.hand_p0, self.hand_q0,
            hand_p, self.hand_q0,
            [1.0, 1.0, 1.0], 1.0, 0.0,
        )
        np.testing.assert_allclose(p_t, [0.40, 0.30, 0.40], atol=1e-12)
        np.testing.assert_allclose(q_t, self.anchor_q, atol=1e-12)

    def test_hand_operator_right_yaws_base_clockwise_preserving_radius(self):
        r0 = math.hypot(self.anchor_p[0], self.anchor_p[1])  # 0.50
        theta0 = math.atan2(self.anchor_p[1], self.anchor_p[0])
        hand_p = self.hand_p0 + np.array([0.0, -0.10, 0.0])
        p_t, _ = map_target(
            self.anchor_p, self.anchor_q,
            self.hand_p0, self.hand_q0,
            hand_p, self.hand_q0,
            [1.0, 1.0, 1.0], 1.0, 0.0,
        )
        r_target = math.hypot(p_t[0], p_t[1])
        theta_target = math.atan2(p_t[1], p_t[0])
        self.assertAlmostEqual(r_target, r0, places=12)
        self.assertAlmostEqual(theta_target - theta0, -0.10 / r0, places=12)
        self.assertAlmostEqual(p_t[2], self.anchor_p[2], places=12)

    def test_hand_up_extends_reach_radially_and_clamps(self):
        r0 = math.hypot(self.anchor_p[0], self.anchor_p[1])  # 0.50
        theta0 = math.atan2(self.anchor_p[1], self.anchor_p[0])
        hand_p = self.hand_p0 + np.array([0.0, 0.0, 0.05])
        p_t, _ = map_target(
            self.anchor_p, self.anchor_q,
            self.hand_p0, self.hand_q0,
            hand_p, self.hand_q0,
            [1.0, 1.0, 1.0], 1.0, 0.0,
        )
        self.assertAlmostEqual(math.hypot(p_t[0], p_t[1]), r0 + 0.05, places=12)
        self.assertAlmostEqual(math.atan2(p_t[1], p_t[0]), theta0, places=12)
        self.assertAlmostEqual(p_t[2], self.anchor_p[2], places=12)

        # Clamp upper bound at 0.80 m
        p_hi, _ = map_target(
            self.anchor_p, self.anchor_q,
            self.hand_p0, self.hand_q0,
            self.hand_p0 + np.array([0.0, 0.0, 0.50]), self.hand_q0,
            [1.0, 1.0, 1.0], 1.0, 0.0,
        )
        self.assertAlmostEqual(math.hypot(p_hi[0], p_hi[1]), 0.80, places=12)

        # Clamp lower bound at 0.15 m
        p_lo, _ = map_target(
            self.anchor_p, self.anchor_q,
            self.hand_p0, self.hand_q0,
            self.hand_p0 + np.array([0.0, 0.0, -0.60]), self.hand_q0,
            [1.0, 1.0, 1.0], 1.0, 0.0,
        )
        self.assertAlmostEqual(math.hypot(p_lo[0], p_lo[1]), 0.15, places=12)

    def test_zero_gains_freeze_corresponding_axes(self):
        hand_p = self.hand_p0 + np.array([0.12, -0.15, 0.08])
        # Freeze reach (gx=0) and lift (gz=0), allow only arc (gy=1)
        p_arc, _ = map_target(
            self.anchor_p, self.anchor_q,
            self.hand_p0, self.hand_q0,
            hand_p, self.hand_q0,
            [0.0, 1.0, 0.0], 1.0, 0.0,
        )
        self.assertAlmostEqual(math.hypot(p_arc[0], p_arc[1]), 0.50, places=12)
        self.assertAlmostEqual(p_arc[2], self.anchor_p[2], places=12)

        # Freeze all three position axes
        p_frozen, _ = map_target(
            self.anchor_p, self.anchor_q,
            self.hand_p0, self.hand_q0,
            hand_p, self.hand_q0,
            [0.0, 0.0, 0.0], 1.0, 0.0,
        )
        np.testing.assert_allclose(p_frozen, self.anchor_p, atol=1e-12)

    def test_palm_normal_extracted_from_tracker_quaternion(self):
        # Build R_palm in camera optical frame so that CAMERA_TO_ROBOT @ R_palm has palm normal toward -X.
        # In camera frame: +Z_cam maps to -X_robot via CAMERA_TO_ROBOT.
        # Set palm x = [0, -1, 0]_cam (+Z_robot), palm y = [1, 0, 0]_cam (+Y_robot), palm z = [0, 0, 1]_cam (-X_robot).
        r_palm_cam = np.column_stack(([0., -1., 0.], [1., 0., 0.], [0., 0., 1.]))
        r_palm_robot = CAMERA_TO_ROBOT @ r_palm_cam
        np.testing.assert_allclose(r_palm_robot[:, 2], [-1.0, 0.0, 0.0], atol=1e-12)
        q = matrix_to_quat(r_palm_robot @ HAND_TO_FR3_TCP)
        np.testing.assert_allclose(palm_normal(q), [-1.0, 0.0, 0.0], atol=1e-12)

    def test_palm_roll_about_away_from_camera_normal_rolls_tool_z(self):
        deg30 = math.radians(30.0)
        dq_roll = axis_angle_quat([-1.0, 0.0, 0.0], deg30)
        hand_q = _quat_mul(dq_roll, self.hand_q0)

        _, q_t = map_target(
            self.anchor_p, self.anchor_q,
            self.hand_p0, self.hand_q0,
            self.hand_p0, hand_q,
            [1.0, 1.0, 1.0], 1.0, 0.0,
        )
        expected_30 = _quat_mul(self.anchor_q, axis_angle_quat([0.0, 0.0, 1.0], deg30))
        np.testing.assert_allclose(q_t, expected_30, atol=1e-12)

        # roll_gain = 0.5 -> +15 degrees about tool Z
        _, q_half = map_target(
            self.anchor_p, self.anchor_q,
            self.hand_p0, self.hand_q0,
            self.hand_p0, hand_q,
            [1.0, 1.0, 1.0], 0.5, 0.0,
        )
        expected_15 = _quat_mul(self.anchor_q, axis_angle_quat([0.0, 0.0, 1.0], math.radians(15.0)))
        np.testing.assert_allclose(q_half, expected_15, atol=1e-12)

        # roll_gain = 0.0 freezes orientation
        _, q_frozen = map_target(
            self.anchor_p, self.anchor_q,
            self.hand_p0, self.hand_q0,
            self.hand_p0, hand_q,
            [1.0, 1.0, 1.0], 0.0, 0.0,
        )
        np.testing.assert_allclose(q_frozen, self.anchor_q, atol=1e-12)

    def test_pure_swing_ignored_when_tilt_gain_zero_and_applied_when_nonzero(self):
        # Axis [0, 1, 0] is perpendicular to n0 = [-1, 0, 0]
        swing_angle = math.radians(24.0)
        dq_swing = axis_angle_quat([0.0, 1.0, 0.0], swing_angle)
        hand_q = _quat_mul(dq_swing, self.hand_q0)

        _, q_no_tilt = map_target(
            self.anchor_p, self.anchor_q,
            self.hand_p0, self.hand_q0,
            self.hand_p0, hand_q,
            [1.0, 1.0, 1.0], 1.0, 0.0,
        )
        np.testing.assert_allclose(q_no_tilt, self.anchor_q, atol=1e-12)

        _, q_half_tilt = map_target(
            self.anchor_p, self.anchor_q,
            self.hand_p0, self.hand_q0,
            self.hand_p0, hand_q,
            [1.0, 1.0, 1.0], 1.0, 0.5,
        )
        expected_tilt = _quat_mul(
            rotation_delta(dq_swing, np.array([0., 0., 0., 1.]), 0.5),
            self.anchor_q,
        )
        np.testing.assert_allclose(q_half_tilt, expected_tilt, atol=1e-12)

    def test_mapping_settings_validates_two_and_three_args(self):
        gains, rot = mapping_settings([1.0, -1.0, 0.5], 1.5)
        np.testing.assert_allclose(gains, [1.0, -1.0, 0.5])
        self.assertEqual(rot, 1.5)

        gains, rot, tilt = mapping_settings([1.0, 1.0, 1.0], 1.0, 0.5)
        self.assertEqual(tilt, 0.5)

        for bad_tilt in (-0.1, 3.1, float('nan')):
            with self.assertRaises(ValueError):
                mapping_settings([1.0, 1.0, 1.0], 1.0, bad_tilt)


if __name__ == '__main__':
    unittest.main()

class GestureValidityTests(__import__('unittest').TestCase):
 def test_valid_fist_does_not_require_palm_orientation(self):
  from teleop_mapping import control_gesture
  self.assertEqual(control_gesture(dict(position_valid=True,orientation_valid=False,gesture='Closed_Fist',gesture_score=.9)),'Closed_Fist')
 def test_legacy_paired_v2_keeps_gesture_support(self):
  from teleop_mapping import control_gesture
  self.assertEqual(control_gesture(dict(version=2,gesture='Open_Palm',gesture_score=.9)),'Open_Palm')
 def test_invalid_position_uncertain_unknown_and_nonfinite_rejected(self):
  from teleop_mapping import control_gesture
  good=dict(position_valid=True,gesture='Closed_Fist',gesture_score=.9)
  for change in [dict(position_valid=False),dict(gesture_score=.5),dict(gesture='Victory'),dict(gesture_score=float('nan')),dict(gesture_score=float('inf'))]:
   self.assertIsNone(control_gesture(dict(good,**change)))
