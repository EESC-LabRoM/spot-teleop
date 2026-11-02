import math
import unittest
import numpy as np

from make_clips import find_segments, label_motion, palm_roll_deg, tool_roll_deg
from teleop_mapping import _quat_mul, map_target


def axis_angle_quat(axis, angle_rad):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    half = 0.5 * angle_rad
    return np.r_[axis * math.sin(half), math.cos(half)]


class TestClips(unittest.TestCase):
    def setUp(self):
        self.anchor_p = np.array([0.30689, 0.0, 0.48688], dtype=float)
        self.anchor_q = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        self.p0 = np.array([0.10, -0.05, 0.20], dtype=float)
        self.q0 = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)

    def _build_series(self, delta_p=None, roll_axis=None, roll_deg=0.0, dt=0.05):
        delta_p = np.zeros(3, dtype=float) if delta_p is None else np.asarray(delta_p, dtype=float)
        ts = np.arange(0.0, 4.0 + 1e-9, dt)
        samples = []
        for t in ts:
            if t <= 1.0:
                frac = 0.0
            elif t <= 3.0:
                frac = (t - 1.0) / 2.0
            else:
                frac = 1.0
            p = self.p0 + frac * delta_p
            if roll_axis is not None and abs(roll_deg) > 0.0:
                dq = axis_angle_quat(roll_axis, math.radians(roll_deg * frac))
                q = _quat_mul(dq, self.q0)
            else:
                q = self.q0.copy()
            samples.append({"t": float(t), "p": p, "q": q})
        return samples

    def test_toward_camera_plus_10cm_x(self):
        samples = self._build_series(delta_p=[0.10, 0.0, 0.0])
        segs = find_segments(samples)
        self.assertEqual(len(segs), 1)
        seg = segs[0]
        self.assertEqual(seg["label"], "toward camera")
        self.assertAlmostEqual(seg["dx_cm"], 10.0, delta=0.5)
        self.assertAlmostEqual(seg["dy_cm"], 0.0, delta=0.5)
        self.assertAlmostEqual(seg["dz_cm"], 0.0, delta=0.5)
        self.assertAlmostEqual(seg["duration"], 2.0, delta=0.15)

    def test_operator_right_minus_10cm_y(self):
        samples = self._build_series(delta_p=[0.0, -0.10, 0.0])
        segs = find_segments(samples)
        self.assertEqual(len(segs), 1)
        seg = segs[0]
        self.assertEqual(seg["label"], "operator right (-Y)")
        self.assertAlmostEqual(seg["dy_cm"], -10.0, delta=0.5)
        self.assertAlmostEqual(seg["dx_cm"], 0.0, delta=0.5)
        self.assertAlmostEqual(seg["dz_cm"], 0.0, delta=0.5)

    def test_hand_up_plus_10cm_z(self):
        samples = self._build_series(delta_p=[0.0, 0.0, 0.10])
        segs = find_segments(samples)
        self.assertEqual(len(segs), 1)
        seg = segs[0]
        self.assertEqual(seg["label"], "hand up")
        self.assertAlmostEqual(seg["dz_cm"], 10.0, delta=0.5)
        self.assertAlmostEqual(seg["dx_cm"], 0.0, delta=0.5)
        self.assertAlmostEqual(seg["dy_cm"], 0.0, delta=0.5)

    def test_roll_30_deg_about_camera_forward_matches_map_target(self):
        # Camera-forward axis in packet frame is [-1, 0, 0]
        samples = self._build_series(roll_axis=[-1.0, 0.0, 0.0], roll_deg=30.0)
        segs = find_segments(samples)
        self.assertEqual(len(segs), 1)
        seg = segs[0]
        self.assertEqual(seg["label"], "roll cw")

        # Verify against map_target with the start-of-motion and end-of-motion samples
        s_start = [s for s in samples if abs(s["t"] - 1.0) < 1e-6][0]
        s_end = [s for s in samples if abs(s["t"] - 3.0) < 1e-6][0]
        _, q_target = map_target(
            self.anchor_p,
            self.anchor_q,
            s_start["p"],
            s_start["q"],
            s_end["p"],
            s_end["q"],
            [1.0, 1.0, 1.0],
            1.0,
            0.0,
        )
        expected_tool_roll = tool_roll_deg(self.anchor_q, q_target)
        computed_palm_roll = palm_roll_deg(s_start["q"], s_end["q"])
        self.assertAlmostEqual(computed_palm_roll, expected_tool_roll, places=6)
        self.assertAlmostEqual(seg["roll_deg"], expected_tool_roll, delta=1.5)
        self.assertAlmostEqual(seg["roll_deg"], 30.0, delta=1.5)
        self.assertGreater(seg["roll_deg"] * expected_tool_roll, 0.0)

    def test_series_with_no_pause_yields_no_segment(self):
        dt = 0.05
        ts = np.arange(0.0, 4.0 + 1e-9, dt)
        # Continuous motion at 6 cm/s without any pause
        samples = [
            {"t": float(t), "p": self.p0 + np.array([0.06 * t, 0.0, 0.0]), "q": self.q0.copy()}
            for t in ts
        ]
        segs = find_segments(samples)
        self.assertEqual(segs, [])

    def test_mixed_and_remaining_labels(self):
        self.assertEqual(label_motion(-10.0, 0.0, 0.0, 0.0), "away from camera")
        self.assertEqual(label_motion(0.0, 10.0, 0.0, 0.0), "operator left (+Y)")
        self.assertEqual(label_motion(0.0, 0.0, -10.0, 0.0), "hand down")
        self.assertEqual(label_motion(0.0, 0.0, 0.0, -45.0), "roll ccw")
        self.assertEqual(label_motion(10.0, 8.0, 0.0, 0.0), "mixed")


if __name__ == "__main__":
    unittest.main()
