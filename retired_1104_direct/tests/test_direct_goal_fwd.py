#!/usr/bin/env python3
"""Focused tests for the direct goal-relay gate (no ROS needed).

Covers the exact failure seen live 2026-11-04: with the bridge retired the
planner received zero wrist goals, so the stick did nothing while the
replay path (which publishes wrist_goal itself) worked fine.
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from direct_goal_fwd import fresh_enough, should_forward

NOW = 1_700_000_000_000_000_000


def stamp(age_s):
    return NOW - int(age_s * 1e9)


class FreshTest(unittest.TestCase):
    def test_fresh_passes(self):
        self.assertTrue(fresh_enough(NOW, stamp(0.1)))

    def test_boundary_quarter_second(self):
        self.assertTrue(fresh_enough(NOW, stamp(0.25)))
        self.assertFalse(fresh_enough(NOW, stamp(0.26)))

    def test_future_stamp_rejected(self):
        self.assertFalse(fresh_enough(NOW, NOW + int(0.01 * 1e9)))

    def test_stale_rejected(self):
        self.assertFalse(fresh_enough(NOW, stamp(5.0)))


class GateTest(unittest.TestCase):
    def test_forwards_when_engaged_fresh_right_frame(self):
        ok, _ = should_forward(True, 'fr3_link0', NOW, stamp(0.05))
        self.assertTrue(ok)

    def test_refuses_while_disengaged(self):
        # The live symptom: engaged False -> silence, planner holds.
        ok, msg = should_forward(False, 'fr3_link0', NOW, stamp(0.05))
        self.assertFalse(ok)
        self.assertIn('Start', msg)

    def test_refuses_wrong_frame(self):
        ok, _ = should_forward(True, 'fr3_hand_tcp', NOW, stamp(0.05))
        self.assertFalse(ok)

    def test_refuses_stale_command(self):
        ok, _ = should_forward(True, 'fr3_link0', NOW, stamp(1.0))
        self.assertFalse(ok)

    def test_ros_epoch_scale_contract(self):
        # REGRESSION (2026-11-04): main() once passed a monotonic-clock
        # now_ns (~1.1e15) against ROS-epoch stamps (~1.79e18), so the gate
        # held forever. The caller MUST pass ROS-clock ns; pin both scales.
        ros_now = 1793834643839449251
        ok, _ = should_forward(True, 'fr3_link0', ros_now, ros_now - 10_000_000)
        self.assertTrue(ok)
        mono_now = 1_100_000_000_000_000
        ok, _ = should_forward(True, 'fr3_link0', mono_now, ros_now - 10_000_000)
        self.assertFalse(ok)


if __name__ == '__main__':
    unittest.main()
