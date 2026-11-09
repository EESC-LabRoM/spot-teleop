#!/usr/bin/env python3
"""Focused tests for the auto-rearm gate (no ROS needed).

Run: python3 -m unittest discover -s tests -v   (from zed_wrist_switch/)
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from bridge_rearm import maybe_auto_rearm


class AutoRearmTest(unittest.TestCase):
    def decide(self, **kw):
        base = dict(armed=False, fault_disarm=False, engaged_now=False,
                    engaged_prev=False, controller_ready=True,
                    packet_valid=True)
        base.update(kw)
        return maybe_auto_rearm(**base)

    def test_fresh_start_all_green_arms(self):
        d = self.decide(engaged_now=True, engaged_prev=False)
        self.assertTrue(d.armed)

    def test_stale_held_start_never_arms(self):
        # Start already held from before: no rising edge, no arm. This is
        # the offline equivalent of "disengage before enabling".
        d = self.decide(engaged_now=True, engaged_prev=True)
        self.assertFalse(d.armed)

    def test_idle_stays_disarmed(self):
        d = self.decide()
        self.assertFalse(d.armed)

    def test_fault_disarm_needs_manual_enable_despite_fresh_start(self):
        d = self.decide(fault_disarm=True, engaged_now=True,
                        engaged_prev=False)
        self.assertFalse(d.armed)
        self.assertIn('manual enable', d.note)

    def test_no_arm_without_controller(self):
        d = self.decide(engaged_now=True, engaged_prev=False,
                        controller_ready=False)
        self.assertFalse(d.armed)

    def test_no_arm_without_valid_packet(self):
        d = self.decide(engaged_now=True, engaged_prev=False,
                        packet_valid=False)
        self.assertFalse(d.armed)

    def test_already_armed_unchanged(self):
        d = self.decide(armed=True, engaged_now=True, engaged_prev=False)
        self.assertTrue(d.armed)
        self.assertEqual(d.note, 'already armed')


class PollDecisionTest(unittest.TestCase):
    def test_fresh_inflight_waits(self):
        from bridge_rearm import poll_decision
        self.assertEqual(poll_decision(True, False, 10.0, 11.0, True), 'wait')

    def test_stale_inflight_resets(self):
        from bridge_rearm import poll_decision
        self.assertEqual(poll_decision(True, False, 10.0, 13.0, True), 'reset')
        self.assertEqual(poll_decision(True, False, 10.0, 13.0, False), 'reset')

    def test_done_or_idle_calls_when_ready(self):
        from bridge_rearm import poll_decision
        self.assertEqual(poll_decision(False, False, 0.0, 99.0, True), 'call')
        self.assertEqual(poll_decision(True, True, 10.0, 99.0, True), 'call')

    def test_idle_waits_when_service_down(self):
        from bridge_rearm import poll_decision
        self.assertEqual(poll_decision(False, False, 0.0, 99.0, False), 'wait')


if __name__ == '__main__':
    unittest.main()
