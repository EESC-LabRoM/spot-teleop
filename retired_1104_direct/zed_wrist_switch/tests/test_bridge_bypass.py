#!/usr/bin/env python3
"""Tests for the bridge bypass flag (no ROS needed).

Run from zed_wrist_switch/: python3 -m unittest discover -s tests -v
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from bridge_bypass import (
    BYPASS_REASON,
    bridge_publish_allowed,
    decide_bypass_change,
    parse_bypass,
)


def green(**kw):
    base = dict(bypass=False, armed=True, engaged_now=True,
                controller_ready=True, packet_valid=True, source_ok=True)
    base.update(kw)
    return bridge_publish_allowed(**base)


class ParseBypassTest(unittest.TestCase):
    def test_truthy_strings(self):
        for v in ('1', 'true', 'TRUE', 'yes', 'on', True):
            self.assertTrue(parse_bypass(v), v)

    def test_falsy_and_unknown_fail_closed(self):
        for v in ('0', 'false', '', 'maybe', None, False):
            self.assertFalse(parse_bypass(v), v)


class DecideBypassChangeTest(unittest.TestCase):
    def test_refuses_while_engaged(self):
        ok, msg = decide_bypass_change(True, False, True)
        self.assertFalse(ok)
        self.assertIn('disengage', msg)

    def test_noop_same_value(self):
        ok, msg = decide_bypass_change(False, False, False)
        self.assertTrue(ok)
        self.assertIn('no-op', msg)

    def test_enable_and_disable_messages(self):
        ok, msg = decide_bypass_change(True, False, False)
        self.assertTrue(ok)
        self.assertIn('monitor-only', msg)
        ok, msg = decide_bypass_change(False, True, False)
        self.assertTrue(ok)
        self.assertIn('resume publishing', msg)

    def test_rejects_non_bool(self):
        ok, _ = decide_bypass_change('yes', False, False)
        self.assertFalse(ok)


class PublishGateTest(unittest.TestCase):
    def test_normal_path_publishes(self):
        allowed, _ = green()
        self.assertTrue(allowed)

    def test_bypass_forces_monitor_only(self):
        allowed, note = green(bypass=True)
        self.assertFalse(allowed)
        self.assertEqual(note, BYPASS_REASON)

    def test_bypass_blocks_even_all_green(self):
        # Bypass wins over every other gate: the point of the A/B test.
        allowed, _ = green(bypass=True, armed=True, engaged_now=True,
                           controller_ready=True, packet_valid=True)
        self.assertFalse(allowed)

    def test_bridged_still_gates_disarmed(self):
        allowed, note = green(armed=False)
        self.assertFalse(allowed)
        self.assertIn('disarmed', note)

    def test_bridged_still_gates_bad_packet(self):
        allowed, _ = green(packet_valid=False)
        self.assertFalse(allowed)

    def test_bridged_still_gates_source_hold(self):
        allowed, note = green(source_ok=False)
        self.assertFalse(allowed)
        self.assertIn('ZED paused', note)


if __name__ == '__main__':
    unittest.main()
