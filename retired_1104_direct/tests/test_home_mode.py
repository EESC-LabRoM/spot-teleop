#!/usr/bin/env python3
"""Focused tests for the home controller-mode fallback (no ROS needed).

Bug fixed 2026-11-04: fr3_home_jtc.py refused whenever the position JTC
was already active ("another move running?"). On the direct/cuRobo stack
the JTC stays active by design, so every home path (pad X, dashboard
button, real-home) failed. The fix falls back to commanding through the
live controller when the impedance controller is not holding.

Imports the REAL script with stubbed ROS modules (numpy via `uv run
--with numpy`); no behavior is duplicated here.
"""
import os
import sys
import types
import unittest


def _mod(name, **attrs):
    m = types.ModuleType(name)
    for k, v in attrs.items():
        setattr(m, k, v)
    sys.modules[name] = m
    return m


class _Node:
    def __init__(self, *a, **k):
        pass


class _Q:
    pass


# Stub everything the script imports except argparse/os/signal/subprocess/
# sys/time/numpy. fr3_kin + franka_msgs already tolerate ImportError.
_mod('rclpy', Node=_Node, init=lambda *a, **k: None, shutdown=lambda: None,
     ok=lambda: True, spin_once=lambda *a, **k: None,
     spin_until_future_complete=lambda *a, **k: None, time=types.SimpleNamespace(Time=_Q))
_mod('rclpy.node', Node=_Node)
_mod('rclpy.action', ActionClient=_Q)
_mod('builtin_interfaces.msg', Duration=_Q)
_mod('control_msgs.action', FollowJointTrajectory=_Q)
_mod('controller_manager_msgs.srv', ListControllers=_Q, SwitchController=_Q)
_mod('sensor_msgs.msg', JointState=_Q)
_mod('trajectory_msgs.msg', JointTrajectoryPoint=_Q)

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import importlib.util

_spec = importlib.util.spec_from_file_location(
    'fr3_home_jtc_live',
    os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                 'fr3_home_jtc.py'))
_home = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_home)


class SelectHomeModeTest(unittest.TestCase):
    POS = 'fr3_arm_controller'
    IMP = 'cartesian_impedance_target_controller'

    def test_live_direct_stack_goes_direct(self):
        # The reported failure: JTC active (direct stack), imp not holding.
        mode, why = _home.select_home_mode({self.POS: 'active'}, self.POS, self.IMP)
        self.assertEqual(mode, 'direct')
        self.assertIn('no swap', why)

    def test_direct_when_imp_explicitly_inactive(self):
        mode, _ = _home.select_home_mode(
            {self.POS: 'active', self.IMP: 'inactive'}, self.POS, self.IMP)
        self.assertEqual(mode, 'direct')

    def test_both_active_still_refuses_with_historical_message(self):
        mode, why = _home.select_home_mode(
            {self.POS: 'active', self.IMP: 'active'}, self.POS, self.IMP)
        self.assertEqual(mode, 'refuse')
        self.assertIn('another move running?', why)

    def test_classic_swap_path_unchanged(self):
        mode, _ = _home.select_home_mode(
            {self.POS: 'inactive', self.IMP: 'active'}, self.POS, self.IMP)
        self.assertEqual(mode, 'swap')

    def test_missing_pos_still_goes_swap_loader(self):
        mode, _ = _home.select_home_mode({self.IMP: 'active'}, self.POS, self.IMP)
        self.assertEqual(mode, 'swap')


if __name__ == '__main__':
    unittest.main()
