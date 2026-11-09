#!/usr/bin/env python3
"""Focused tests for the clean control-source switch (no ROS needed).

Run: python3 -m unittest discover -s tests -v   (from zed_wrist_switch/)
Covers: switch gating, bridge endpoint mapping, end-to-end client flow
against a fake :8784 server.
"""
import json
import threading
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from source_switch import SOURCES, decide_switch, verify_settled, SwitchClient, SwitchError
from bridge_source_endpoint import handle_source_post


class DecideSwitchTest(unittest.TestCase):
    def test_unknown_source_rejected(self):
        ok, msg = decide_switch('gamepad', False, 'joystick')
        self.assertFalse(ok)
        self.assertIn('unknown source', msg)

    def test_engaged_refused(self):
        ok, msg = decide_switch('zed', True, 'joystick')
        self.assertFalse(ok)
        self.assertIn('disengage', msg)

    def test_same_source_noop_success(self):
        ok, msg = decide_switch('joystick', False, 'joystick')
        self.assertTrue(ok)
        self.assertTrue(msg.startswith('already on'))

    def test_clean_switch_allowed(self):
        ok, msg = decide_switch('zed', False, 'joystick')
        self.assertTrue(ok)
        self.assertIn('joystick -> zed', msg)


class VerifySettledTest(unittest.TestCase):
    def test_landed_joystick(self):
        ok, msg = verify_settled({'source': 'joystick', 'reason': 'waiting for joystick Start'}, 'joystick')
        self.assertTrue(ok)

    def test_zed_without_tracker_is_landed_hold(self):
        # Param set alone never yields motion: tracker inactive -> hold.
        ok, msg = verify_settled({'source': 'zed', 'reason': 'ZED paused'}, 'zed')
        self.assertTrue(ok)
        self.assertIn('no motion', msg)

    def test_wrong_source_fails(self):
        ok, msg = verify_settled({'source': 'joystick', 'reason': 'active'}, 'zed')
        self.assertFalse(ok)


class EndpointTest(unittest.TestCase):
    def test_malformed_body_400(self):
        code, resp = handle_source_post('{bad json', False, 'joystick',
                                        lambda: (True, 'x'), lambda: (True, 'x'))
        self.assertEqual(code, 400)
        self.assertFalse(resp['ok'])

    def test_engaged_409_no_side_effect(self):
        called = []
        code, resp = handle_source_post({'source': 'zed'}, True, 'joystick',
                                        lambda: (called.append(1), (True, 'x'))[1],
                                        lambda: (True, 'x'))
        self.assertEqual(code, 409)
        self.assertEqual(called, [])

    def test_zed_uses_calibration_path(self):
        calls = []
        code, resp = handle_source_post({'source': 'zed'}, False, 'joystick',
                                        lambda: (calls.append('zed_start'), (True, 'calibrated'))[1],
                                        lambda: (_ for _ in ()).throw(AssertionError('param path must not run for zed')))
        self.assertEqual(code, 200)
        self.assertEqual(calls, ['zed_start'])
        self.assertIn('calibrated', resp['message'])

    def test_joystick_uses_param_path(self):
        calls = []
        code, resp = handle_source_post({'source': 'joystick'}, False, 'zed',
                                        lambda: (_ for _ in ()).throw(AssertionError('trigger path must not run for joystick')),
                                        lambda: (calls.append('param'), (True, 'set'))[1])
        self.assertEqual(code, 200)
        self.assertEqual(calls, ['param'])

    def test_activation_failure_fail_closed(self):
        code, resp = handle_source_post({'source': 'zed'}, False, 'joystick',
                                        lambda: (False, 'service unavailable'),
                                        lambda: (True, 'x'))
        self.assertEqual(code, 502)
        self.assertEqual(resp['source'], 'joystick')  # still reports old source


class FakeBridgeHandler(BaseHTTPRequestHandler):
    state = {'source': 'joystick', 'engaged': False, 'reason': 'waiting for joystick Start'}

    def log_message(self, *args):
        pass

    def _send(self, code, obj):
        data = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path == '/status':
            self._send(200, dict(self.state))
        else:
            self._send(404, {})

    def do_POST(self):
        if self.path != '/source':
            self._send(404, {'ok': False})
            return
        body = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
        code, resp = handle_source_post(
            body, self.state['engaged'], self.state['source'],
            lambda: (True, 'calibrated'),
            lambda: (True, 'set'))
        if code == 200:
            self.state['source'] = resp['source']
            self.state['reason'] = ('ZED paused' if resp['source'] == 'zed'
                                    else 'waiting for joystick Start')
        self._send(code, resp)


class ClientFlowTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.server = ThreadingHTTPServer(('127.0.0.1', 0), FakeBridgeHandler)
        cls.port = cls.server.server_address[1]
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()

    def setUp(self):
        FakeBridgeHandler.state = {'source': 'joystick', 'engaged': False,
                                   'reason': 'waiting for joystick Start'}

    def client(self):
        return SwitchClient('http://127.0.0.1:%d' % self.port, timeout=3.0)

    def test_full_switch_to_zed_lands_in_hold(self):
        out = self.client().switch('zed')
        self.assertTrue(out['ok'])
        self.assertEqual(out['status']['source'], 'zed')
        self.assertIn('no motion', out['message'])

    def test_switch_back_to_joystick(self):
        self.client().switch('zed')
        out = self.client().switch('joystick')
        self.assertTrue(out['ok'])
        self.assertEqual(out['status']['source'], 'joystick')

    def test_engaged_switch_refused(self):
        FakeBridgeHandler.state['engaged'] = True
        with self.assertRaises(SwitchError):
            self.client().switch('zed')
        self.assertEqual(FakeBridgeHandler.state['source'], 'joystick')


if __name__ == '__main__':
    unittest.main()
