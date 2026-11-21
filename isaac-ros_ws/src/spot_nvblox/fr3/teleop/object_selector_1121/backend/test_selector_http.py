import json
import threading
import unittest
import urllib.error
import urllib.request

from selector_backend import ObjectSelector
from selector_http import SelectorService, make_server, teleop_gate


class Clock:
    t = 1000.0
    def __call__(self): return self.t


def fresh(clock, **kw):
    s = dict(engaged=False, busy=False, held=False, stamp=clock.t)
    s.update(kw)
    return s


class GateUnit(unittest.TestCase):
    def test_teleop_gate_fail_closed(self):
        now = 10.0
        ok = dict(engaged=False, busy=False, held=False, stamp=now)
        self.assertTrue(teleop_gate(ok, now)[0])
        bad = [None, {}, 'x', dict(ok, engaged=None), dict(ok, busy=0), dict(ok, held='false'),
               dict(ok, stamp=None), dict(ok, stamp=True), dict(ok, stamp=now - 1.1),
               dict(ok, stamp=now + 0.5), dict(ok, engaged=True), dict(ok, busy=True),
               dict(ok, held=True)]
        for b in bad:
            self.assertFalse(teleop_gate(b, now)[0], b)


class Http(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.state = fresh(self.clock)
        self.calls = 0
        def provider():
            self.calls += 1
            if isinstance(self.state, Exception):
                raise self.state
            return self.state
        self.svc = SelectorService(ObjectSelector(self.clock), provider, self.clock)
        self.srv = make_server(self.svc)
        self.base = f'http://127.0.0.1:{self.srv.server_address[1]}/object-target'
        threading.Thread(target=self.srv.serve_forever, daemon=True).start()

    def tearDown(self):
        self.srv.shutdown()
        self.srv.server_close()

    def req(self, method='GET', body=None, raw=None, path=''):
        data = raw if raw is not None else (json.dumps(body).encode() if body is not None else None)
        r = urllib.request.Request(self.base + path, data=data, method=method,
                                   headers={'Content-Type': 'application/json'})
        try:
            with urllib.request.urlopen(r, timeout=5) as resp:
                return resp.status, json.loads(resp.read())
        except urllib.error.HTTPError as e:
            return e.code, json.loads(e.read())

    def test_get_contract_idle(self):
        code, b = self.req()
        self.assertEqual(code, 200)
        for k in ('selected', 'generation', 'status', 'reason', 'mask', 'map', 'grasp',
                  'can_select', 'ready'):
            self.assertIn(k, b)
        self.assertEqual((b['selected'], b['status'], b['ready'], b['can_select']),
                         (None, 'idle', False, True))

    def test_post_select_pending_never_ready(self):
        for tgt in ('drill', 'tennis_ball', 'sprite'):
            code, b = self.req('POST', {'target': tgt})
            self.assertEqual(code, 202, b)
            self.assertEqual((b['selected'], b['status'], b['ready']), (tgt, 'pending', False))
        code, b = self.req()
        self.assertEqual((b['selected'], b['generation'], b['ready']), ('sprite', 3, False))
        self.assertEqual(b['grasp']['reason'], 'Grasp planner not validated for sprite')

    def test_post_refused_when_state_stale_unknown_or_busy(self):
        cases = [None, fresh(self.clock, stamp=self.clock.t - 2), fresh(self.clock, engaged=True),
                 fresh(self.clock, busy=True), fresh(self.clock, held=True), {'engaged': False},
                 RuntimeError('ros down')]
        for st in cases:
            self.state = st
            code, b = self.req('POST', {'target': 'tennis_ball'})
            self.assertEqual(code, 409, (st, b))
            _, g = self.req()
            self.assertFalse(g['can_select'])
            self.assertIsNone(g['selected'])            # nothing changed
            self.assertEqual(g['generation'], 0)

    def test_can_select_not_inferred_from_status(self):
        self.state = fresh(self.clock, held=True)
        _, g = self.req()
        self.assertEqual(g['status'], 'idle')
        self.assertFalse(g['can_select'])

    def test_bad_bodies_rejected(self):
        for raw in (b'{', b'[]', b'{"target": 1}', b'{"target":"drill","force":true}',
                    b'{"target":"screwdriver"}', b'{"tgt":"drill"}'):
            code, _ = self.req('POST', raw=raw)
            self.assertEqual(code, 400, raw)
        self.assertEqual(self.req()[1]['generation'], 0)

    def test_unknown_path_404(self):
        self.assertEqual(self.req(path='/x')[0], 404)
        self.assertEqual(self.req('POST', {'target': 'drill'}, path='/x')[0], 404)


if __name__ == '__main__':
    unittest.main()
