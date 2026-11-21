"""Offline HTTP adapter: GET/POST /object-target over ObjectSelector.

stdlib only, no ROS. The teleop state comes from an injected provider that
returns {'engaged','busy','held': bool, 'stamp': float} or None. can_select is
computed from that fresh state only, never from selector status; unknown,
stale or malformed state makes can_select false and POST refuses (409).
Binds 127.0.0.1 by default. Nothing here commands the robot, tracker or map.
"""
from __future__ import annotations

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Callable, Optional

from selector_backend import ObjectSelector, SelectionRefused, Target

STATE_MAX_AGE_S = 1.0
MAX_BODY = 1024


def teleop_gate(state: Optional[dict], now: float) -> tuple:
    """(can_select, reason, flags) from fresh teleop state; fail closed."""
    if not isinstance(state, dict):
        return False, 'Teleop state unknown', None
    flags = {k: state.get(k) for k in ('engaged', 'busy', 'held')}
    if not all(isinstance(v, bool) for v in flags.values()):
        return False, 'Teleop state malformed', None
    stamp = state.get('stamp')
    if isinstance(stamp, bool) or not isinstance(stamp, (int, float)):
        return False, 'Teleop state has no stamp', flags
    age = now - stamp
    if not (0 <= age <= STATE_MAX_AGE_S):
        return False, f'Teleop state stale ({age:.2f}s)', flags
    if any(flags.values()):
        return False, 'Disengage and release any held object before changing targets', flags
    return True, 'OK', flags


class SelectorService:
    """Transport-free core so the HTTP layer stays trivial and testable."""

    def __init__(self, selector: ObjectSelector, state_provider: Callable[[], Optional[dict]],
                 clock: Callable[[], float]):
        self.selector = selector
        self.state_provider = state_provider
        self.clock = clock
        self.lock = threading.Lock()

    def _state(self):
        try:
            return teleop_gate(self.state_provider(), self.clock())
        except Exception as exc:
            return False, f'Teleop state error: {exc}', None

    def get(self) -> tuple:
        with self.lock:
            try:
                self.selector.evaluate()
                body = self.selector.to_api()
            except Exception as exc:
                body = {'selected': None, 'status': 'error', 'ready': False,
                        'reason': f'Selector error: {exc}'}
            can, why, _ = self._state()
            body['can_select'] = can
            body['can_select_reason'] = why
            body['targets'] = [t.value for t in Target]
            return 200, body

    def post(self, raw: bytes) -> tuple:
        try:
            req = json.loads(raw.decode('utf-8'))
            target = req['target']
            if not isinstance(req, dict) or set(req) != {'target'} or not isinstance(target, str):
                raise ValueError
        except Exception:
            return 400, {'ok': False, 'reason': 'Body must be {"target": "drill"|"tennis_ball"|"sprite"}'}
        if target not in {t.value for t in Target}:
            return 400, {'ok': False, 'reason': f'Unknown target {target!r}'}
        with self.lock:
            can, why, flags = self._state()
            if not can:
                return 409, {'ok': False, 'reason': why}
            try:
                ident = self.selector.select(target, **flags)
            except SelectionRefused as exc:
                return 409, {'ok': False, 'reason': str(exc)}
            self.selector.evaluate()
            body = self.selector.to_api()
            body.update(ok=True, accepted=ident.target, ready=False)
            return 202, body


def make_handler(service: SelectorService):
    class Handler(BaseHTTPRequestHandler):
        def _send(self, code, body):
            data = json.dumps(body).encode()
            self.send_response(code)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path.split('?')[0] != '/object-target':
                return self._send(404, {'ok': False, 'reason': 'not found'})
            self._send(*service.get())

        def do_POST(self):
            if self.path.split('?')[0] != '/object-target':
                return self._send(404, {'ok': False, 'reason': 'not found'})
            try:
                n = int(self.headers.get('Content-Length', '0'))
            except ValueError:
                n = -1
            if not (0 < n <= MAX_BODY):
                return self._send(400, {'ok': False, 'reason': 'Bad Content-Length'})
            self._send(*service.post(self.rfile.read(n)))

        def log_message(self, *args):
            pass
    return Handler


def make_server(service: SelectorService, host='127.0.0.1', port=0) -> ThreadingHTTPServer:
    return ThreadingHTTPServer((host, port), make_handler(service))
