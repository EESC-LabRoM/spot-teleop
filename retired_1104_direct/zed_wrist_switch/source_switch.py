#!/usr/bin/env python3
"""Clean control-source switch for FR3 cuRobo teleop: joystick <-> zed wrist.

Offline candidate — no ROS import, stdlib only. Nothing here touches
hardware, controllers, or the map; the live wiring is a small diff to the
lab files (see README.md in this directory).

Confirmed live facts (Hipolito's lane, 2026-11-04):
- `ros2 param set /fr3_joy_teleop input_source joystick|zed` is accepted at
  runtime (_on_params): it calls disengage(), resets zed_anchor, republishes
  latched /teleop/input_source. So the teleop side of the switch is
  fail-closed by itself: any source change drops engagement.
- joystick_bridge.py hardware already gates on that latched topic plus
  engaged plus ZED-active phase, so no bridge restart is needed.
- Setting the param alone does NOT start calibrated ZED tracking
  (start_zed() explicit calibration path). Switching to zed with an
  inactive tracker must therefore end in 'ZED paused', never in motion.

Switch rule implemented below:
  1. refuse while engaged (operator releases Start first);
  2. set the source (via bridge POST /source, which owns the param write);
  3. verify the latched echo (bridge /status must report the new source);
  4. zed additionally requires ZED phase active before re-engage.
"""

SOURCES = ('joystick', 'zed')

# Substrings of bridge /status['reason'] that prove the switch landed.
# When disengaged the bridge reports 'waiting for joystick Start'; a zed
# switch with an inactive tracker reports 'ZED paused' (hold, no motion).
SETTLED_REASONS = ('waiting for joystick Start', 'ZED paused', 'active')


def decide_switch(requested, engaged, current_source):
    """Pure gate for a switch request. Returns (ok, message)."""
    if requested not in SOURCES:
        return False, 'unknown source %r (want one of %s)' % (requested, SOURCES)
    if engaged:
        return False, 'disengage first (release Start) before switching source'
    if requested == current_source:
        return True, 'already on %s (no-op)' % requested
    return True, 'ready to switch %s -> %s' % (current_source, requested)


def verify_settled(status, wanted):
    """Check a bridge /status dict after the set. Returns (ok, message)."""
    source = status.get('source', status.get('input_source'))
    if source != wanted:
        return False, 'bridge still reports source %r (want %r)' % (source, wanted)
    reason = status.get('reason', '')
    if wanted == 'zed' and reason == 'ZED paused':
        return True, 'on zed but tracker inactive: run ZED calibration/start, no motion until active'
    if reason not in SETTLED_REASONS and 'unknown' in reason:
        return False, 'bridge reports: %s' % reason
    return True, 'switch landed: source=%s reason=%s' % (wanted, reason)


class SwitchError(RuntimeError):
    pass


class SwitchClient:
    """Stdlib HTTP client for the bridge :8784 API (GET /status, POST /source)."""

    def __init__(self, base_url='http://127.0.0.1:8784', timeout=5.0):
        import urllib.request
        self._request = urllib.request.Request
        self._urlopen = urllib.request.urlopen
        self.base = base_url.rstrip('/')
        self.timeout = timeout

    def _get_json(self, path):
        import json
        import urllib.error
        try:
            with self._urlopen(self.base + path, timeout=self.timeout) as resp:
                return json.load(resp)
        except urllib.error.HTTPError as exc:
            raise SwitchError('GET %s -> HTTP %s' % (path, exc.code))
        except OSError as exc:
            raise SwitchError('GET %s failed: %s' % (path, exc))

    def _post_json(self, path, body):
        import json
        import urllib.error
        data = json.dumps(body).encode()
        try:
            req = self._request(self.base + path, data=data,
                                headers={'Content-Type': 'application/json'})
            with self._urlopen(req, timeout=self.timeout) as resp:
                return resp.status, json.load(resp)
        except urllib.error.HTTPError as exc:
            try:
                payload = json.load(exc)
            except (ValueError, OSError):
                payload = {'ok': False, 'message': 'HTTP %s' % exc.code}
            return exc.code, payload
        except OSError as exc:
            raise SwitchError('POST %s failed: %s' % (path, exc))

    def status(self):
        return self._get_json('/status')

    def switch(self, wanted):
        """Full clean switch: gate on current state, set, verify echo."""
        import time
        if wanted not in SOURCES:
            raise SwitchError('unknown source %r' % (wanted,))
        before = self.status()
        ok, msg = decide_switch(wanted, before.get('engaged', True),
                                before.get('source', before.get('input_source')))
        if not ok:
            raise SwitchError(msg)
        if msg.startswith('already on'):
            return {'ok': True, 'message': msg, 'status': before}
        code, resp = self._post_json('/source', {'source': wanted})
        if code != 200 or not resp.get('ok'):
            raise SwitchError('bridge refused switch: %s' % resp.get('message', code))
        deadline = time.monotonic() + 5.0
        last = resp
        while time.monotonic() < deadline:
            cur = self.status()
            ok, msg = verify_settled(cur, wanted)
            if ok:
                return {'ok': True, 'message': msg, 'status': cur}
            last = cur
            time.sleep(0.25)
        raise SwitchError('switch not confirmed within 5 s: %r' % (last,))
