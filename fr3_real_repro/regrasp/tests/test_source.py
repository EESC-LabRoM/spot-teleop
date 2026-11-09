"""source.py: parse + acceptance on a RECORDED grasp.json (lying3 snap_1).

The fetch path is exercised against a stub HTTP server (no live viewer
needed), so this test is deterministic offline.

Run: docker exec vm-viewer bash -c
  'source /opt/ros/humble/setup.bash && python3 <this file>'
"""
import copy
import functools
import http.server
import json
import os
import sys
import threading

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from source import is_paired, poll_tick, tick_from_grasp

SNAP = ('/home/nexus/spot-teleop/fr3_real_repro/grasp_loop/runs/'
        '20261106_152112_drill_lying3/snap_1.json')


def check(cond, msg):
    if not cond:
        raise AssertionError(msg)
    print(f"ok: {msg}")


d = json.load(open(SNAP))
t = tick_from_grasp(d, wall=123.0)
check(t.wall == 123.0 and t.n_zed2 >= 3000 and len(t.pts) >= 6000,
      f"recorded snap parses ({len(t.pts)} pts, {t.n_zed2} zed2)")
check(is_paired(t), "recorded mask tick accepted")

box = copy.deepcopy(d)
box['live']['cloud'] = 'live_mask_depth+zed2_none'
box['live']['n_zed2'] = 0
check(not is_paired(tick_from_grasp(box)), "box-fallback tick rejected")
stale = copy.deepcopy(d)
stale['live']['cloud_age_s'] = 30.0
check(not is_paired(tick_from_grasp(stale)), "stale tick rejected")
try:
    tick_from_grasp({'run': 'x'})
    check(False, "cloudless payload must raise")
except ValueError:
    check(True, "cloudless payload raises ValueError")


class _H(http.server.BaseHTTPRequestHandler):
    payload = b'{}'

    def do_GET(self):
        body = _H.payload
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *a):
        pass


_H.payload = json.dumps(d).encode()
srv = http.server.HTTPServer(('127.0.0.1', 0), _H)
threading.Thread(target=srv.serve_forever, daemon=True).start()
url = f'http://127.0.0.1:{srv.server_port}/grasp.json'
t2, reason = poll_tick(url)
check(t2 is not None and reason == 'ok', "poll_tick fetches + accepts")
t3, reason = poll_tick('http://127.0.0.1:1/none', timeout=1)
check(t3 is None and reason.startswith('fetch'), "dead URL never raises")
srv.shutdown()
print("7/7 source checks passed")
