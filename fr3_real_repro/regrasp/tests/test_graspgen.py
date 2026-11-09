"""graspgen.py: protocol end-to-end against local dirs (no lab, no GPU).

A stub sampler plays the server: submit -> poll -> collect must roundtrip
bytes, the timeout path must fire fast with no server, and the one-shot
command must carry the cuda/venv/infer args.

Run: docker exec vm-viewer bash -c
  'source /opt/ros/humble/setup.bash && python3 <this file>'
"""
import importlib.util
import json
import os
import shutil
import sys
import threading
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import graspgen
from graspgen import (DirTransport, infer_oneshot, infer_via_server,
                      new_job_id, server_alive)

SRV = ('/home/nexus/spot-teleop/fr3_real_repro/regrasp/server/'
       'graspgen_server.py')
spec = importlib.util.spec_from_file_location('gserv', SRV)
gserv = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gserv)

ROOT = '/tmp/regrasp_gg_test'


def check(cond, msg):
    if not cond:
        raise AssertionError(msg)
    print(f"ok: {msg}")


check(len(new_job_id()) == 12 and new_job_id() != new_job_id(),
      "job ids unique")
check(graspgen.inbox_name('ab') == 'cloud_ab.npy', "inbox name matches server")
check(graspgen.result_name('ab') == 'grasps_ab.npy',
      "result name matches server")

if os.path.exists(ROOT):
    shutil.rmtree(ROOT)
tr = DirTransport(ROOT)
check(not server_alive(tr, '/srv'), "no heartbeat = not alive")
os.makedirs(os.path.join(ROOT, 'srv'), exist_ok=True)
json.dump({'wall': time.time()}, open(os.path.join(ROOT, 'srv', 'alive.json'), 'w'))
check(server_alive(tr, '/srv'), "fresh heartbeat = alive")
json.dump({'wall': time.time() - 100}, open(os.path.join(ROOT, 'srv', 'alive.json'), 'w'))
check(not server_alive(tr, '/srv'), "stale heartbeat = not alive")

cloud = np.random.default_rng(0).uniform(0.2, 0.5, (500, 3)).astype(np.float32)
os.makedirs('/tmp', exist_ok=True)
np.save('/tmp/regrasp_gg_cloud.npy', cloud)
G0 = np.random.default_rng(1).uniform(-1, 1, (100, 17)).astype(np.float32)


def fake_server():
    inbox = os.path.join(ROOT, 'srv2', 'inbox')
    outbox = os.path.join(ROOT, 'srv2', 'outbox')
    for _ in range(100):
        if gserv.process_inbox(inbox, outbox, lambda c, n: G0) > 0:
            return
        time.sleep(0.05)


threading.Thread(target=fake_server, daemon=True).start()
out = infer_via_server('/tmp/regrasp_gg_cloud.npy', 100, tr, '/srv2', timeout=10)
check(bool((np.load(out) == G0).all()), "server roundtrip bytes exact")

try:
    infer_via_server('/tmp/regrasp_gg_cloud.npy', 100, tr, '/srv3', timeout=2)
    check(False, "no server must time out")
except TimeoutError:
    check(True, "no server times out fast")

os.makedirs(os.path.join(ROOT, 'one', 'grasps.npy'), exist_ok=True)
shutil.rmtree(os.path.join(ROOT, 'one'))
np.save(os.path.join(ROOT, 'grasps.npy'), G0)
tr2 = DirTransport(ROOT)
tr2.get = lambda remote, local: shutil.copy(os.path.join(ROOT, 'grasps.npy'),
                                            local)
out2 = infer_oneshot('/tmp/regrasp_gg_cloud.npy', 100, tr2, '/one')
check(bool((np.load(out2) == G0).all()), "one-shot collects")
check(any('grasp_infer.py' in c and 'CUDA_VISIBLE_DEVICES' in c
          for c in tr2.cmds), "one-shot command carries cuda+infer")
shutil.rmtree(ROOT, ignore_errors=True)
print("9/9 graspgen checks passed")
