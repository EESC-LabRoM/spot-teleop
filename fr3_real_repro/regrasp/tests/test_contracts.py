"""contracts.py: JSON roundtrips + the lock predicate matrix.

Run: docker exec vm-viewer bash -c
  'source /opt/ros/humble/setup.bash && python3 <this file>'
"""
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from contracts import (DrillState, GraspTarget, LockState, Tick,
                       read_lock)


def check(cond, msg):
    if not cond:
        raise AssertionError(msg)
    print(f"ok: {msg}")


t = Tick(wall=1.0, pts=[[0.1, 0.2, 0.3]], n_zed2=10, zed2_src='mask')
check(json.loads(json.dumps(t.__dict__))['n_zed2'] == 10, "tick JSON roundtrip")
d = DrillState(state='TRACKING', run_id='r', trusted=True, n_pts=5,
               icp={'rmse': 4.5})
check(json.loads(d.to_json())['icp']['rmse'] == 4.5, "drill state JSON")
g = GraspTarget(pos=[0.1, 0.2, 0.3], quat_xyzw=[0, 0, 0, 1], run_id='r')
check(json.loads(g.to_json())['pos'][2] == 0.3, "grasp target JSON")

now = 1000.0
lk = LockState(state='HELD_OBJECT', pid=999999, mtime=now - 10000)
check(lk.blocks(now, lambda p: False)[0], "HELD_OBJECT blocks even if stale")
lk = LockState(state='RUN_ACTIVE', pid=123, mtime=now - 5)
check(lk.blocks(now, lambda p: True) == (True, 'run-active'),
      "live RUN_ACTIVE blocks")
lk = LockState(state='RUN_ACTIVE', pid=123, mtime=now - 500)
check(lk.blocks(now, lambda p: True)[0] is False, "stale heartbeat ignored")
lk = LockState(state='RUN_ACTIVE', pid=123, mtime=now - 5)
check(lk.blocks(now, lambda p: False)[0] is False, "dead pid ignored")

lp = '/tmp/regrasp_lock_test.json'
if os.path.exists(lp):
    os.remove(lp)
check(read_lock(lp) == (None, False), "missing lock = no lock")
open(lp, 'w').write('{"state": "HELD_OBJECT", "pid": 1}')
lk, corrupt = read_lock(lp)
check(not corrupt and lk.state == 'HELD_OBJECT', "lock file reads")
open(lp, 'w').write('{broken json')
check(read_lock(lp) == (None, True), "corrupt lock fails safe")
open(lp, 'w').write('')
check(read_lock(lp) == (None, True), "fresh empty lock fails safe")
treal = time.time()
old = treal - 10000
os.utime(lp, (old, old))
check(read_lock(lp, now=treal) == (None, False),
      "old empty lock rots to none")
open(lp, 'w').write('{broken json')
os.utime(lp, (old, old))
check(read_lock(lp, now=treal) == (None, False),
      "old garbled lock rots to none")
os.remove(lp)
print("13/13 contract checks passed")
