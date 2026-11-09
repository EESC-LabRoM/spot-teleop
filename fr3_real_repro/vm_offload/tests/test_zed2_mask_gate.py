"""ZED2 cloud is mask-ONLY: no matched mask means hold-or-nothing, never a box.

Pairing is mask-anchored (newest fresh mask picks its contemporary depth
frame, |dt| <= 0.1 s) because masks lag ~1 s behind depth. The floor falls
back to the FK-measured table height when the ZED1 ring fails.

Run (ROS-sourced): docker exec vm-viewer bash -c
  'source /opt/ros/humble/setup.bash && python3 <this file>'
"""
import os
import sys
import time
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import seg_view
from seg_view import _cap_points, _z2_pair_gate, _zed2_points

W, H = 640, 360
FX, FY, CX, CY = 476.4265441894531, 476.4265441894531, 323.4320373535156, 177.6329345703125


class _TFStub:
    def lookup_transform(self, *a, **k):
        raise Exception('no TF in test (want the JSON-file fallback)')


def _cloud():
    rng = np.random.default_rng(7)
    return np.column_stack([rng.normal(0.48, 0.02, 1500),
                            rng.normal(-0.32, 0.015, 1500),
                            rng.normal(0.20, 0.02, 1500)])


def _depth_msg(depth_m=0.85, stamp=None):
    D = np.full((H, W), depth_m, np.float32)
    now = time.time()
    st = now if stamp is None else stamp
    return (now, SimpleNamespace(
        data=D.tobytes(), height=H, width=W, step=W * 4, encoding='32FC1',
        header=SimpleNamespace(stamp=SimpleNamespace(sec=int(st), nanosec=int(st % 1 * 1e9)),
                               frame_id='zed2_left_camera_optical_frame')))


def _mask_msg_from(M, stamp=None):
    now = time.time()
    st = now if stamp is None else stamp
    return (now, SimpleNamespace(
        data=np.ascontiguousarray(M).tobytes(), height=M.shape[0], width=M.shape[1],
        header=SimpleNamespace(stamp=SimpleNamespace(sec=int(st), nanosec=int(st % 1 * 1e9)),
                               frame_id='mask')))


def _mask_msg(fill):
    return _mask_msg_from(np.full((H, W), fill, np.uint8))


def _info2():
    return SimpleNamespace(k=[FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0], width=W,
                           header=SimpleNamespace(frame_id='zed2_left_camera_optical_frame'))


def _hold_reset():
    seg_view._z2_hold['pts'], seg_view._z2_hold['wall'] = None, 0.0


def check(cond, msg):
    if not cond:
        raise AssertionError(msg)
    print(f"ok: {msg}")


_hold_reset()
Pc, depths = _cloud(), [_depth_msg()]

# 1. no mask + empty hold -> nothing, NEVER a box crop (the box fallback
# was removed: it silently admitted table/clutter as drill).
Pno, sno = _zed2_points(Pc, 0.0, depths, _info2(), _TFStub(), masks2=None)
check(Pno is None and sno == 'none', 'no mask -> None/none, never a box crop')

# 2-3. fresh mask -> mask src, and the mask gates the ROI.
full = np.full((H, W), 255, np.uint8)
Pfull, sfull = _zed2_points(Pc, 0.0, depths, _info2(), _TFStub(), masks2=[_mask_msg_from(full)])
check(Pfull is not None and sfull == 'mask', f'fresh mask -> mask src ({0 if Pfull is None else len(Pfull)} pts)')
left = np.zeros((H, W), np.uint8)
left[:, :W // 2] = 255
right = np.zeros((H, W), np.uint8)
right[:, W // 2:] = 255
nmask, sms = [], []
for M in (left, right):
    _hold_reset()  # isolate halves: no hold carry between them
    Pm, sm = _zed2_points(Pc, 0.0, depths, _info2(), _TFStub(), masks2=[_mask_msg_from(M)])
    sms.append(sm)
    nmask.append(0 if Pm is None else len(Pm))
check(set(sms) <= {'mask', 'none'} and max(nmask) > 0,
      f'halves report mask/none without hold ({sms}, {nmask})')
check(min(nmask) < len(Pfull), f'mask gates the ROI ({nmask} vs {len(Pfull)} full-mask pts)')

# 4. fresh-but-empty mask -> None via mask, and it clears the hold.
_hold_reset()
_zed2_points(Pc, 0.0, depths, _info2(), _TFStub(), masks2=[_mask_msg_from(full)])
check(seg_view._z2_hold['pts'] is not None, 'hold armed by a success')
Pzero, szero = _zed2_points(Pc, 0.0, depths, _info2(), _TFStub(), masks2=[_mask_msg(0)])
check(Pzero is None and szero == 'mask', 'fresh-but-empty mask -> None via mask')
check(seg_view._z2_hold['pts'] is None, 'empty mask clears the hold (object gone, no ghost)')

# 5. mask-anchored pairing: the mask picks its contemporary depth, not the
# latest one. Mask stamp == first depth; latest depth is 1 s newer.
_hold_reset()
d2 = [_depth_msg(0.85, stamp=1000.0), _depth_msg(0.85, stamp=1001.0)]
Pm, sm = _zed2_points(Pc, 0.0, d2, _info2(), _TFStub(), masks2=[_mask_msg_from(full, stamp=1000.0)])
check(Pm is not None and sm == 'mask', 'mask pairs its contemporary depth (latest-depth pairing would miss)')

# 6. miss after success -> held mask set, then expiry -> none.
Ph, sh = _zed2_points(Pc, 0.0, d2, _info2(), _TFStub(), masks2=None)
check(Ph is not None and sh == 'mask-held' and np.array_equal(Ph, Pm),
      'pairing miss holds the last mask-gated set (<=4 s)')
seg_view._z2_hold['wall'] = time.time() - 3.0
P3, s3 = _zed2_points(Pc, 0.0, d2, _info2(), _TFStub(), masks2=None)
check(P3 is not None and s3 == 'mask-held', 'hold survives a missed 2 s tick')
seg_view._z2_hold['wall'] = time.time() - 5.0
Pe, se = _zed2_points(Pc, 0.0, d2, _info2(), _TFStub(), masks2=None)
check(Pe is None and se == 'none', 'aged hold expires to None/none')

# 7. floor falls back to the FK table height when the ring fails: W with a
# high p2 must not cut drill points at ~0.2 m (old floor W-p2+2.5 cm did).
_hold_reset()
Whi = Pc + [0.0, 0.0, 0.30]
Pf, sf = _zed2_points(Whi, None, depths, _info2(), _TFStub(), masks2=[_mask_msg_from(full)])
check(Pf is not None and sf == 'mask' and len(Pf) > 100,
      f'ring failed -> FK floor keeps the drill ({0 if Pf is None else len(Pf)} pts)')

# 8. cap subsamples to exactly max_pts.
big = np.random.default_rng(3).normal(size=(10000, 3))
Pc4 = _cap_points(big, 4000)
have = set(map(tuple, np.round(big, 9)))
check(len(Pc4) == 4000 and all(tuple(np.round(r, 9)) in have for r in Pc4),
      'cap keeps exactly max_pts, all from the input')
check(len(_cap_points(big[:100], 4000)) == 100, 'cap passes small sets through')

# 9. gate adapts to sparse depth: 0.5 s spacing -> 0.5 s gate, so a mask
# 0.25 s from the nearest depth still pairs (a fixed 0.1 s gate misses).
d3 = [_depth_msg(0.85, stamp=1000.0), _depth_msg(0.85, stamp=1000.5),
      _depth_msg(0.85, stamp=1001.0)]
check(abs(_z2_pair_gate(d3) - 0.5) < 1e-9, 'sparse depth widens the gate to 0.5 s')
check(abs(_z2_pair_gate([_depth_msg(0.85, stamp=1000.0 + 0.033 * i) for i in range(5)]) - 0.1) < 1e-9,
      'dense depth keeps the 0.1 s floor')
_hold_reset()
Pa, sa = _zed2_points(Pc, 0.0, d3, _info2(), _TFStub(), masks2=[_mask_msg_from(full, stamp=1000.75)])
check(Pa is not None and sa == 'mask', 'mask 0.25 s off the depth grid still pairs')
_hold_reset()  # isolate: Pa armed the hold, the miss below must see it empty
Pb, sb = _zed2_points(Pc, 0.0, d3, _info2(), _TFStub(), masks2=[_mask_msg_from(full, stamp=1001.75)])
check(Pb is None and sb == 'none', 'mask beyond the gate still misses')

# 10. the mask is eroded (same 5x5 as ZED1): fine speckle dies instead of
# becoming flying pixels (without erode this mask yields points).
chk = (np.indices((H, W)).sum(0) // 2 % 2 * 255).astype(np.uint8)
_hold_reset()
Ps, ss = _zed2_points(Pc, 0.0, depths, _info2(), _TFStub(), masks2=[_mask_msg_from(chk)])
check(Ps is None and ss == 'mask', 'speckle mask erodes to None via mask')

print('test_zed2_mask_gate: 10/10 PASS')
