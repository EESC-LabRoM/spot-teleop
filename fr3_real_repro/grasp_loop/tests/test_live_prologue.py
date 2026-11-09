"""Offline tests for live_prologue pure functions (no ROS, stdlib only)."""
import math
import os
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from live_prologue import (apply_transform, check_mask_age, invert_transform,
                           load_chain, median_point, quat_mul, quat_rotate,
                           shift_pick, Fail)


def test_median_rejects_outlier():
    pts = [(0.12, 0.23, 0.53), (0.121, 0.231, 0.531), (9.9, -9.9, 9.9),
           (0.119, 0.229, 0.529), (0.122, 0.232, 0.532)]
    m = median_point(pts)
    assert abs(m[0] - 0.121) < 1e-9 and abs(m[2] - 0.531) < 1e-9, m


def test_median_needs_samples():
    try:
        median_point([(0, 0, 0)])
    except Fail:
        return
    raise AssertionError('should have failed')


def test_quat_identity_and_yaw90():
    assert quat_rotate([0, 0, 0, 1], [1, 2, 3]) == [1, 2, 3]
    s = math.sqrt(0.5)
    r = quat_rotate([0, 0, s, s], [1, 0, 0])  # +90 deg about z
    assert all(abs(a - b) < 1e-9 for a, b in zip(r, [0, 1, 0])), r


def test_apply_transform():
    p = apply_transform([1, 0, 0], [0, 0, 0, 1], [0.1, 0.2, 0.3])
    assert p == [1.1, 0.2, 0.3], p


def test_shift_pick_and_jump_guard():
    P, d = shift_pick([0.5, 0.0, 0.3], [0.6, -0.2, 0.1], [0.62, -0.21, 0.11])
    assert [round(v, 4) for v in P] == [0.52, -0.01, 0.31], P
    try:
        shift_pick([0.5, 0.0, 0.3], [0.0, 0.0, 0.0], [0.5, 0.0, 0.0])
    except Fail:
        return
    raise AssertionError('jump guard should have tripped')


def test_mask_age_gate():
    assert check_mask_age(100.0, 101.5) == 1.5
    for bad in (97.0, 105.0):  # stale and future
        try:
            check_mask_age(bad, 100.0)
        except Fail:
            continue
        raise AssertionError(f'age gate missed {bad}')


def test_quat_mul():
    s = math.sqrt(0.5)
    q90 = [0, 0, s, s]  # +90 deg about z
    q180 = quat_mul(q90, q90)
    r = quat_rotate(q180, [1, 0, 0])
    assert all(abs(a - b) < 1e-9 for a, b in zip(r, [-1, 0, 0])), r


def test_chain_matches_live_tf2_echo():
    # Independent oracle: tf2_echo on the lab composed link0->optical as
    # t=[0.238,-0.594,0.632] q=[-0.841,0.120,-0.140,0.508] (3 decimals).
    t, q = load_chain()
    assert all(abs(a - b) < 2e-3 for a, b in
               zip(t, [0.238, -0.594, 0.632])), t
    # quaternions double-cover: accept q or -q
    ref = [-0.841, 0.120, -0.140, 0.508]
    close = lambda s: all(abs(s * a - b) < 2e-3 for a, b in zip(q, ref))
    assert close(1.0) or close(-1.0), q


def test_inverse_roundtrip():
    t, q = load_chain()
    ti, qi = invert_transform(t, q)
    p = [0.1227, 0.2330, 0.5367]
    fwd = apply_transform(t, q, p)
    back = apply_transform(ti, qi, fwd)
    assert all(abs(a - b) < 1e-6 for a, b in zip(back, p)), back


def test_replay_end_to_end(tmpdir=None):
    import json
    d = tempfile.mkdtemp()
    json.dump({'pos': [0.556, -0.228, 0.322],
               'quat_xyzw': [0, 0, 0, 1], 'conf': 0.7},
              open(os.path.join(d, 'pick.json'), 'w'))
    json.dump({'seed_3d': [0.61, -0.267, 0.143]},
              open(os.path.join(d, 'seed.json'), 'w'))
    # live tracker point already in link0-ish coords via identity TF
    json.dump({'points': [[0.62, -0.26, 0.15]] * 5,
               'mask_stamp': 100.0, 'now': 100.5,
               'tf_t': [0, 0, 0], 'tf_q': [0, 0, 0, 1]},
              open(os.path.join(d, 'samples.json'), 'w'))
    r = subprocess.run([sys.executable, 'live_prologue.py', '--run', d,
                        '--replay', os.path.join(d, 'samples.json'),
                        '--out', os.path.join(d, 'prologue.json')],
                       cwd=os.path.join(os.path.dirname(__file__), '..'),
                       capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    out = json.load(open(os.path.join(d, 'prologue.json')))
    assert [round(v, 4) for v in out['pick_live_m']] == [0.566, -0.221, 0.329]
    env = open(os.path.join(d, 'prologue.env')).read().strip()
    assert env == 'GTX=0.5660 GTY=-0.2210 GTZ=0.3290', env


if __name__ == '__main__':
    fns = [(k, v) for k, v in sorted(globals().items())
           if k.startswith('test_')]
    for name, fn in fns:
        fn()
        print(f'PASS {name}')
    print(f'{len(fns)}/{len(fns)} green')
