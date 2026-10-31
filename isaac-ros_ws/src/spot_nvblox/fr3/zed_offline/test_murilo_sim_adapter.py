"""Positive and fail-closed tests; ROS messages never leave memory."""
import json
import sys
import time
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent))
from murilo_sim_adapter import (ARM, TrajectoryAdapter, hand_goal, instrument_native,
                                live_controller)

Q = [0., -.6, 0., -2., 0., 1.5, 0.]


def trajectory(q=None, names=None):
    names = ARM[::-1] if names is None else names
    q = Q if q is None else q
    order = [ARM.index(n) for n in names]
    return NS(joint_names=names, points=[NS(positions=[q[i] for i in order],
        velocities=[.01]*7, time_from_start=NS(sec=0, nanosec=20000000))])


def setup():
    adapter = TrajectoryAdapter()
    now = time.time_ns()
    adapter.feedback(ARM, Q, now)
    flags = dict(now_ns=now, feasible=True, map_ready=True, map_ns=now,
                 goal_ns=now, coverage=True, active=True)
    return adapter, flags


def test_positive_order_transform_and_continuation():
    adapter, flags = setup()
    target = Q.copy()
    target[0] += .001
    names, q, v = adapter.convert(trajectory(target), **flags)
    assert names == ARM and q == target and v == [.01]*7
    assert adapter.reason == 'motion'
    flags['now_ns'] += 20000000
    adapter.convert(trajectory(target), **flags)
    assert adapter.reason == 'motion'
    p, q = hand_goal([.4, .1, .5], [0., 0., 0., 1.])
    np.testing.assert_allclose(p, [.4, .1, .3966])
    np.testing.assert_allclose(q, [0., 0., 0., 1.])
    p, _ = hand_goal([.4, .1, .5], [1., 0., 0., 0.])
    np.testing.assert_allclose(p, [.4, .1, .6034])


@pytest.mark.parametrize('change', [dict(feasible=False), dict(map_ready=False),
    dict(map_ns=0), dict(coverage=False), dict(active=False), dict(owners=2), dict(goal_ns=0)])
def test_invalid_metadata_produces_measured_hold(change):
    adapter, flags = setup()
    assert adapter.convert(trajectory(), **flags)[1] == Q
    flags.update(change)
    result = adapter.convert(trajectory(), **flags)
    assert result == (ARM, Q, [0.]*7)
    assert adapter.reason != 'motion'


def test_bad_state_and_malformed_output_hold():
    adapter, flags = setup()
    adapter.state_ns = 0
    assert adapter.convert(trajectory(), **flags) == (ARM, Q, [0.]*7)
    adapter.feedback(ARM, Q, flags['now_ns'])
    malformed = trajectory()
    malformed.points[0].positions[0] = float('nan')
    assert adapter.convert(malformed, **flags) == (ARM, Q, [0.]*7)
    adapter.feedback(ARM, [float('nan')]*7, flags['now_ns'])
    assert adapter.convert(trajectory(), **flags) is None


def test_native_hook_exactly_once_and_no_unverified_publish():
    tree = instrument_native('self.cmd_pub.publish(joint_cmd)', 'fixture')
    import ast
    assert ast.unparse(tree) == 'offline_sink(self, joint_cmd, bool(is_feasible), cu_js)'
    with pytest.raises(ValueError):
        instrument_native('pass', 'fixture')


def test_live_mapping_positive_and_dropout_hold():
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Bool, String, Float32MultiArray
    now = time.time_ns()
    clock = NS(now=lambda: NS(nanoseconds=now))
    measured = lambda *frames: (np.zeros(3) if frames else np.array([.4, 0., .4]),
                                np.array([0., 0., 0., 1.]), 0.)
    node = live_controller(dict(PoseStamped=PoseStamped, Bool=Bool, String=String,
                               Float32MultiArray=Float32MultiArray), clock, measured,
                           dict(zed_mapping_profile='review', zed_review_exercise='full_6d', speed_scale=.1))
    node._zed_controller_ready = lambda: None
    published = []
    node._publish = lambda: published.append(node.cmd_p.copy())

    def packet(x):
        nonlocal now
        now += 1000000
        data = dict(version=3, frame='fr3_link0', orientation_mode='full_6d',
                    stamp_ns=now, orientation_stamp_ns=now, position_valid=True,
                    orientation_valid=True, p=[x, 0., .4], q=[0., 0., 0., 1.])
        node.zed_socket.queue.append(json.dumps(data).encode())
        node._receive_zed()

    packet(.2)
    assert node.engage_zed()[0]
    packet(.3)
    node.t_last = time.monotonic()-.02
    node.tick()
    assert node.cmd_p[2] < .4
    assert .4-node.cmd_p[2] <= .08*.1*.021
    assert published
    node.zed_received -= .5
    count = len(published)
    node.tick()
    assert node.zed_phase == 'paused' and len(published) == count
    packet(.31)
    node.tick()
    assert node.engaged and node.zed_phase == 'active'
    packet(.32)
    node.t_last = time.monotonic()-.02
    node.tick()
    assert len(published) > count
    # A larger recovery uses two consistent current wrists and re-anchors to measured TCP.
    node.zed_received -= 1.
    node.tick()
    assert node.zed_phase == 'paused'
    packet(.8)
    node.tick()
    assert node.zed_phase == 'paused'
    packet(.81)
    node.tick()
    assert node.zed_phase == 'active'
    np.testing.assert_allclose(node.cmd_p, [.4, 0., .4])
    node.zed_received -= 2.
    node.tick()
    assert not node.engaged


def test_measured_simulator_feedback_moves_and_hold_tracks_measurement():
    from mujoco_remote_arm import MujocoArm
    arm = MujocoArm()
    try:
        adapter, flags = setup()
        names, measured, _ = arm.joint_state()
        adapter.feedback(names, measured, flags['now_ns'])
        target = measured[:7].copy()
        target[0] += .003
        command = adapter.convert(trajectory(target), **flags)
        assert adapter.reason == 'motion'
        arm.set_targets(*command)
        arm.step(40)
        names, after, _ = arm.joint_state()
        assert after[0] > measured[0]+.0001
        adapter.feedback(names, after, flags['now_ns'])
        flags['active'] = False
        assert adapter.convert(trajectory(target), **flags) == (ARM, after[:7], [0.]*7)
    finally:
        arm.close()
