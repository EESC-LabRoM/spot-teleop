import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from object_gate import ObjectGate, gate_yaw  # noqa: E402

DRILL = np.array([0.442, -0.103, 0.232])
OFF = np.array([-0.03, -0.12, 0.10])          # grasp hand = drill + OFF
HOME = np.array([0.310, -0.003, 0.589])
# target_object samples logged by vm-target-bridge during the 21:21Z pad test (drill untouched)
JUMPS = [(0.504, 0.056, 0.095), (0.444, -0.099, 0.228), (0.434, -0.095, 0.211), (0.426, -0.021, 0.103),
         (0.564, 0.136, 0.035), (0.550, 0.119, 0.046), (0.583, 0.187, 0.017), (0.487, -0.047, 0.208)]


def run(g, seq, dt=0.05):
    out, t = [], 0.0
    for tf, hand in seq:
        out.append(g.update(t, tf, hand))
        t += dt
    return out


def test_jumps_while_arm_moves_are_ignored():
    g = ObjectGate(DRILL, OFF)
    seq = []
    for i in range(400):                       # 20 s, the hand sweeping 1 cm/s+ toward the drill, TF jumping
        hand = HOME + (DRILL + OFF - HOME) * min(1.0, i / 300) * 0.8 + np.array([0.0, 0.002 * np.sin(i), 0.0])
        seq.append((np.array(JUMPS[(i // 20) % len(JUMPS)]), hand))
    out = run(g, seq)
    assert g.n_updates == 0
    assert np.allclose(out[-1][0], DRILL)


def test_real_move_is_taken_once_arm_pauses():
    g = ObjectGate(DRILL, OFF)
    moved = DRILL + np.array([0.05, 0.03, 0.0])
    seq = [(moved + np.random.default_rng(i).normal(0, 0.002, 3), HOME) for i in range(60)]
    out = run(g, seq)
    assert g.n_updates >= 1
    assert np.linalg.norm(out[-1][0] - moved) < 0.005


def test_unstable_tf_with_still_arm_is_held():
    g = ObjectGate(DRILL, OFF)
    seq = [(np.array(JUMPS[i % 2]), HOME) for i in range(40)]
    run(g, seq)
    assert g.n_updates == 0


def test_no_update_with_hand_near_grasp():
    g = ObjectGate(DRILL, OFF)
    hand = DRILL + OFF + np.array([0.0, 0.0, 0.05])   # hovering 5 cm above the grasp: hand occludes the drill
    seq = [(DRILL + np.array([0.04, 0.0, 0.0]), hand) for _ in range(40)]
    run(g, seq)
    assert g.n_updates == 0


def test_gap_resets_evidence():
    g = ObjectGate(DRILL, OFF)
    moved = DRILL + np.array([0.05, 0.0, 0.0])
    seq = [(moved, HOME)] * 30 + [(None, HOME)] + [(moved, HOME)] * 30   # 1.5 s + gap + 1.5 s: never 2 s straight
    run(g, seq)
    assert g.n_updates == 0


def test_blend_line_points_with_still_arm_are_held():
    # 21:30Z, arm still at home: tracker point slid along drill -> table, each value held ~0.6 s (1.6 Hz).
    # Off-height blend values are dropped; the in-band drill value a is the real drill and may be adopted,
    # a blend value never is.
    g = ObjectGate(DRILL, OFF)
    a, b = np.array([0.471, -0.078, 0.233]), np.array([0.58, 0.19, 0.02])
    seq = []
    for k in range(20):
        f = [0.0, 0.2, 0.0, 0.4, 0.15, 0.0][k % 6]
        seq += [(a + f * (b - a), HOME)] * 12
    run(g, seq)
    assert np.allclose(g.held, DRILL) or np.linalg.norm(g.held - a) < 0.003


def test_moved_drill_adopted_despite_table_jumps():
    # 22:33-22:35Z: drill moved, arm still, tracker jumps to the table blob for one sample every ~1 s
    g = ObjectGate(DRILL, OFF)
    moved = np.array([0.444, -0.145, 0.240])
    table = np.array([0.55, 0.10, 0.02])
    seq = [((table if i % 20 == 10 else moved), HOME) for i in range(80)]   # 4 s at 20 Hz
    out = run(g, seq)
    assert g.n_updates >= 1 and np.linalg.norm(out[-1][0] - moved) < 0.003


def test_yaw_only_moves_on_trusted_ticks():
    held = np.radians(5.0)
    for why in ('hold: arm moving', 'hold: hand near grasp', 'hold: TF unstable', 'hold: TF height off',
                'hold: collecting', 'hold: no TF', 'hold: no hand TF'):
        assert gate_yaw(np.radians(25.0), held, why) == held
    assert gate_yaw(np.radians(25.0), held, 'steady') == np.radians(25.0)
    assert gate_yaw(np.radians(25.0), held, 'updated') == np.radians(25.0)
