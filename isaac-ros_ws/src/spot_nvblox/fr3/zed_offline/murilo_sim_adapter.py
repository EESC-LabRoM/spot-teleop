"""Offline transport for the unchanged live ZED controller and native Murilo MPC.

No hardware transport is constructed. Native feasibility accompanies each
trajectory atomically at its publish boundary; a trajectory alone is not proof
of feasibility (upstream publishes infeasible results too).
"""
import ast
import copy
import json
import math
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace as NS

import numpy as np

FR3 = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(FR3 / 'remote_sim'), str(FR3 / 'teleop/curobo')]
from command_guard import ARM, LOW, HIGH, validate  # noqa: E402
from goal_math import tcp_to_hand  # noqa: E402


class MemorySocket:
    """The live parser consumes the same bytes, without opening any UDP socket."""
    def __init__(self):
        self.queue = []

    def bind(self, _):
        pass

    def setblocking(self, _):
        pass

    def recvfrom(self, _):
        if not self.queue:
            raise BlockingIOError
        return self.queue.pop(0), ('127.0.0.1', 0)


class NullPublisher:
    def __init__(self):
        self.messages = []
        self.topic_name = '/fr3_offline/null'

    def publish(self, message):
        self.messages.append(message)

    def get_subscription_count(self):
        return 1


class OfflineNode:
    """In-memory transport facade: the live constructor cannot create ROS endpoints."""
    settings = {}

    def __init__(self, _):
        pass

    def declare_parameter(self, name, default):
        return NS(value=self.settings.get(name, default))

    def create_publisher(self, *args, **kwargs):
        return NullPublisher()

    def create_subscription(self, *args, **kwargs):
        pass

    create_timer = create_subscription
    create_service = create_subscription
    add_on_set_parameters_callback = create_subscription

    def create_client(self, *args, **kwargs):
        return None

    def get_logger(self):
        return NS(info=lambda *a, **k: None, warn=lambda *a, **k: None)


def live_controller(message_types, clock, measured, settings):
    """Execute actual live methods/defaults, replacing transport only.

    AST selection excludes module imports and __main__; no copied mapping,
    follower, pause/recovery, or speed-limit implementation can drift here.
    """
    source = FR3 / 'teleop/fr3_input_teleop.py'
    tree = ast.parse(source.read_text(), filename=str(source))
    selected = [n for n in tree.body if isinstance(n, (ast.FunctionDef, ast.ClassDef))
                or (isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id == 'LAYOUTS' for t in n.targets))]
    namespace = dict(np=np, math=math, time=time, threading=threading, Path=Path,
                     Node=OfflineNode, socket=NS(socket=lambda *a: MemorySocket(), AF_INET=0, SOCK_DGRAM=0),
                     Buffer=lambda: None, TransformListener=lambda *a: None,
                     Grasp=None, Move=None, ActionClient=None, ListControllers=None, Trigger=None, Joy=None,
                     QoSProfile=lambda **k: None, DurabilityPolicy=NS(TRANSIENT_LOCAL=0),
                     ReliabilityPolicy=NS(RELIABLE=0), json=json, __file__=str(source), **message_types)
    OfflineNode.settings = dict(settings, joy_device='', zed_port=0, zed_gripper_enabled=False,
                                input_source='zed', zed_orientation_mode='full_6d')
    exec(compile(ast.Module(body=selected, type_ignores=[]), str(source), 'exec'), namespace)
    node = namespace['Fr3JoyTeleop']()
    node.get_clock = lambda: clock
    node._measured = measured
    node._home_running = lambda: False
    node._home_hold = lambda *a: None
    node._use_layout('xinput')
    return node


def rebase_packet(packet, start_ns, epoch_ns, rate=1.):
    """Use bag_replay's actual stamp rebasing, including paired orientation stamps."""
    if not math.isfinite(rate) or rate <= 0:
        raise ValueError('invalid rate')
    path = FR3 / 'remote_sim/bag_replay.py'
    tree = ast.parse(path.read_text())
    funcs = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'rebase_packet']
    replay = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'Replay')
    mapping = next(n for n in replay.body if isinstance(n, ast.FunctionDef) and n.name == 'map_stamp')
    namespace = dict(copy=copy)
    exec(compile(ast.Module(body=funcs + [mapping], type_ignores=[]), str(path), 'exec'), namespace)
    mapper = NS(offset_ns=epoch_ns, start_ns=start_ns, args=NS(rate=rate))
    return namespace['rebase_packet'](packet, lambda stamp: namespace['map_stamp'](mapper, stamp))


class TrajectoryAdapter:
    """Guard native output with measured simulation state and the live joint limits."""
    def __init__(self):
        self.measured = None
        self.state_ns = 0
        self.previous = None
        self.reason = 'no simulated feedback'

    def feedback(self, names, positions, stamp_ns):
        try:
            if len(names) != len(set(names)) or len(positions) != len(names):
                raise ValueError('feedback shape')
            q = np.array([positions[names.index(n)] for n in ARM], float)
            if q.shape != (7,) or not np.all(np.isfinite(q)):
                raise ValueError('invalid feedback')
            self.measured, self.state_ns = q.tolist(), stamp_ns
        except (ValueError, TypeError):
            self.measured, self.state_ns = None, 0

    def hold(self, reason):
        self.reason = reason
        self.previous = None
        return None if self.measured is None else (ARM.copy(), self.measured.copy(), [0.] * 7)

    def convert(self, trajectory, *, now_ns, feasible, map_ready, map_ns,
                goal_ns, coverage, active, owners=1):
        try:
            if not active or owners != 1:
                raise ValueError('inactive or unexpected output ownership')
            if not map_ready or not 0 <= (now_ns-map_ns)*1e-9 < 1.5:
                raise ValueError('map absent/stale')
            if self.measured is None or not 0 <= (now_ns-self.state_ns)*1e-9 < .2:
                raise ValueError('simulated state absent/stale')
            names = list(trajectory.joint_names)
            if len(names) != 7 or set(names) != set(ARM) or len(trajectory.points) != 1:
                raise ValueError('trajectory names/points')
            point = trajectory.points[0]
            duration = point.time_from_start.sec + point.time_from_start.nanosec*1e-9
            if abs(duration-.02) > 1e-9 or len(point.positions) != 7 or len(point.velocities) != 7:
                raise ValueError('trajectory timing/shape')
            order = [names.index(n) for n in ARM]
            packet = dict(version=1, frame='fr3_link0', stamp_ns=now_ns, goal_stamp_ns=goal_ns,
                          joint_stamp_ns=self.state_ns, esdf_stamp_ns=map_ns,
                          # Offline static snapshot is immutable; no live depth freshness is asserted.
                          depth_stamp_ns=map_ns, feasible=feasible, grid_covers_robot=coverage,
                          names=ARM.copy(), q=[point.positions[i] for i in order],
                          v=[point.velocities[i] for i in order])
            q, v = validate(packet, self.measured, now_ns, self.previous)
            self.previous, self.reason = packet, 'motion'
            return ARM.copy(), q, v
        except (ValueError, KeyError, TypeError, AttributeError, IndexError, OverflowError) as error:
            return self.hold(str(error))


def hand_goal(position, quaternion):
    """MuJoCo's fixed fr3_hand_tcp -> fr3_hand transform (metres, xyzw)."""
    return tcp_to_hand(position, quaternion, [0., 0., -.1034], [0., 0., 0., 1.])


def instrument_native(source, filename):
    """Intercept exactly one native publish, carrying its actual feasibility result."""
    tree = ast.parse(source, filename=filename)
    count = 0

    class Hook(ast.NodeTransformer):
        def visit_Expr(self, node):
            nonlocal count
            if (isinstance(node.value, ast.Call)
                    and ast.unparse(node.value.func) == 'self.cmd_pub.publish'
                    and [ast.unparse(a) for a in node.value.args] == ['joint_cmd']):
                count += 1
                return ast.copy_location(ast.parse(
                    'offline_sink(self, joint_cmd, bool(is_feasible), cu_js, mpc_result, fk_current)').body[0], node)
            return self.generic_visit(node)

    tree = Hook().visit(tree)
    if count != 1:
        raise ValueError(f'native publish interface changed: {count}')
    return ast.fix_missing_locations(tree)


def validate_initial_state(data):
    """Strict measured seven-joint seed; physical state is only used at reset."""
    if not isinstance(data, dict) or data.get('names') != ARM:
        raise ValueError('initial joint names/order')
    if not isinstance(data.get('q'), list) or any(isinstance(value, bool) for value in data['q']):
        raise ValueError('malformed initial joints')
    q = np.asarray(data.get('q'), float)
    if q.shape != (7,) or not np.all(np.isfinite(q)) or np.any(q < LOW) or np.any(q > HIGH):
        raise ValueError('invalid/out-of-limit initial joints')
    provenance = data.get('provenance', {})
    captured, stamp = int(data.get('captured_ns', 0)), int(data.get('stamp_ns', 0))
    if not provenance or not provenance.get('topic') or not 0 <= (captured-stamp)*1e-9 < .2 or stamp <= 0:
        raise ValueError('initial state provenance/freshness at acquisition')
    return q.tolist()


def interior_target(position, settings, margin=.02):
    lo, hi = np.array(settings['workspace_min']), np.array(settings['workspace_max'])
    if np.any(hi-lo <= 2*margin):
        raise ValueError('workspace has no approach interior')
    return np.clip(position, lo+margin, hi-margin)


def cartesian_guard(measured_p, measured_q, target_p, target_q, settings, approach=False):
    """Guard native one-period command FK, including approach from outside the box."""
    p, target = np.asarray(measured_p), np.asarray(target_p)
    lo, hi = np.array(settings['workspace_min']), np.array(settings['workspace_max'])
    violation = lambda x: np.maximum(lo-x, 0)+np.maximum(x-hi, 0)
    if approach:
        if np.any(violation(target) > violation(p)+1e-5):
            raise ValueError('approach moves farther outside workspace')
    elif np.any(target < lo) or np.any(target > hi):
        raise ValueError('native target outside workspace')
    delta = target-p
    if (np.linalg.norm(delta[:2]) > settings['linear_speed']*settings['speed_scale']*.02+1e-5
            or abs(delta[2]) > settings['vertical_speed']*settings['speed_scale']*.02+1e-5
            or np.linalg.norm(delta) > settings['max_step']+1e-5):
        raise ValueError('native Cartesian translation speed/step limit')
    angle = 2*math.acos(min(1., abs(float(np.dot(measured_q, target_q)))))
    if angle > min(math.radians(settings['angular_speed_deg'])*settings['rot_scale']*.02,
                   math.radians(settings['max_slew_deg']))+1e-5:
        raise ValueError('native Cartesian rotation speed limit')


def validate_run_outcome(accepted, movement, source_s, wall_s, sim_s, lateness_s):
    if accepted <= 0 or movement <= .0001:
        raise ValueError('no positive native/measured motion')
    if not .9 <= sim_s/wall_s <= 1.1:
        raise ValueError('simulation real-time factor outside [0.9,1.1]')
    if not .9 <= source_s/wall_s <= 1.1 or lateness_s > .1:
        raise ValueError('1x source scheduling tolerance exceeded')
