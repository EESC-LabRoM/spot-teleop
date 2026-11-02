"""Isolated native Murilo MPC / current ESDF snapshot / measured MuJoCo pilot.

Host entry launches one network-disabled child container. The child has no live
ROS context, hardware devices, host networking, published ports, or UDP bridge.
A current snapshot is an explicitly different environment from the recorded one.
"""
import argparse
import base64
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import threading
import queue
import time
from types import SimpleNamespace as NS

HERE = Path(__file__).resolve().parent
FR3 = HERE.parent
REPO = FR3.parents[3]


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def enumerate_takes(root):
    return sorted(p.name for p in Path(root).iterdir() if p.is_dir() and (p/'d8/metadata.yaml').is_file())


def timed_events(bag):
    """Read operator-only MCAP bytes, retaining acquisition AND bag receipt times."""
    from mcap_ros2.reader import read_ros2_messages
    import yaml
    metadata = yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    start_ns = metadata['starting_time']['nanoseconds_since_epoch']
    events = []
    for path in sorted(bag.glob('*.mcap')):
        with path.open('rb') as stream:
            for msg in read_ros2_messages(stream, topics=['/fr3_operator/teleop_sample']):
                events.append((msg.log_time_ns, json.loads(msg.ros_msg.data)))
    return start_ns, sorted(events, key=lambda e: e[0])


def pilot(args, manifest):
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.serialization import deserialize_message
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, Float32MultiArray, String
    from nvblox_msgs.srv import EsdfAndGradients
    from murilo_sim_adapter import (TrajectoryAdapter, live_controller, hand_goal,
                                    instrument_native, rebase_packet, validate_initial_state,
                                    interior_target, cartesian_guard, validate_run_outcome)
    sys.path.insert(0, str(FR3/'remote_sim'))
    from murilo_sim_physics import PhysicsProcess
    DT = .005
    settings = json.loads(args.limits_json.read_text())
    seed_data = json.loads(args.initial_state.read_text())
    seed = validate_initial_state(seed_data)
    manifest["initial_state"] = dict(seed_data, sha256=digest(args.initial_state))

    snapshot = json.loads(args.map_snapshot.read_text())
    response = deserialize_message(base64.b64decode(snapshot['cdr']), EsdfAndGradients.Response)
    if not response.success or abs(response.voxel_size_m-.05) > 1e-6 or not response.esdf_and_gradients.data:
        raise ValueError('invalid or empty ESDF snapshot')
    manifest['map'] = {k: v for k, v in snapshot.items() if k != 'cdr'}
    manifest['map'].update(sha256=digest(args.map_snapshot), environment='current snapshot, not historical',
                           replay_policy='immutable offline snapshot; refresh is local serving time',
                           unknown_voxels=int(np.sum(np.asarray(response.esdf_and_gradients.data) < -999.9)),
                           unknown_policy='unchanged native parser treats distances below -999.9 as free; not observed safety evidence')
    rclpy.init(domain_id=args.domain, args=['--ros-args'] + sum((['-p', f'{k}:={v}'] for k, v in {
        'robot':'franka', 'base_frame':'fr3_link0', 'robot_config':str(args.yaml),
        'use_sim':'false', 'use_ros2_control':'false', 'feedback_mode':'measured',
        'start_active':'true', 'use_esdf':'true', 'voxel_size':'0.05',
        'grid_size_m':'[1.4,1.6,1.2]', 'grid_center_m':'[0.35,0.0,0.55]',
        'esdf_service_name':'/snapshot', 'esdf_frame_id':'fr3_link0', 'esdf_global_frame':'fr3_link0',
        'pf_mode':"'off'", 'esdf_update_rate':'4.0', 'command_type':'joint_trajectory',
        'command_period':'0.02', 'mpc_on_infeasible':'keep', 'log_csv_path':str(args.output/'native.csv')
    }.items()), []) + ['-r', '/rosout:=/fr3_offline/rosout'])

    class PrivateNode(Node):
        def __init__(self, name):
            super().__init__(name, namespace='/fr3_offline')

        def create_publisher(self, kind, topic, *a, **kw):
            return super().create_publisher(kind, private(topic), *a, **kw)

        def create_subscription(self, kind, topic, *a, **kw):
            return super().create_subscription(kind, private(topic), *a, **kw)

        def create_service(self, kind, topic, *a, **kw):
            return super().create_service(kind, private(topic), *a, **kw)

        def create_client(self, kind, topic, *a, **kw):
            return super().create_client(kind, private(topic), *a, **kw)

    def private(topic):
        return '/fr3_offline/' + topic.replace('~', 'native').lstrip('/')

    service = PrivateNode('snapshot_provider')
    joint_pub = service.create_publisher(JointState, '/joint_states', 10)
    tcp_pub = service.create_publisher(PoseStamped, '/measured_tcp', 10)
    goal_pub = service.create_publisher(PoseStamped, '/wrist_pose', 1)
    map_served_ns = 0

    def serve(request, _response):
        nonlocal map_served_ns
        # Reject differing query geometry/clearing: never claim another map's semantics.
        minimum, size = request.aabb_min_m, request.aabb_size_m
        if (request.frame_id != 'fr3_link0' or not request.use_aabb
                or not np.allclose([minimum.x, minimum.y, minimum.z], [-.35, -.8, -.05])
                or not np.allclose([size.x, size.y, size.z], [1.4, 1.6, 1.2])
                or request.aabbs_to_clear_min_m or request.spheres_to_clear_center_m):
            return EsdfAndGradients.Response()
        map_served_ns = time.time_ns()
        return response

    service.create_service(EsdfAndGradients, '/snapshot', serve)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(service)
    executor_stopping = threading.Event()

    def spin():
        while not executor_stopping.is_set():
            executor.spin_once(timeout_sec=.05)

    executor_thread = threading.Thread(target=spin, daemon=True)
    executor_thread.start()
    arm = PhysicsProcess(seed, settings, args.output)
    lock = threading.RLock()
    adapter = TrajectoryAdapter()
    trace = (args.output/'trace.jsonl').open('x')
    pending = []
    native_count = 0
    accepted = 0
    controller = None
    stop = threading.Event()
    phase = 'initial_check'
    approach_p = None
    approach_q = None
    feasible_count = 0
    costs = {}
    timings = []
    measured_samples = []
    lateness = []
    log_queue = queue.SimpleQueue()

    def write_logs():
        while True:
            value = log_queue.get()
            if value is None:
                break
            trace.write(json.dumps(value, allow_nan=False)+'\n')
        trace.flush()

    logger_thread = threading.Thread(target=write_logs, daemon=True)
    logger_thread.start()

    def log(event, **fields):
        log_queue.put(dict(event=event, wall_ns=time.time_ns(), **fields))

    def sink(node, trajectory, feasible, state, result, fk_current):
        nonlocal native_count, accepted, feasible_count, costs
        if stop.is_set():
            return
        started = time.perf_counter()
        node.cmd_pub.publish(trajectory)
        native_count += 1
        feasible_count += bool(feasible and node._esdf_initialized)
        # Native FK already computed by the unchanged native control loop.
        spheres = fk_current.robot_spheres.detach().cpu().numpy().reshape(-1, 4)
        spheres = spheres[spheres[:, 3] > 0]
        lo = np.asarray(getattr(node, '_last_grid_bb_min', [np.inf]*3))
        hi = np.asarray(getattr(node, '_last_grid_bb_max', [-np.inf]*3))
        coverage = bool(len(spheres) and np.all(spheres[:, :3]-spheres[:, 3:] >= lo)
                        and np.all(spheres[:, :3]+spheres[:, 3:] <= hi))
        if not feasible and not costs:
            # Evaluate actual native constraints; do not infer free space from a loaded map.
            rollout = node.mpc.solver.safety_rollout
            current = rollout._get_augmented_state(state)
            seq = result.metrics.state
            unpack = lambda value: value.detach().cpu().numpy().reshape(-1).tolist()
            costs = dict(native_total=unpack(result.metrics.constraint),
                         native_cost=unpack(result.metrics.cost))
            for label, constraint in [('bound', rollout.bound_constraint),
                                      ('self', rollout.robot_self_collision_constraint),
                                      ('world', rollout.primitive_collision_constraint)]:
                try:
                    if label == 'bound':
                        actual = unpack(constraint.forward(current.state_seq))
                        predicted = constraint.forward(seq.state_seq)
                    elif label == 'world':
                        actual = unpack(constraint.forward(current.robot_spheres, env_query_idx=rollout._goal_buffer.batch_world_idx))
                        predicted = constraint.forward(seq.robot_spheres, env_query_idx=rollout._goal_buffer.batch_world_idx)
                    else:
                        actual = unpack(constraint.forward(current.robot_spheres))
                        predicted = constraint.forward(seq.robot_spheres)
                    costs[label] = dict(actual=actual, predicted=unpack(predicted))
                except Exception as error:
                    costs[label] = dict(error=repr(error))
            try:
                costs['sphere_distances_native'] = node._get_sphere_distances()
            except Exception as error:
                costs['sphere_distances_error'] = repr(error)
            manifest['constraint_evidence'] = costs
            (args.output/'constraints.json').write_text(json.dumps(costs, indent=2))
        kinematics_done = time.perf_counter()
        goal_ns = 0 if node._pose_msg is None else (node._pose_msg.header.stamp.sec*10**9+node._pose_msg.header.stamp.nanosec)
        extra_reason = None
        if feasible:
            predicted = node.mpc.rollout_fn.compute_kinematics(result.js_action)
            points = predicted.robot_spheres.detach().cpu().numpy().reshape(-1, 4)
            points = points[points[:, 3] > 0]
            coverage = coverage and bool(len(points) and np.all(points[:, :3]-points[:, 3:] >= lo)
                                        and np.all(points[:, :3]+points[:, 3:] <= hi))
            hand_p = predicted.ee_pos_seq.detach().cpu().numpy().reshape(-1, 3)[0]
            hand_wxyz = predicted.ee_quat_seq.detach().cpu().numpy().reshape(-1, 4)[0]
            quat = hand_wxyz[[1, 2, 3, 0]]
            from goal_math import tcp_to_hand
            tcp_p, tcp_q = tcp_to_hand(hand_p, quat, [0., 0., .1034], [0., 0., 0., 1.])
            try:
                measured_p, measured_q = arm.tcp_pose()
                cartesian_guard(measured_p, measured_q, tcp_p, tcp_q, settings, approach=phase == 'approach')
            except ValueError as error:
                extra_reason = str(error)
        # All GPU and telemetry work above is outside the short feedback/guard lock.
        with lock:
            result_command = adapter.convert(trajectory, now_ns=time.time_ns(), feasible=feasible,
                map_ready=node._esdf_initialized, map_ns=map_served_ns, goal_ns=goal_ns,
                coverage=coverage, active=extra_reason is None and (phase == 'approach' or
                    (phase == 'replay' and controller is not None and controller.engaged
                     and controller.zed_phase == 'active')),
                owners=len(node.get_publishers_info_by_topic('/fr3_offline/arm/joint_command')))
            reason = extra_reason or adapter.reason
            if (phase == 'approach' or (phase == 'replay' and controller.engaged and controller.zed_phase == 'active')) and reason != 'motion':
                pending.append('native command fault: '+reason)
            if result_command:
                arm.set_targets(*result_command)
            accepted += reason == 'motion'
            measured_q = adapter.measured
        log('native', phase=phase, feasible=feasible, coverage=coverage, reason=reason,
            names=list(trajectory.joint_names), q=list(trajectory.points[0].positions),
            v=list(trajectory.points[0].velocities), measured=measured_q)
        timings.append(dict(total_s=time.perf_counter()-started,
                            fk_and_diagnostics_s=kinematics_done-started))

    tree = instrument_native(args.source.read_text(), str(args.source))
    # Replace only the ROS Node transport base; every endpoint gets a private prefix.
    import ast
    native_class = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'CuroboMpcNode')
    native_class.bases = [ast.Name(id='PrivateNode', ctx=ast.Load())]
    namespace = dict(__name__='offline_native', __file__=str(args.source), offline_sink=sink, PrivateNode=PrivateNode)
    exec(compile(ast.fix_missing_locations(tree), str(args.source), 'exec'), namespace)
    native = namespace['CuroboMpcNode']()
    executor.add_node(native)

    def measured(*frames):
        if frames:
            return np.zeros(3), np.array([0., 0., 0., 1.]), 0.
        values = arm.read()
        return np.array(values[21:24]), np.array(values[24:28]), time.time()-values[0]

    types = dict(PoseStamped=PoseStamped, Bool=Bool, String=String, Float32MultiArray=Float32MultiArray)
    required = {'linear_speed', 'vertical_speed', 'angular_speed_deg', 'speed_scale', 'rot_scale',
                'max_step', 'max_slew_deg', 'max_error', 'max_rot_error_deg', 'max_lead',
                'workspace_min', 'workspace_max', 'follower_time_constant', 'timeout',
                'zed_timeout', 'zed_recovery_timeout', 'zed_scale'}
    if set(settings) != required or settings['speed_scale'] != args.speed_scale or settings['rot_scale'] != args.rot_scale:
        raise ValueError('current limits missing or explicit scales differ')
    controller = live_controller(types, native.get_clock(), measured,
                                 dict(settings, zed_mapping_profile='review', zed_review_exercise=args.profile))
    manifest['limits'] = settings
    manifest['limits_sha256'] = digest(args.limits_json)
    manifest['initialization'] = dict(source='measured seven-joint seed; simulation reset only', q=arm.joint_state()[1],
                                      tcp=arm.tcp_pose()[0].tolist(), recorded_joints_used_as_feedback=False)
    # Readiness and ownership come from the actual native/simulation path.
    controller._zed_controller_ready = lambda: None if native._esdf_initialized and not native._tracking_fault else 'native map/state not ready'
    controller._zed_start_ready = controller._zed_controller_ready

    def publish_goal():
        p, q, _ = measured()
        if phase == 'approach':
            p, q = approach_p, approach_q
        elif controller.engaged and controller.zed_phase == 'active' and controller._zed_live():
            p, q = controller.cmd_p, controller.cmd_q
        p, q = hand_goal(p, q)
        msg = PoseStamped()
        msg.header.frame_id = 'fr3_link0'
        msg.header.stamp = native.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = p
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = q
        goal_pub.publish(msg)

    controller._publish = publish_goal

    def physics():
        # This thread only forwards independent CPU physics measurements to private DDS.
        due = time.monotonic()
        try:
            while not stop.is_set():
                values = arm.read()
                names = [f'fr3_joint{i}' for i in range(1, 8)]+['fr3_finger_joint1', 'fr3_finger_joint2']
                q, v = values[3:12], values[12:21]
                with lock:
                    adapter.feedback(names, q, round(values[0]*1e9))
                msg = JointState()
                msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(round(values[0]*1e9), 10**9)
                msg.name, msg.position, msg.velocity = names, q, v
                joint_pub.publish(msg)
                p, quat = np.array(values[21:24]), np.array(values[24:28])
                tcp = PoseStamped()
                tcp.header = msg.header
                tcp.header.frame_id = 'fr3_link0'
                tcp.pose.position.x, tcp.pose.position.y, tcp.pose.position.z = p.tolist()
                tcp.pose.orientation.x, tcp.pose.orientation.y, tcp.pose.orientation.z, tcp.pose.orientation.w = quat.tolist()
                tcp_pub.publish(tcp)
                measured_samples.append((values[0], values[2], p.tolist(), quat.tolist()))
                log('measured_sim', phase=phase, sim_s=values[2], q=q, v=v, tcp=p.tolist(), quat=quat.tolist())
                due += .01
                stop.wait(max(0., due-time.monotonic()))
        except Exception as error:
            pending.append(repr(error))
            stop.set()

    physics_thread = threading.Thread(target=physics, daemon=True)
    physics_thread.start()
    try:
        deadline = time.monotonic()+args.ready_timeout
        while not native._esdf_initialized or adapter.measured is None or native._mode_state != 'active' or native_count < 1:
            if time.monotonic() > deadline or pending:
                raise RuntimeError('native ESDF / simulated feedback readiness timeout: '+str(pending))
            publish_goal()
            time.sleep(.02)
        # Certify the exact measured seed against native constraints before any approach.
        check_started = time.monotonic()
        while feasible_count == 0:
            publish_goal()
            if time.monotonic()-check_started > 3.:
                manifest['transition'] = dict(status='failed_initial_native_constraints',
                    accepted=0, constraints=costs)
                raise RuntimeError('initial measured state has no feasible native result; see constraints.json')
            time.sleep(.02)
        readiness_first = arm.read()
        time.sleep(1.)
        readiness_last = arm.read()
        readiness_rtf = (readiness_last[2]-readiness_first[2])/(readiness_last[1]-readiness_first[1])
        manifest['readiness_sim_wall'] = readiness_rtf
        if not .9 <= readiness_rtf <= 1.1:
            raise RuntimeError('simulation timing readiness outside [0.9,1.1]')
        initial_p, initial_q = arm.tcp_pose()
        destination = interior_target(initial_p, settings)
        approach_p, approach_q = initial_p.copy(), initial_q.copy()
        phase = 'approach'
        approach_started, approach_sim = time.monotonic(), arm.n*DT
        approach_accepted = accepted
        last_step = time.monotonic()
        while np.linalg.norm(arm.tcp_pose()[0]-destination) > .003:
            if pending or native._tracking_fault or time.monotonic()-approach_started > args.approach_timeout:
                manifest['transition'] = dict(status='failed', accepted=accepted-approach_accepted,
                    initial_tcp=initial_p.tolist(), target_tcp=destination.tolist(), constraints=costs)
                raise RuntimeError('native measured approach failed: '+str(pending))
            now_step = time.monotonic()
            dt = min(.02, now_step-last_step)
            last_step = now_step
            delta = destination-approach_p
            horizontal = np.linalg.norm(delta[:2])
            if horizontal:
                delta[:2] *= min(1., settings['linear_speed']*settings['speed_scale']*dt/horizontal)
            delta[2] = np.clip(delta[2], -settings['vertical_speed']*settings['speed_scale']*dt,
                                settings['vertical_speed']*settings['speed_scale']*dt)
            approach_p += delta
            # Cartesian goals are only requests; ONLY feasible native joint outputs drive physics.
            publish_goal()
            time.sleep(.02)
        manifest['transition'] = dict(status='complete', accepted=accepted-approach_accepted,
            initial_tcp=initial_p.tolist(), target_tcp=destination.tolist(), measured_tcp=arm.tcp_pose()[0].tolist(),
            wall_s=time.monotonic()-approach_started, sim_s=arm.n*DT-approach_sim)
        if pending:
            raise RuntimeError(str(pending))
        phase = 'replay'
        replay_accepted_start = accepted
        sample_start_index = len(measured_samples)
        start_ns, events = timed_events(args.bag)
        if not events:
            raise RuntimeError('no operator events')
        epoch_ns, wall_start, sim_start = time.time_ns(), time.monotonic(), arm.n*DT
        next_tick = wall_start
        for receipt_ns, packet in events:
            due = wall_start+(receipt_ns-start_ns)*1e-9/args.rate
            while time.monotonic() < due:
                if pending:
                    raise RuntimeError(str(pending))
                controller.tick()
                publish_goal()  # measured hold heartbeat on pause; never old command
                if controller.zed_phase != 'active':
                    with lock:
                        hold = adapter.hold('operator '+controller.zed_phase)
                        if hold:
                            arm.set_targets(*hold)
                next_tick += .02
                time.sleep(max(0., min(.02, next_tick-time.monotonic())))
            lateness.append(max(0., time.monotonic()-due))
            if lateness[-1] > .1:
                raise RuntimeError('1x source receipt scheduling exceeded 100ms tolerance')
            rebased = rebase_packet(packet, start_ns, epoch_ns, args.rate)
            controller.zed_socket.queue.append(json.dumps(rebased).encode())
            controller._receive_zed()
            if not controller.engaged and controller.zed_phase == 'idle':
                # One simulated Start per take; expiry never auto-reengages.
                if not manifest.get('start_attempted') and controller._zed_live():
                    manifest['start_attempted'] = True
                    manifest['engage'] = controller.engage_zed()
            log('hand', receipt_ns=receipt_ns, source=packet, rebased=rebased,
                acquisition_lag_ns=receipt_ns-int(packet['stamp_ns']), phase=controller.zed_phase)
            if pending:
                raise RuntimeError(str(pending))
        controller.disengage()
        with lock:
            hold = adapter.hold('stopped')
            if hold:
                arm.set_targets(*hold)
        manifest['topics'] = native.get_topic_names_and_types()
        manifest['services'] = native.get_service_names_and_types()
        manifest['native_output_topic'] = native.cmd_pub.topic_name
        assert native.cmd_pub.topic_name == '/fr3_offline/arm/joint_command'
        manifest.update(native_outputs=native_count, native_feasible=feasible_count, accepted_motion=accepted-replay_accepted_start,
                        samples=len(events), wall_s=time.monotonic()-wall_start,
                        sim_s=arm.n*DT-sim_start, source_s=(events[-1][0]-start_ns)*1e-9,
                        status='complete' if accepted else 'blocked_no_accepted_motion')
        captures = [int(packet['stamp_ns']) for _, packet in events]
        manifest['hand_timing'] = dict(first_ns=captures[0], last_ns=captures[-1],
            acquisition_hz=(len(captures)-1)/((captures[-1]-captures[0])*1e-9),
            receipt_hz=(len(events)-1)/((events[-1][0]-events[0][0])*1e-9),
            paired=sum(packet.get('orientation_stamp_ns') == packet['stamp_ns'] for _, packet in events),
            acquisition_lag_ms=[(receipt-int(packet['stamp_ns']))*1e-6 for receipt, packet in events])
        manifest['wall_factor'] = manifest['source_s']/manifest['wall_s']
        manifest['sim_factor'] = manifest['sim_s']/manifest['wall_s']
        replay_samples = measured_samples[sample_start_index:]
        positions = np.asarray([sample[2] for sample in replay_samples])
        movement = float(np.max(np.ptp(positions, axis=0))) if len(positions) else 0.
        manifest['measured_tcp_span_m'] = np.ptp(positions, axis=0).tolist() if len(positions) else []
        manifest['gripper'] = 'disabled'
        manifest['max_source_lateness_s'] = max(lateness)
        manifest['profile_timings'] = dict(sink_max_s=max(t['total_s'] for t in timings),
            sink_mean_s=float(np.mean([t['total_s'] for t in timings])),
            physics_critical_section='dedicated process; no GPU/ROS/log flushing under shared state lock')
        validate_run_outcome(manifest['accepted_motion'], movement, manifest['source_s'],
            manifest['wall_s'], manifest['sim_s'], max(lateness))
    finally:
        manifest['native_outputs'] = native_count
        manifest['native_feasible'] = feasible_count
        manifest['accepted_total'] = accepted
        manifest['pf_mode'] = native._pf_mode
        manifest['sink_timings_s'] = timings
        manifest['gripper'] = 'disabled'
        manifest['topics'] = native.get_topic_names_and_types()
        manifest['services'] = native.get_service_names_and_types()
        manifest['native_output_topic'] = native.cmd_pub.topic_name
        stop.set()
        physics_thread.join(timeout=3)
        native._active = False
        executor_stopping.set()
        executor.wake()
        executor_thread.join(timeout=5)
        # Stop submission, drain callbacks, then destroy executor/node handles.
        executor._executor.shutdown(wait=True)
        executor.shutdown(timeout_sec=20)
        native.destroy_node()
        service.destroy_node()
        rclpy.shutdown()
        log_queue.put(None)
        logger_thread.join(timeout=5)
        trace.close()
        arm.close()
        manifest['physics'] = json.loads((args.output/'physics_summary.json').read_text())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--bags', type=Path, required=True)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--condition', choices=['original_sparse', 'corrected_dense'], required=True)
    parser.add_argument('--map-snapshot', type=Path, required=True)
    parser.add_argument('--source', type=Path, default=Path('/home/murilo/franka_ws/hub-curobo/node/curobo_mpc_node.py'))
    parser.add_argument('--yaml', type=Path, default=Path('/home/murilo/franka_ws/hub-curobo/node_config/fr3_vcap1.2_acc10.yml'))
    parser.add_argument('--profile', choices=['full_6d', 'translation', 'height', 'horizontal', 'reach', 'roll', 'pitch', 'yaw', 'neutral', 'cross'], default='full_6d')
    parser.add_argument('--speed-scale', type=float, required=True)
    parser.add_argument('--rot-scale', type=float, required=True)
    parser.add_argument('--limits-json', type=Path, required=True)
    parser.add_argument('--initial-state', type=Path, required=True)
    parser.add_argument('--approach-timeout', type=float, default=20.)
    parser.add_argument('--rate', type=float, default=1.)
    parser.add_argument('--domain', type=int, default=196)
    parser.add_argument('--ready-timeout', type=float, default=40.)
    parser.add_argument('--image', default='fr3-murilo-offline:20261031')
    parser.add_argument('--inside', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.rate != 1. or not 0 <= args.speed_scale <= 1 or not 0 <= args.rot_scale <= 1:
        parser.error('pilot requires 1x and scales in [0,1]')
    if not args.inside:
        args.output = args.output.resolve()
        args.output.mkdir(parents=True, exist_ok=False)
        inspect = json.loads(subprocess.check_output(['docker', 'inspect', 'fr3-zed-live-mpc']))[0]
        env = [e for e in inspect['Config']['Env'] if e.split('=', 1)[0] in
               ('PYTHONPATH', 'LD_LIBRARY_PATH', 'ROS_DISTRO', 'RMW_IMPLEMENTATION', 'SETUPTOOLS_SCM_PRETEND_VERSION')]
        command = ['docker', 'run', '--rm', '--network', 'none', '--ipc', 'private', '--gpus', 'all',
                   '-e', f'ROS_DOMAIN_ID={args.domain}', '-e', 'PYTHONDONTWRITEBYTECODE=1',
                   '-v', f'{REPO}:{REPO}:ro', '-v', f'{args.output}:{args.output}:rw']
        for item in env:
            command += ['-e', item]
        for mount in inspect['Mounts']:
            if not mount['RW']:
                command += ['-v', f"{mount['Source']}:{mount['Destination']}:ro"]
        # Source paths remain readable at their explicit host locations too.
        command += ['-v', '/home/murilo/franka_ws/hub-curobo:/home/murilo/franka_ws/hub-curobo:ro',
                    '--entrypoint', '/isaac-sim/python.sh', args.image, str(Path(__file__).resolve()),
                    *sys.argv[1:], '--inside']
        image_id = subprocess.check_output(['docker', 'image', 'inspect', args.image, '--format', '{{.Id}}'], text=True).strip()
        (args.output/'launch.json').write_text(json.dumps(dict(command=command, image_id=image_id,
            network='none', ipc='private', host_udp=False), indent=2))
        with (args.output/'pilot.log').open('x') as log:
            result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, timeout=240)
        print(json.dumps(dict(returncode=result.returncode, log=str(args.output/'pilot.log'), manifest=str(args.output/'manifest.json'))))
        return result.returncode
    launch = json.loads((args.output/'launch.json').read_text())
    if not Path('/.dockerenv').exists() or any(line.split()[1] != '00000000' for line in Path('/proc/net/route').read_text().splitlines()[1:]):
        raise RuntimeError('inside mode requires the network-disabled child container')
    manifest = dict(image_id=launch['image_id'], adapter_sha256=digest(HERE/'murilo_sim_adapter.py'),
                    runner_sha256=digest(__file__), live_controller_sha256=digest(FR3/'teleop/fr3_input_teleop.py'),
                    solver='native Murilo MPC', simulation_only=True, physical_commands=False,
                    condition=args.condition, profile=args.profile, rate=args.rate,
                    takes=enumerate_takes(args.bags), source_sha256=digest(args.source),
                    yaml_sha256=digest(args.yaml), bag=str(args.bag), speed_scale=args.speed_scale,
                    rot_scale=args.rot_scale, status='starting')
    try:
        pilot(args, manifest)
    except Exception as error:
        manifest.update(status='blocked', error=repr(error))
        raise
    finally:
        (args.output/'manifest.json').write_text(json.dumps(manifest, indent=2))
        print(json.dumps(manifest))
    return 0 if manifest['status'] == 'complete' else 2


if __name__ == '__main__':
    sys.exit(main())
