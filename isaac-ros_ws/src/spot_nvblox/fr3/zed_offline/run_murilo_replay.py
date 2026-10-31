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
                                    instrument_native, rebase_packet)
    sys.path.insert(0, str(FR3/'remote_sim'))
    from mujoco_remote_arm import MujocoArm, DT

    snapshot = json.loads(args.map_snapshot.read_text())
    response = deserialize_message(base64.b64decode(snapshot['cdr']), EsdfAndGradients.Response)
    if not response.success or abs(response.voxel_size_m-.05) > 1e-6 or not response.esdf_and_gradients.data:
        raise ValueError('invalid or empty ESDF snapshot')
    manifest['map'] = {k: v for k, v in snapshot.items() if k != 'cdr'}
    manifest['map'].update(sha256=digest(args.map_snapshot), environment='current snapshot, not historical',
                           replay_policy='immutable offline snapshot; refresh is local serving time')
    rclpy.init(domain_id=args.domain, args=['--ros-args'] + sum((['-p', f'{k}:={v}'] for k, v in {
        'robot':'franka', 'base_frame':'fr3_link0', 'robot_config':str(args.yaml),
        'use_sim':'false', 'use_ros2_control':'false', 'feedback_mode':'measured',
        'start_active':'true', 'use_esdf':'true', 'voxel_size':'0.05',
        'grid_size_m':'[1.4,1.6,1.2]', 'grid_center_m':'[0.35,0.0,0.55]',
        'esdf_service_name':'/snapshot', 'esdf_frame_id':'fr3_link0', 'esdf_global_frame':'fr3_link0',
        'pf_mode':'none', 'esdf_update_rate':'4.0', 'command_type':'joint_trajectory',
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
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    arm = MujocoArm()
    lock = threading.RLock()
    adapter = TrajectoryAdapter()
    trace = (args.output/'trace.jsonl').open('x')
    pending = []
    native_count = 0
    accepted = 0
    controller = None
    stop = threading.Event()

    def log(event, **fields):
        trace.write(json.dumps(dict(event=event, wall_ns=time.time_ns(), **fields), allow_nan=False)+'\n')
        trace.flush()

    def sink(node, trajectory, feasible, state):
        nonlocal native_count, accepted
        with lock:
            if stop.is_set():
                return
            node.cmd_pub.publish(trajectory)  # native output stays in the private DDS graph
            native_count += 1
            spheres = node.mpc.rollout_fn.compute_kinematics(state).robot_spheres.detach().cpu().numpy().reshape(-1, 4)
            spheres = spheres[spheres[:, 3] > 0]
            lo = np.asarray(getattr(node, '_last_grid_bb_min', [np.inf]*3))
            hi = np.asarray(getattr(node, '_last_grid_bb_max', [-np.inf]*3))
            coverage = bool(len(spheres) and np.all(spheres[:, :3]-spheres[:, 3:] >= lo)
                            and np.all(spheres[:, :3]+spheres[:, 3:] <= hi))
            now = time.time_ns()
            goal_ns = 0 if node._pose_msg is None else (node._pose_msg.header.stamp.sec*10**9+node._pose_msg.header.stamp.nanosec)
            result = adapter.convert(trajectory, now_ns=now, feasible=feasible,
                                     map_ready=node._esdf_initialized, map_ns=map_served_ns,
                                     goal_ns=goal_ns, coverage=coverage,
                                     active=controller is not None and controller.engaged and controller.zed_phase == 'active',
                                     owners=len(node.get_publishers_info_by_topic('/fr3_offline/arm/joint_command')))
            if result:
                arm.set_targets(*result)
            accepted += adapter.reason == 'motion'
            log('native', feasible=feasible, coverage=coverage, reason=adapter.reason,
                names=list(trajectory.joint_names), q=list(trajectory.points[0].positions),
                v=list(trajectory.points[0].velocities), measured=adapter.measured)

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
        with lock:
            p, q = arm.tcp_pose()
            return p.copy(), q.copy(), (time.time_ns()-adapter.state_ns)*1e-9

    types = dict(PoseStamped=PoseStamped, Bool=Bool, String=String, Float32MultiArray=Float32MultiArray)
    settings = json.loads(args.limits_json.read_text())
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
    manifest['initialization'] = dict(source='simulator HOME reset', q=arm.joint_state()[1],
                                      tcp=arm.tcp_pose()[0].tolist(), recorded_joints_used_as_feedback=False)
    # Readiness and ownership come from the actual native/simulation path.
    controller._zed_controller_ready = lambda: None if native._esdf_initialized and not native._tracking_fault else 'native map/state not ready'
    controller._zed_start_ready = controller._zed_controller_ready

    def publish_goal():
        with lock:
            p, q, _ = measured()
            if controller.engaged and controller.zed_phase == 'active' and controller._zed_live():
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
        due = time.monotonic()
        try:
            while not stop.is_set():
                with lock:
                    arm.step()
                    # Stop on robot/environment contact; never drive through it.
                    for contact in arm.data.contact[:arm.data.ncon]:
                        import mujoco
                        bodies = [int(arm.model.geom_bodyid[g]) for g in (contact.geom1, contact.geom2)]
                        labels = [mujoco.mj_id2name(arm.model, mujoco.mjtObj.mjOBJ_BODY, b) or '' for b in bodies]
                        moving = [label.startswith('fr3_') and label != 'fr3_link0' for label in labels]
                        robot = [label.startswith('fr3_') for label in labels]
                        if (moving[0] and not robot[1]) or (moving[1] and not robot[0]):
                            raise RuntimeError('simulated robot/environment contact: '+str(labels))
                    names, q, v = arm.joint_state()
                    now = time.time_ns()
                    adapter.feedback(names, q, now)
                    msg = JointState()
                    msg.header.stamp = native.get_clock().now().to_msg()
                    msg.name, msg.position, msg.velocity = names, q, v
                    joint_pub.publish(msg)
                    if arm.n % 4 == 0:
                        p, quat = arm.tcp_pose()
                        tcp = PoseStamped()
                        tcp.header = msg.header
                        tcp.header.frame_id = 'fr3_link0'
                        tcp.pose.position.x, tcp.pose.position.y, tcp.pose.position.z = p.tolist()
                        tcp.pose.orientation.x, tcp.pose.orientation.y, tcp.pose.orientation.z, tcp.pose.orientation.w = quat.tolist()
                        tcp_pub.publish(tcp)
                        log('measured_sim', sim_s=arm.n*DT, q=q, v=v, tcp=p.tolist(), quat=quat.tolist())
                due += DT
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
        manifest.update(native_outputs=native_count, accepted_motion=accepted,
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
    finally:
        stop.set()
        physics_thread.join(timeout=3)
        native._active = False
        executor.shutdown(timeout_sec=20)
        executor_thread.join(timeout=3)
        # Bundled Jazzy shutdown returns before the worker pool drains.
        executor._executor.shutdown(wait=True)
        native.destroy_node()
        service.destroy_node()
        rclpy.shutdown()
        trace.close()
        arm.close()


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
            result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, timeout=180)
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
