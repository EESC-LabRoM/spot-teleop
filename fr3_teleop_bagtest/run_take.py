#!/usr/bin/env python3
"""Run one ZED protocol stage as a real-time camera driving the real FR3.

Dry run (default): replays the stage bag through a NEW test tracker +
relay (dedicated topics/ports; no live process is touched) and checks the
movement-recognition pipeline: sample flow, valid fractions, re-inference
drift vs the recorded samples. The robot never moves.

Robot run (--execute): additionally starts a test teleop (dedicated UDP
port), engages it on the replayed hand with the stage exercise profile,
records the robot motion, stops, homes, and CHECKs the motion against the
sim-validated stage spec. Requires a person at the e-stop and ownership of
the franka-robot resource.

  python3 run_take.py --stage 04_frente_tras --out takes/04_take1 [--execute]

Only processes started by this script are ever stopped (matched by exact
test-only command-line markers). The live tracker/relay/teleop are never
restarted or remapped.
"""
import argparse
import json
import math
import os
import re
import shutil
import signal
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from bagtime import (STAGES, compare_take, check_line, recognition_verdict,
                         sample_flow_metrics)  # noqa: E402

HERE = Path(__file__).resolve().parent
BAGS = Path(os.environ.get('BAGTEST_BAGS', '/home/hipolito/spot-teleop/bags/zed_protocol_20261030_005244'))
REPO_TELEOP = Path(os.environ.get('BAGTEST_REPO_TELEOP',
                                  '/home/hipolito/spot-teleop/isaac-ros_ws/src/spot_nvblox/fr3/teleop'))
TRACKER = '/home/spot-teleop/isaac-ros_ws/src/spot_nvblox/fr3/remote_sim/operator_hand_tracker.py'
RELAY = '/home/spot-teleop/isaac-ros_ws/src/spot_nvblox/fr3/teleop/zed_input_relay.py'

ZED_PORT = 47817          # test teleop UDP (live uses 47813)
CAL_PORT = 47818          # test tracker calibration (live uses 47815)
CAM_PAGE = 8793           # bag camera visualization
TRACKER_PREVIEW = 8790    # test tracker overlay (live uses 8786; 8796 is taken)

HOME_TCP = [0.3069, 0.0, 0.4865]
HOME_TOL_M = 0.04
POST_STOP_DWELL_S = 5.0  # disengaged hold window before home (recorders run; no motion)

HUMBLE = 'source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=8; '
JAZZY = ('source /opt/ros/jazzy/setup.bash; source /ros2_ws/install/setup.bash; '
         'export ROS_DOMAIN_ID=77 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST; cd /ros2_ws; ')


def log(msg):
    print('[run_take] %s' % msg, flush=True)


def sh(cmd, timeout=60, check=False):
    p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    if check and p.returncode:
        raise RuntimeError('%s failed: %s' % (cmd[:4], (p.stdout + p.stderr)[-500:]))
    return p


def dexec(container, cmd, timeout=60, check=False):
    return sh(['docker', 'exec', container, 'bash', '-c', cmd], timeout=timeout, check=check)


def self_safe(pattern):
    """Bracket the first char so pgrep/pkill never match their own command."""
    return '[' + pattern[0] + ']' + pattern[1:]


def dexec_detached(container, cmd, clog):
    """Start detached; process output goes to the container-side log CLOG."""
    wrapped = '%s > %s 2>&1' % (cmd, clog)
    subprocess.run(['docker', 'exec', '-d', container, 'bash', '-c', wrapped], check=True)


def http_json(url, timeout=5):
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return json.load(r)


def http_post(url, payload, header):
    req = urllib.request.Request(url, data=json.dumps(payload).encode(),
                                 headers={'Content-Type': 'application/json', header: '1'})
    with urllib.request.urlopen(req, timeout=5) as r:
        return r.status


def bag_duration(bag_d8):
    meta = (bag_d8 / 'metadata.yaml').read_text()
    m = re.search(r'duration:\s*\n\s*nanoseconds:\s*(\d+)', meta)
    if not m:
        raise RuntimeError('no duration in metadata.yaml')
    return int(m.group(1)) * 1e-9


def container_running(name):
    p = sh(['docker', 'inspect', '-f', '{{.State.Running}}', name], timeout=15)
    return p.returncode == 0 and p.stdout.strip() == 'true'


def procs_matching(container, pattern):
    p = dexec(container, 'pgrep -af "%s" || true' % self_safe(pattern), timeout=15)
    return [l for l in p.stdout.splitlines() if l.strip()]


class RobotLink:
    """ stdin/stdout helper (robot_side.py serve) inside fr3-ros2. """

    def __init__(self):
        self.proc = subprocess.Popen(
            ['docker', 'exec', '-i', 'fr3-ros2', 'bash', '-c',
             JAZZY + 'exec python3 /tmp/bagtest_robot_side.py serve'],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL, text=True, bufsize=1)
        self.status = {}
        self._buf = []

    def poll(self, timeout=5.0):
        import select
        end = time.time() + timeout
        while time.time() < end:
            r, _, _ = select.select([self.proc.stdout], [], [], 0.2)
            if r:
                line = self.proc.stdout.readline()
                if not line:
                    break
                try:
                    e = json.loads(line)
                except ValueError:
                    continue
                if e.get('event') == 'status':
                    self.status = e
                else:
                    self._buf.append(e)
            if self.status:
                return self.status
        return self.status

    def send(self, cmd):
        self.proc.stdin.write(cmd + '\n')
        self.proc.stdin.flush()

    def wait_phase(self, cond, timeout, what):
        end = time.time() + timeout
        while time.time() < end:
            st = self.poll(timeout=2.0)
            if cond(st):
                return st
        raise RuntimeError('timeout waiting for ' + what)

    def close(self):
        try:
            self.proc.terminate()
        except OSError:
            pass


def preflight(args, stage_dir):
    log('preflight: stage=%s execute=%s' % (args.stage, args.execute))
    if args.stage not in STAGES:
        raise RuntimeError('unknown stage (need one of %s)' % sorted(STAGES))
    d8 = stage_dir / 'd8'
    if not (d8 / 'metadata.yaml').is_file():
        raise RuntimeError('bag d8 missing: %s' % d8)
    duration = bag_duration(d8)
    log('bag d8 duration %.1fs' % duration)
    for c in ('fr3-ros2', 'fr3-curobo'):
        if not container_running(c):
            raise RuntimeError('container %s not running' % c)
    # Live perception must be idle: a flowing live hand means a human session.
    p = dexec('fr3-curobo', HUMBLE + 'timeout 6 ros2 topic hz --window 20 /fr3_operator/teleop_sample 2>/dev/null | head -2',
              timeout=20)
    if 'average rate' in p.stdout:
        raise RuntimeError('live /fr3_operator/teleop_sample is flowing; a human session may be active')
    log('live hand topics idle')
    if args.persistent_tracker:
        if not procs_matching('fr3-curobo', 'operator_hand_tracker.*fr3_bagtest'):
            raise RuntimeError('persistent test chain not running; start it with --chain up')
        try:
            http_json('http://127.0.0.1:%d/state' % CAM_PAGE, timeout=5)
        except Exception:
            raise RuntimeError('persistent bag camera not answering on %d' % CAM_PAGE)
        log('persistent test chain alive')
    else:
        # Test ports must be free (single-take mode starts its own chain).
        p = sh(['ss', '-lun'], timeout=10)
        for port in (ZED_PORT, CAL_PORT):
            if re.search(r':%d\b' % port, p.stdout):
                raise RuntimeError('UDP port %d busy' % port)
        p = sh(['ss', '-tln'], timeout=10)
        for port in (CAM_PAGE, TRACKER_PREVIEW):
            if re.search(r':%d\b' % port, p.stdout):
                raise RuntimeError('TCP port %d busy' % port)
    if args.execute:
        teles = procs_matching('fr3-ros2', 'fr3_input_teleop')
        teles += [t for t in procs_matching('fr3-ros2', 'fr3_joy_teleop.py') if t not in teles]
        foreign = [t for t in teles if 'zed_port:=%d' % ZED_PORT not in t]
        if foreign:
            raise RuntimeError('a foreign teleop runs in fr3-ros2; refusing a second writer: %s' % foreign)
        # Refuse only when the arm is actually being driven: an idle teleop
        # (ours from a previous take, disengaged) holds a publisher but sends
        # nothing, and the take restarts it. Any live flow means hold.
        p = dexec('fr3-ros2', JAZZY + 'timeout 8 ros2 topic hz --window 5 '
                  '/cartesian_impedance_target_controller/equilibrium_pose 2>/dev/null', timeout=20)
        if 'average rate' in p.stdout:
            raise RuntimeError('equilibrium_pose is streaming; someone drives the arm: '
                               + p.stdout.strip().splitlines()[-1][:100])
        p = dexec('fr3-ros2', JAZZY + 'timeout 20 ros2 control list_controllers 2>/dev/null', timeout=30)
        row = next((l for l in p.stdout.splitlines() if 'cartesian_impedance_target_controller' in l), '')
        log('controller row: %s' % row.strip()[:120])
        if 'active' not in row:
            raise RuntimeError('cartesian impedance controller not active; activate it explicitly first')
        log('no teleop process running; robot path clear')
    return duration


def deploy_files():
    for name in ('bag_camera.py', 'bagtime.py', 'record_jsonl.py'):
        src = HERE / name
        sh(['docker', 'cp', str(src), 'fr3-curobo:/tmp/%s' % name], timeout=30, check=True)
    sh(['docker', 'cp', str(HERE / 'record_jsonl.py'), 'fr3-ros2:/tmp/bagtest_record_jsonl.py'],
       timeout=30, check=True)
    sh(['docker', 'cp', str(REPO_TELEOP / 'zed_protocol' / 'robot_side.py'),
        'fr3-ros2:/tmp/bagtest_robot_side.py'], timeout=30, check=True)
    dexec('fr3-curobo', 'mkdir -p /tmp/bagtest', check=True)
    dexec('fr3-ros2', 'mkdir -p /tmp/bagtest', check=True)


# Test teleop: canonical /lab/fr3_joy_teleop.yaml plus test-only overrides
# (dedicated UDP/calibration ports, ZED input, Oct-31 replay box).
TELEOP_CMD = (
    'exec python3 /lab/fr3_zed_runtime/fr3_input_teleop.py --ros-args '
    '--params-file /lab/fr3_joy_teleop.yaml -p joy_device:=auto -p input_source:=zed '
    '-p zed_port:=%d -p zed_calibration_port:=%d '
    '-p workspace_min:=[0.20,-0.35,0.22] -p workspace_max:=[0.65,0.35,0.70]'
    % (ZED_PORT, CAL_PORT))

# Same site packages as the live tracker (mediapipe/cv2 live outside ROS).
TRACKER_ENV = ('export PYTHONPATH=/home/spot-teleop/tmp/cv_site_hipolito:'
               '/home/spot-teleop/tmp/mp_site_hipolito:'
               '/home/spot-teleop/spot-ros2_ws/src/arm_pose_estimator:${PYTHONPATH:-}; ')
TRACKER_CMD = (
    'exec python3 -u %s --domain 8 --calibrate-arm --smooth-preview '
    '--preview-port %d --calibration-port %d --output-prefix /fr3_bagtest '
    '--max-input-age .25 --max-output-age .35 --ros-args '
    '-r /fr3_sim/operator/rgb:=/fr3_bagtest/rgb '
    '-r /fr3_sim/operator/depth:=/fr3_bagtest/depth '
    '-r /fr3_sim/operator/camera_info:=/fr3_bagtest/camera_info '
    '-r /fr3_bagtest/sample:=/fr3_bagtest/teleop_sample' % (TRACKER, TRACKER_PREVIEW, CAL_PORT))

RELAY_CMD = ('exec python3 %s --orientation-mode full_6d --port %d --ros-args '
             '-r /fr3_operator/teleop_sample:=/fr3_bagtest/teleop_sample '
             '-r /fr3_operator/tracking:=/fr3_bagtest/tracking' % (RELAY, ZED_PORT))


def start_chain(args, stage_dir, outdir):
    (outdir / 'logs').mkdir(parents=True, exist_ok=True)
    dexec('fr3-curobo', 'rm -f /tmp/bagtest/samples.jsonl', timeout=15)
    dexec('fr3-ros2', 'rm -f /tmp/bagtest/robot.jsonl', timeout=15)
    dexec_detached('fr3-curobo',
                   HUMBLE + 'exec python3 /tmp/record_jsonl.py --domain 8 --out /tmp/bagtest/samples.jsonl '
                   '--str /fr3_bagtest/teleop_sample /fr3_bagtest/recorded_sample /fr3_bagtest/tracking',
                   '/tmp/bagtest/rec_samples.log')
    if args.execute:
        dexec_detached('fr3-ros2',
                       JAZZY + 'exec python3 /tmp/bagtest_record_jsonl.py --domain 77 --out /tmp/bagtest/robot.jsonl '
                       '--str /teleop/zed_status --bool /teleop/engaged '
                       '--pose /franka_robot_state_broadcaster/current_pose /teleop/command_tool '
                       '--joints /joint_states /franka_gripper/joint_states',
                       '/tmp/bagtest/rec_robot.log')
    if args.persistent_tracker:
        log('using persistent test chain')
        return
    dexec_detached('fr3-curobo', HUMBLE + TRACKER_ENV + TRACKER_CMD, '/tmp/bagtest/tracker.log')
    dexec_detached('fr3-curobo', HUMBLE + RELAY_CMD, '/tmp/bagtest/relay.log')
    bag_in_container = '/home/spot-teleop/bags/zed_protocol_20261030_005244/%s' % args.stage
    dexec_detached('fr3-curobo',
                   HUMBLE + 'exec python3 /tmp/bag_camera.py %s --port %d' % (bag_in_container, CAM_PAGE),
                   '/tmp/bagtest/camera.log')
    deadline = time.time() + 30
    while time.time() < deadline:
        try:
            st = http_json('http://127.0.0.1:%d/state' % CAM_PAGE, timeout=3)
            log('bag camera ready: %s (%.1fs)' % (st['bag'], st['duration_s']))
            break
        except Exception:
            time.sleep(1.0)
    else:
        raise RuntimeError('bag camera did not come up; check fr3-curobo:/tmp/bagtest/camera.log')
    # The tracker needs ~10 s for MediaPipe init; playing a short bag
    # before it is ready yields zero samples (stage 09 is only 6.6 s).
    deadline = time.time() + 90
    while time.time() < deadline:
        p = dexec('fr3-curobo', 'grep -c "DIRECT HAND TRACKER ready" /tmp/bagtest/tracker.log', timeout=15)
        if p.stdout.strip().startswith('1'):
            log('test tracker ready')
            return
        time.sleep(2.0)
    raise RuntimeError('test tracker did not initialize; check fr3-curobo:/tmp/bagtest/tracker.log')


def load_stage_bag(stage):
    """Point the persistent camera at this stage's bag (same publishers)."""
    bag_in_container = '/home/spot-teleop/bags/zed_protocol_20261030_005244/%s' % stage
    http_post('http://127.0.0.1:%d/control' % CAM_PAGE, dict(action='load', bag=bag_in_container),
              'X-Bag-Camera')
    deadline = time.time() + 30
    while time.time() < deadline:
        st = http_json('http://127.0.0.1:%d/state' % CAM_PAGE, timeout=5)
        if st['bag'].endswith('/%s/d8' % stage) and st['duration_s'] > 0 and not st['playing']:
            log('bag camera loaded: %s (%.1fs)' % (stage, st['duration_s']))
            return st['duration_s']
        time.sleep(0.5)
    raise RuntimeError('bag camera did not load %s' % stage)


CHAIN_PROCS = (('fr3-curobo', 'fr3_bagtest'),
               ('fr3-curobo', 'bag_camera.py'),
               ('fr3-curobo', '/tmp/record_jsonl.py'),
               ('fr3-ros2', '/tmp/bagtest_record_jsonl.py'))
TRACKER_PROCS = (('fr3-curobo', 'operator_hand_tracker.*fr3_bagtest'),
                 ('fr3-curobo', 'zed_input_relay.*47817'))


def stop_procs(procs, verify=()):
    for container, pattern in procs:
        dexec(container, 'pkill -f "%s" || true' % self_safe(pattern), timeout=15)
    time.sleep(3.0)
    strays = []
    for container, pattern in (verify or procs):
        strays += ['%s:%s' % (container, l) for l in procs_matching(container, pattern)]
    if strays:  # SIGTERM grace expired; escalate once, then verify
        for container, pattern in (verify or procs):
            dexec(container, 'pkill -9 -f "%s" || true' % self_safe(pattern), timeout=15)
        time.sleep(1.0)
        strays = []
        for container, pattern in (verify or procs):
            strays += ['%s:%s' % (container, l) for l in procs_matching(container, pattern)]
    if strays:
        raise RuntimeError('test processes still alive: %s' % strays)


PER_TAKE_PROCS = (('fr3-curobo', '/tmp/record_jsonl.py'),
                  ('fr3-ros2', '/tmp/bagtest_record_jsonl.py'))

# Per-take stdin/stdout helper: RobotLink.close() only kills the host-side
# docker-exec client, so the in-container serve keeps spinning (~43% CPU)
# unless reaped here. It matches no other cleanup pattern, so list it.
SERVE_PROCS = (('fr3-ros2', 'bagtest_robot_side.py serve'),)


def take_cleanup_procs(persistent_tracker):
    base = PER_TAKE_PROCS if persistent_tracker else CHAIN_PROCS
    return base + SERVE_PROCS


def stop_chain(persistent_tracker=False):
    # Persistent chain (tracker+relay+camera) survives takes; only the
    # per-take recorders stop. The recorder cmdlines match neither the
    # tracker nor the relay nor the camera.
    procs = take_cleanup_procs(persistent_tracker)
    stop_procs(procs, procs)


def chain_up(stage):
    if procs_matching('fr3-curobo', 'operator_hand_tracker.*fr3_bagtest'):
        raise RuntimeError('a test chain already runs; stop it first')
    dexec('fr3-curobo', 'rm -f /tmp/bagtest/tracker.log /tmp/bagtest/relay.log /tmp/bagtest/camera.log', timeout=15)
    dexec_detached('fr3-curobo', HUMBLE + TRACKER_ENV + TRACKER_CMD, '/tmp/bagtest/tracker.log')
    dexec_detached('fr3-curobo', HUMBLE + RELAY_CMD, '/tmp/bagtest/relay.log')
    bag_in_container = '/home/spot-teleop/bags/zed_protocol_20261030_005244/%s' % stage
    dexec_detached('fr3-curobo',
                   HUMBLE + 'exec python3 /tmp/bag_camera.py %s --port %d' % (bag_in_container, CAM_PAGE),
                   '/tmp/bagtest/camera.log')
    deadline = time.time() + 90
    while time.time() < deadline:
        p = dexec('fr3-curobo', 'grep -c "DIRECT HAND TRACKER ready" /tmp/bagtest/tracker.log', timeout=15)
        ready = p.stdout.strip().startswith('1')
        try:
            st = http_json('http://127.0.0.1:%d/state' % CAM_PAGE, timeout=3)
            cam = st['bag'].endswith('/%s/d8' % stage)
        except Exception:
            cam = False
        if ready and cam:
            log('persistent test chain ready (tracker + camera on %s)' % stage)
            return
        time.sleep(2.0)
    raise RuntimeError('test chain did not come up; check fr3-curobo:/tmp/bagtest/*.log')


CHAIN_DOWN_PROCS = (TRACKER_PROCS + (('fr3-curobo', 'bag_camera.py'),
                                     ('fr3-curobo', '/tmp/record_jsonl.py'),
                                     ('fr3-ros2', '/tmp/bagtest_record_jsonl.py')) +
                    SERVE_PROCS)


def chain_down():
    stop_procs(CHAIN_DOWN_PROCS +
               (('fr3-ros2', 'zed_port:=%d' % ZED_PORT),))


def stop_teleop():
    dexec('fr3-ros2', 'pkill -f "%s" || true' % self_safe('zed_port:=%d' % ZED_PORT), timeout=15)
    time.sleep(1.0)
    if procs_matching('fr3-ros2', 'zed_port:=%d' % ZED_PORT):
        raise RuntimeError('test teleop did not stop')


def grep_count(container, path, pattern):
    p = dexec(container, 'grep -c "%s" %s 2>/dev/null || true' % (pattern, path), timeout=15)
    try:
        return int(p.stdout.strip().splitlines()[0])
    except (ValueError, IndexError):
        return 0


def collect_files(outdir):
    for container, src, dst in (
            ('fr3-curobo', '/tmp/bagtest/samples.jsonl', 'samples.jsonl'),
            ('fr3-ros2', '/tmp/bagtest/robot.jsonl', 'robot.jsonl')):
        p = sh(['docker', 'cp', '%s:%s' % (container, src), str(outdir / dst)], timeout=30)
        if p.returncode:
            log('collect %s:%s failed (may not exist in dry mode)' % (container, src))
    for container, src in (('fr3-curobo', '/tmp/bagtest/rec_samples.log'),
                           ('fr3-curobo', '/tmp/bagtest/tracker.log'),
                           ('fr3-curobo', '/tmp/bagtest/relay.log'),
                           ('fr3-curobo', '/tmp/bagtest/camera.log'),
                           ('fr3-ros2', '/tmp/bagtest/rec_robot.log'),
                           ('fr3-ros2', '/tmp/bagtest/teleop.log')):
        name = Path(src).name
        sh(['docker', 'cp', '%s:%s' % (container, src), str(outdir / 'logs' / name)], timeout=30)


def parse_samples(path, scored_from=0.0):
    new, recorded = [], []
    if not path.is_file():
        return new, recorded
    t0 = None
    for line in path.read_text().splitlines():
        try:
            e = json.loads(line)
        except ValueError:
            continue
        if e.get('wall_t', 0.0) < scored_from:
            continue  # warm-up pass samples do not count
        if e.get('topic') == '/fr3_bagtest/teleop_sample':
            d = e['data'] if isinstance(e.get('data'), dict) else {}
            if t0 is None:
                t0 = e['wall_t']
            if d.get('position_valid') is True and isinstance(d.get('p'), list):
                q = d.get('q') if d.get('orientation_valid') is True and isinstance(d.get('q'), list) else None
                new.append(dict(offset_s=0.0, wall_t=e['wall_t'], p=d['p'], q=q,
                                orientation_valid=q is not None,
                                gesture=d.get('gesture')))
        elif e.get('topic') == '/fr3_bagtest/recorded_sample':
            d = e['data'] if isinstance(e.get('data'), dict) else {}
            if d.get('position_valid') is True and isinstance(d.get('p'), list):
                q = d.get('q') if d.get('orientation_valid') is True and isinstance(d.get('q'), list) else None
                recorded.append(dict(offset_s=0.0, wall_t=e['wall_t'], p=d['p'], q=q))
    # Both streams flow simultaneously in wall time, so wall_t is the common
    # clock; offsets only need a shared origin for the drift matcher.
    t0 = min([s['wall_t'] for s in new] + [r['wall_t'] for r in recorded], default=0.0)
    for s in new:
        s['offset_s'] = s['wall_t'] - t0
    for r in recorded:
        r['offset_s'] = r['wall_t'] - t0
    return new, recorded


def parse_robot(path, scored_from=0.0, scored_to=None):
    series, grips, engaged = [], [], False
    if not path.is_file():
        return series, grips, engaged
    for line in path.read_text().splitlines():
        try:
            e = json.loads(line)
        except ValueError:
            continue
        if e.get('wall_t', 0.0) < scored_from:
            continue
        if scored_to is not None and e.get('wall_t', 0.0) > scored_to:
            continue
        if e.get('topic') == '/franka_robot_state_broadcaster/current_pose':
            d = e['data']
            # The broadcaster labels base-frame poses 'base' (verified equal
            # to fr3_link0->fr3_hand_tcp TF on the live robot).
            if d.get('frame') not in ('fr3_link0', 'base', ''):
                raise RuntimeError('unexpected TCP frame: %s' % d.get('frame'))
            series.append(dict(t=e['wall_t'], p=d['p'], q=d['q']))
        elif e.get('topic') == '/franka_gripper/joint_states':
            pos = (e['data'] or {}).get('position') or []
            if pos:
                grips.append(float(pos[0]))
        elif e.get('topic') == '/teleop/engaged' and e.get('data') is True:
            engaged = True
    return series, grips, engaged


def parse_post_stop(path, scored_from=0.0, t_stop_confirmed=None, t_home_start=None):
    """Post-stop evidence: last target vs TCP at disengage + hold drift.

    Checks the retained-target concern: after the teleop disengages, the
    impedance controller must hold (not drift toward a stale target) and no
    new command_tool targets may appear. Informational only (never raises on
    missing data); fields are None with ok=False when the log lacks them.
    """
    res = dict(ok=False, t_disengage=None, target_frame=None,
               tracking_err_at_stop_cm=None, post_stop_drift_cm=None,
               commands_after_stop=None, n_hold_points=0, detail='')
    if not path.is_file():
        res['detail'] = 'no robot.jsonl'
        return res
    saw_true = False
    disengage = None
    targets, poses = [], []  # (wall_t, p[, frame])
    for line in path.read_text().splitlines():
        try:
            e = json.loads(line)
        except ValueError:
            continue
        if e.get('wall_t', 0.0) < scored_from:
            continue
        topic = e.get('topic')
        if topic == '/teleop/engaged':
            if e.get('data') is True:
                saw_true = True
            elif e.get('data') is False and saw_true and disengage is None:
                disengage = e['wall_t']
        elif topic == '/teleop/command_tool':
            d = e.get('data') or {}
            if isinstance(d.get('p'), list) and len(d['p']) == 3:
                targets.append((e['wall_t'], d['p'], d.get('frame')))
        elif topic == '/franka_robot_state_broadcaster/current_pose':
            d = e.get('data') or {}
            if isinstance(d.get('p'), list) and len(d['p']) == 3:
                poses.append((e['wall_t'], d['p']))
    if disengage is None:
        res['detail'] = 'no engaged True->False transition after scored_from'
        return res
    res['t_disengage'] = disengage
    tgt_before = [x for x in targets if x[0] <= disengage]
    tcp_before = [x for x in poses if x[0] <= disengage]
    if not tgt_before or not tcp_before:
        res['detail'] = 'no target or TCP sample at/before disengage'
        return res
    last_target, tcp_at_stop, res['target_frame'] = tgt_before[-1][1], tcp_before[-1][1], tgt_before[-1][2]
    res['tracking_err_at_stop_cm'] = round(math.dist(last_target, tcp_at_stop) * 100, 2)
    hold_end = t_home_start or (disengage + 1.0)
    hold = [p for (t, p) in poses if disengage < t <= hold_end]
    res['n_hold_points'] = len(hold)
    if hold:
        res['post_stop_drift_cm'] = round(max(math.dist(p, tcp_at_stop) for p in hold) * 100, 2)
    res['commands_after_stop'] = sum(1 for (t, _, _) in targets if disengage < t <= hold_end)
    res['ok'] = True
    res['detail'] = 'hold window %.1fs' % (hold_end - disengage)
    return res


def post_stop_line(post):
    if not post.get('ok'):
        return 'post-stop: NO DATA (%s)' % post.get('detail')
    drift = post['post_stop_drift_cm']
    return ('post-stop: track_err=%.2fcm drift=%s(n=%d) cmds_after_stop=%d frame=%s [%s]'
            % (post['tracking_err_at_stop_cm'],
               'n/a' if drift is None else '%.2fcm' % drift,
               post['n_hold_points'], post['commands_after_stop'],
               post['target_frame'], post['detail']))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--stage', default=None)
    ap.add_argument('--out', type=Path, default=None)
    ap.add_argument('--execute', action='store_true',
                    help='robot follows the bag (needs e-stop + franka-robot ownership)')
    ap.add_argument('--keep-teleop', action='store_true',
                    help='leave the test teleop running for the next take')
    ap.add_argument('--persistent-tracker', action='store_true',
                    help='use the persistent session chain (started with --chain up)')
    ap.add_argument('--chain', choices=['up', 'down'], default=None,
                    help='start/stop the persistent session chain (tracker+relay+camera), then exit')
    args = ap.parse_args()
    if args.chain == 'up':
        deploy_files()
        chain_up(args.stage or '00_neutro')
        return
    if args.chain == 'down':
        chain_down()
        return
    if not args.stage or not args.out:
        ap.error('--stage and --out are required for a take')
    stage_dir = BAGS / args.stage
    duration = preflight(args, stage_dir)
    if args.out.exists():
        raise RuntimeError('output dir exists: %s' % args.out)
    args.out.mkdir(parents=True)
    summary = dict(stage=args.stage, profile=STAGES[args.stage], execute=args.execute,
                   bag_duration_s=round(duration, 1), t_start=time.time())
    link = None
    teleop_started = False
    try:
        deploy_files()
        start_chain(args, stage_dir, args.out)
        if args.persistent_tracker:
            load_stage_bag(args.stage)
        if args.execute:
            ours = procs_matching('fr3-ros2', 'zed_port:=%d' % ZED_PORT)
            if ours and not args.keep_teleop:
                log('stopping our teleop from a previous take for a clean state')
                stop_teleop()
                ours = []
            if ours:
                log('reusing our test teleop (--keep-teleop)')
            else:
                log('starting test teleop (UDP %d)' % ZED_PORT)
                dexec_detached('fr3-ros2', JAZZY + TELEOP_CMD, '/tmp/bagtest/teleop.log')
                teleop_started = True
            link = RobotLink()
            st = link.wait_phase(lambda s: bool(s) and isinstance(s.get('tcp'), list)
                                 and all(v is not None for v in s['tcp']), 45, 'teleop status with TCP')
            log('teleop status: engaged=%s phase=%s' % (st.get('engaged'), st.get('phase')))
            dist = math.dist(st['tcp'], HOME_TCP)
            if dist > HOME_TOL_M:
                log('TCP %.3fm from home; homing first' % dist)
                link.send('home')
                link.wait_phase(lambda s: not s.get('home_running') and s.get('home_result'), 150, 'home')
                log('home: %s' % link.status.get('home_result'))
            p = dexec('fr3-ros2', JAZZY + 'ros2 param set /fr3_joy_teleop zed_review_exercise %s'
                      % STAGES[args.stage], timeout=20)
            if 'successful' not in p.stdout.lower():
                raise RuntimeError('profile not accepted: ' + (p.stdout + p.stderr)[-300:])
            log('exercise profile: %s' % STAGES[args.stage])
        http_post('http://127.0.0.1:%d/control' % CAM_PAGE, dict(action='play'), 'X-Bag-Camera')
        log('bag playing (warm-up for tracker lock); waiting for new-inference samples')
        scored_from = None
        for attempt in range(2):
            if attempt > 0:  # previous pass ended without lock; replay for warm-up
                http_post('http://127.0.0.1:%d/control' % CAM_PAGE, dict(action='restart'), 'X-Bag-Camera')
                time.sleep(1.0)
                http_post('http://127.0.0.1:%d/control' % CAM_PAGE, dict(action='play'), 'X-Bag-Camera')
                log('warm-up pass %d' % (attempt + 1))
            deadline = time.time() + duration + 30
            while time.time() < deadline:
                if grep_count('fr3-curobo', '/tmp/bagtest/samples.jsonl', 'fr3_bagtest/teleop_sample') > 0:
                    break
                time.sleep(1.0)
            else:
                continue  # bag ended with no lock; one more warm-up pass, then fail
            break
        else:
            raise RuntimeError('tracker never locked in 2 warm-up passes (camera broken?)')
        # Scored pass: restart the bag with the tracker already locked, as in
        # a continuous operator session. Only wall_t >= scored_from counts.
        http_post('http://127.0.0.1:%d/control' % CAM_PAGE, dict(action='restart'), 'X-Bag-Camera')
        time.sleep(1.0)
        http_post('http://127.0.0.1:%d/control' % CAM_PAGE, dict(action='play'), 'X-Bag-Camera')
        scored_from = time.time()
        summary['scored_from_wall'] = scored_from
        summary['warmup_passes'] = attempt + 1
        log('samples flowing; scored pass started')
        if args.execute:
            link.send('zed_start')
            st = link.wait_phase(lambda s: s.get('engaged'), 20, 'engage')
            log('engaged: %s' % st.get('note'))
            was_engaged = True
        else:
            was_engaged = False
        deadline = time.time() + duration + 120
        while time.time() < deadline:
            st = http_json('http://127.0.0.1:%d/state' % CAM_PAGE, timeout=5)
            if args.execute:
                cur = link.poll(timeout=1.0)
                if was_engaged and cur and not cur.get('engaged'):
                    raise RuntimeError('teleop disengaged mid-take: %s' % cur.get('note'))
            if st.get('ended'):
                break
            time.sleep(1.0)
        else:
            raise RuntimeError('bag did not finish in time')
        log('bag ended: %s' % json.dumps(st.get('published')))
        summary['replay_published'] = st.get('published')
        if args.execute:
            link.send('stop')
            link.wait_phase(lambda s: not s.get('engaged'), 15, 'stop')
            summary['t_stop_confirmed'] = time.time()
            # Dwell disengaged with recorders running: proves the controller
            # holds instead of drifting toward a stale target (holds=0 history).
            log('post-stop dwell %.0fs (holding, no motion)' % POST_STOP_DWELL_S)
            time.sleep(POST_STOP_DWELL_S)
            summary['t_home_start'] = time.time()
            link.send('home')
            link.wait_phase(lambda s: not s.get('home_running') and s.get('home_result'), 150, 'home')
            summary['home_result'] = link.status.get('home_result')
            tcp = link.status.get('tcp')
            summary['home_tcp'] = tcp
            log('home: %s tcp=%s' % (summary['home_result'], tcp))
    finally:
        try:
            if link is not None:
                try:
                    link.send('stop')
                except OSError:
                    pass
                link.close()
        finally:
            collect_files(args.out)
            try:
                stop_chain(args.persistent_tracker)
            except RuntimeError as e:
                summary['cleanup_error'] = repr(e)
                log('CLEANUP: %s' % e)
            if teleop_started and not args.keep_teleop:
                try:
                    stop_teleop()
                except RuntimeError as e:
                    summary['teleop_cleanup_error'] = repr(e)
                    log('CLEANUP: %s' % e)
    # ---- compare ----
    scored_from = summary.get('scored_from_wall', 0.0)
    new, recorded = parse_samples(args.out / 'samples.jsonl', scored_from)
    take_span = max([s['wall_t'] for s in new], default=0) - min([s['wall_t'] for s in new], default=0)
    flow = sample_flow_metrics(sorted(s['wall_t'] for s in new), take_span or duration)
    orient_frac = (sum(1 for s in new if s['orientation_valid']) / len(new)) if new else 0.0
    recog = recognition_verdict(args.stage, new, recorded, flow)
    summary['samples_new'] = len(new)
    summary['samples_recorded'] = len(recorded)
    summary['orientation_valid_fraction'] = round(orient_frac, 3)
    summary['reinfer_drift_cm'] = recog['drift_cm']
    summary['reinfer_orient_drift_deg'] = recog['orient_drift_deg']
    summary['gestures_seen'] = recog['gestures']
    summary['wrist_max_jump_m'] = recog['max_jump_m']
    summary['wrist_big_jumps'] = recog['n_big_jumps']
    log('samples: new=%d recorded=%d orient_valid=%.2f drift=%scm/%sdeg pairs=%d/%d jumps=%d(max %.2fm) gestures=%s' % (
        len(new), len(recorded), orient_frac,
        'nan' if recog['drift_cm'] is None else '%.1f' % recog['drift_cm'],
        'nan' if recog['orient_drift_deg'] is None else '%.1f' % recog['orient_drift_deg'],
        recog['pairs'][0], recog['pairs'][1],
        recog['n_big_jumps'], recog['max_jump_m'], recog['gestures']))
    if args.execute:
        post = parse_post_stop(args.out / 'robot.jsonl', scored_from,
                               summary.get('t_stop_confirmed'), summary.get('t_home_start'))
        summary['post_stop'] = post
        log(post_stop_line(post))
        # Score motion on the engaged exercise window only: post-stop dwell
        # and home return must not inflate spans (04: identical either way).
        scored_to = post.get('t_disengage')
        if scored_to:
            summary['scored_to_wall'] = scored_to
        series, grips, engaged = parse_robot(args.out / 'robot.jsonl', scored_from, scored_to)
        summary['tcp_points'] = len(series)
        summary['ever_engaged'] = engaged
        if len(series) < 5:
            raise RuntimeError('too few TCP points (%d); robot recording broken?' % len(series))
        result = compare_take(args.stage, series, flow, grips or None)
        summary['check'] = result
        log(check_line(result))
    else:
        summary['recognition_metric'] = recog['metric']
        summary['recognition_ok'] = recog['ok']
        summary['recognition_detail'] = {k: recog[k] for k in (
            'flow_ok', 'metric_ok', 'jumps_ok', 'pairs', 'max_jump_m', 'n_big_jumps')}
        log('RECOGNITION %s [%s] flow=%s metric=%s jumps=%s (dry run, robot untouched)' % (
            'PASS' if recog['ok'] else 'FAIL', recog['metric'],
            recog['flow_ok'], recog['metric_ok'], recog['jumps_ok']))
    summary['t_end'] = time.time()
    (args.out / 'summary.json').write_text(json.dumps(summary, indent=1))
    if args.execute and not summary['check']['ok']:
        sys.exit('TAKE FAILED: %s' % args.stage)
    if not args.execute and not summary['recognition_ok']:
        sys.exit('RECOGNITION FAILED: %s' % args.stage)
    log('done: %s' % args.out)


if __name__ == '__main__':
    main()
