#!/usr/bin/env python3
"""Live object-target selector for the FR3 dashboard (VM, :8898; SELECTOR_BIND, default 127.0.0.1).

GET  /object-target  -> status JSON (selected, phase, mask/map/grasp checks, can_select)
POST /object-target  {"target": "drill|tennis_ball|sprite", "click": [x, y]}
                     x, y = click on the ZED1 image, normalised 0..1.
POST /object-target  {"prompt": "red mug"}      (VLM text path, 2026-11-23)
                     The vm-vlm node (Gemini grounding on /vm/cable/rgb = ZED1 left, 640x360) returns bbox_1000; its
                     centre is used as the click and the box as the SAM2 seed, then the same fail-closed switch runs with a
                     generic 'custom' target spec. Y stays locked for text targets (no validated grasp for them).
                     NOTE: each prompt sends one ZED1 frame to the Gemini API.

A switch runs as one fail-closed sequence and only while the robot is
disengaged, idle and empty-handed (fresh /teleop/engaged + /teleop/grasp_status):
  1. Y lock: park grasp_binding.json, stop the drill regrasp loop.
  2. Map gate: publish the new id on /wrist_map/target_selection. The depth
     filter still expects the old id, so map integration PAUSES (mismatch).
  3. Reseed SAM2 on the click; require TRACKING and a fresh /vm/view_mask
     whose size fits the object and that contains the click.
  4. Restart the depth filter with mask_object_id=<target> (carving resumes).
  5. Restart /nvblox_curobo with its identical command (clears stale target
     voxels) and wait for the map to rebuild; require the selected object
     carved out and the map repopulated.
  6. Grasp: only targets in Y_VALIDATED get the binding/loop back. Others stay
     Y-locked with the reason shown on the dashboard.
Any failure leaves Y locked; after step 2 the map stays paused rather than
carving an unverified mask. Nothing here commands the arm.
"""
import base64
import json
import math
import os
import re
import shlex
import subprocess
import threading
import time
import traceback
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import numpy as np

BASE = Path('/home/nexus/spot-teleop/fr3_real_repro')
STATE = BASE / 'regrasp/state'
BINDING = STATE / 'grasp_binding.json'
SEL_FILE = STATE / 'object_selection.json'
LOG_FILE = STATE / 'object_selector.log'
LAB = 'murilo@100.88.134.127'
KEY = os.environ.get('LAB_KEY', '/ssh/id_lab')
TOWER_MESH = 'http://100.88.134.127:8788/mesh.bin'
LIVE_CLOUD = 'http://127.0.0.1:8791/livecloud.json'
REGRASP_CONTAINER = 'vm-regrasp-live'
# 0 = never start the drill regrasp loop (Y uses the quick ROS grasp since 11-23, not the broker).
START_REGRASP = os.environ.get('SELECTOR_START_REGRASP', '1') != '0'

TARGETS = {
    # mask fraction of the 640x360 ZED1 image; cloud largest-extent bounds (m); min points
    'drill': dict(label='Drill', mask=(0.002, 0.20), extent=(0.06, 0.45), min_pts=300, box=45),
    'tennis_ball': dict(label='Tennis ball', mask=(0.0005, 0.05), extent=(0.04, 0.15), min_pts=100, box=35),
    'sprite': dict(label='Sprite bottle', mask=(0.001, 0.08), extent=(0.05, 0.60), min_pts=50, box=40),
}
# Free-text (VLM) target: wide gates, label = the prompt. Never in Y_VALIDATED.
CUSTOM_ID = 'custom'
PROMPT_RE = re.compile(r"^[\w][\w \-.,']{0,59}$")
MAX_VLM_BOX = 0.60          # a box wider/taller than this fraction of the image is not an object, refuse it


def custom_spec(prompt):
    return dict(label=prompt[:40], mask=(0.0003, 0.15), extent=(0.02, 0.80), min_pts=50, box=40)


# Only these may get Y. Ball/Sprite are added only after their grasp path and
# swept clearance are validated (handoff 2026-11-21); never flip without evidence.
# Sprite (2026-11-21): top-down/tilted CAP grasp planned live by the broker
# (object_grasps.plan_sprite); the opaque cap leaves ~25 mm/side gripper slack.
Y_VALIDATED = {'drill', 'sprite'}   # sprite: supervised side-grasp trials (user 'try it now' 11-21)
Y_LOCK_REASON = {
    'tennis_ball': 'Ball grasp not validated yet: camera error (5-16 mm) exceeds the 6.5 mm/side gripper margin',
}
Y_NOTE = {
    'drill': 'drill regrasp loop running; Y re-checks the binding against the live cloud',
    'sprite': 'SUPERVISED TRIAL: Y plans a pitched side grasp on the label band; camera error ~10 mm vs 7.5 mm/side slack',
}
FILTER_CMD = ('source /opt/ros/humble/setup.bash; source /workspaces/isaac_ros-dev/install/setup.bash; '
              'export ROS_DOMAIN_ID=8; python3 /tmp/wrist_map_filter/drill_depth_filter.py --ros-args '
              '-p mask_topic:=/vm/view_mask -p target_state_topic:=/wrist_map/target_state '
              '-p mask_object_id:={target} -p depth_scale:=0.9755371809 -p dilation_px:=0 '
              '-p state_timeout_s:=3.0 >> /tmp/wrist_map_filter/filter_scaled.log 2>&1')
NVBLOX_CMD = ('source /opt/ros/humble/setup.bash; source /workspaces/isaac_ros-dev/install/setup.bash; '
              'export ROS_DOMAIN_ID=8; ros2 run nvblox_ros nvblox_node --ros-args '
              '--params-file /workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_examples/nvblox_examples_bringup/config/nvblox/nvblox_base.yaml '
              '--params-file /workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_examples/nvblox_examples_bringup/config/nvblox/specializations/nvblox_zed.yaml '
              '-p num_cameras:=1 -p global_frame:=fr3_link0 -p pose_frame:=fr3_link0 -p map_clearing_frame_id:=fr3_link0 '
              '-p map_clearing_radius_m:=-1.0 -p use_lidar:=false -p use_depth:=true -p use_color:=true -p input_qos:=SENSOR_DATA '
              '-p use_tf_transforms:=true -p voxel_size:=0.02 -p mapping_type:=static_tsdf -p use_segmentation:=false '
              '-p integrate_depth_rate_hz:=5.0 -p integrate_color_rate_hz:=5.0 -p update_mesh_rate_hz:=2.0 -p update_esdf_rate_hz:=2.0 '
              '-p esdf_mode:=3d -p static_mapper.esdf_slice_min_height:=0.02 -p static_mapper.esdf_slice_max_height:=0.8 '
              '-p static_mapper.esdf_slice_height:=0.15 -p static_mapper.projective_integrator_max_integration_distance_m:=2.5 '
              '-p static_mapper.projective_integrator_truncation_distance_vox:=4.0 -p max_back_projection_distance:=7.0 '
              '-r __node:=nvblox_curobo -r camera_0/depth/image:=/wrist_map/drill_free_depth '
              '-r camera_0/depth/camera_info:=/zed1/zed_node/depth/camera_info '
              '-r camera_0/color/image:=/zed1/zed_node/rgb/image_rect_color '
              '-r camera_0/color/camera_info:=/zed1/zed_node/rgb/camera_info '
              '>> /tmp/wrist_map_filter/nvblox_scaled_rebuild.log 2>&1')
MESH_SERVER_CMD = ('source /opt/ros/humble/setup.bash; source /workspaces/isaac_ros-dev/install/setup.bash; '
                   'export ROS_DOMAIN_ID=8 RMW_IMPLEMENTATION=rmw_fastrtps_cpp; cd /tmp/nvblox_real; '
                   'python3 -u mesh_server.py --node nvblox_curobo --port 8788,8795 --bind 127.0.0.1,100.88.134.127 '
                   '>> /tmp/nvblox_real/mesh_server_selector.log 2>&1')
ESDF_SCRIPT = (Path(__file__).parent / 'esdf_points.py').read_text()
ESDF_RUN = ('source /opt/ros/humble/setup.bash; source /workspaces/isaac_ros-dev/install/setup.bash; '
            'export ROS_DOMAIN_ID=8; python3 /tmp/wrist_map_filter/esdf_points.py 2>/dev/null')
TELEOP_ECHO = ('source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=77; '
               'timeout 6 ros2 topic echo --once --full-length /teleop/grasp_status std_msgs/msg/String; '
               'timeout 6 ros2 topic echo --once /teleop/engaged std_msgs/msg/Bool')
MIN_SWITCH_INTERVAL_S = 20.0


def log(msg):
    line = f'{time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())} {msg}'
    print(line, flush=True)
    try:
        with LOG_FILE.open('a') as f:
            f.write(line + '\n')
    except OSError:
        pass


class Fail(RuntimeError):
    pass


# ---------------- pure helpers (unit-tested) ----------------
def click_to_bbox_1000(click, half):
    x, y = (float(v) for v in click)
    if not (math.isfinite(x) and math.isfinite(y) and 0 <= x <= 1 and 0 <= y <= 1):
        raise ValueError('click must be two numbers in 0..1')
    cx, cy = round(x * 1000), round(y * 1000)
    return [max(0, cx - half), max(0, cy - half), min(1000, cx + half), min(1000, cy + half)]


def check_prompt(prompt):
    """Plain object description only: it goes to a ROS parameter and to an external API, keep it boring."""
    if not isinstance(prompt, str):
        raise ValueError('prompt must be text')
    if any(c in prompt for c in '\r\n\t\x00'):
        raise ValueError('prompt must be a single line')
    prompt = ' '.join(prompt.split())
    if not PROMPT_RE.match(prompt):
        raise ValueError('prompt: 1-60 characters, letters/digits/space and - . , \' only')
    return prompt


def parse_vlm_result(text):
    """bbox/label from the vm-vlm Trigger response message (JSON of the best box)."""
    try:
        d = json.loads(text)
    except (TypeError, ValueError) as e:
        raise ValueError(f'VLM answer is not JSON: {str(text)[:80]!r}') from e
    box = d.get('bbox_1000') if isinstance(d, dict) else None
    if not (isinstance(box, (list, tuple)) and len(box) == 4):
        raise ValueError('VLM answer has no bbox_1000')
    x1, y1, x2, y2 = (float(v) for v in box)
    if not all(math.isfinite(v) and 0 <= v <= 1000 for v in (x1, y1, x2, y2)) or x2 <= x1 or y2 <= y1:
        raise ValueError(f'VLM box invalid: {box}')
    if (x2 - x1) / 1000 > MAX_VLM_BOX or (y2 - y1) / 1000 > MAX_VLM_BOX:
        raise ValueError(f'VLM box too large for an object: {[round(v) for v in (x1, y1, x2, y2)]}')
    return dict(bbox=[round(x1), round(y1), round(x2), round(y2)],
                click=[(x1 + x2) / 2000.0, (y1 + y2) / 2000.0],
                confidence=d.get('confidence'), latency_ms=d.get('vlm_latency_ms'))


def pick_coordinator_seed(seed_cmds, t_set, own_label):
    """First /perception/seed_command seen after t_set that did NOT come from this selector.
    vm-coordinator (coordinator_node) reacts to every /vlm/prompt_change_id by calling the VLM itself and publishing
    the seed; a second seed from here would reset the tracker (tracking_active=False) right after it initialised."""
    for t, d in seed_cmds:
        if t >= t_set and isinstance(d, dict) and d.get('label') != own_label and 'bbox_1000' in d:
            return d
    return None


def publish_seed(publish, message):
    """Exactly one seed per attempt; a duplicate can crash active SAM2 tracking."""
    publish(message)


def parse_teleop(text, now):
    """(ok, reason, flags) from the two `ros2 topic echo --once` outputs."""
    blocks = [b for b in text.split('---') if 'data:' in b]
    if len(blocks) < 2:
        return False, 'teleop state unavailable', None
    raw = blocks[0].split('data:', 1)[1].strip()
    if raw[:1] in "'\"":
        raw = raw[1:-1]
    try:
        gs = json.loads(raw.replace("\\'", "'"))
    except ValueError:
        return False, 'grasp status unreadable', None
    eng = blocks[1].split('data:', 1)[1].strip().lower()
    if eng not in ('true', 'false'):
        return False, 'engaged state unreadable', None
    flags = dict(engaged=eng == 'true', held=gs.get('held'), busy=gs.get('busy'), phase=gs.get('phase'), stamp=now)
    if not isinstance(flags['held'], bool) or not isinstance(flags['busy'], bool):
        return False, 'grasp status malformed', flags
    if flags['engaged']:
        return False, 'Disengage (release Start) before changing the target', flags
    if flags['held'] or flags['busy']:
        return False, 'Finish/release the current grasp before changing the target', flags
    return True, 'OK', flags


def mask_check(mask, click, spec, max_age, age):
    if mask is None:
        return False, 'no target mask received'
    if not 0 <= age <= max_age:
        return False, f'target mask stale ({age:.1f}s)'
    m = mask > 0
    frac = float(m.mean())
    lo, hi = spec['mask']
    if not lo <= frac <= hi:
        return False, f'mask size {frac:.4f} outside {lo}..{hi} for this object'
    h, w = m.shape
    u, v = int(click[0] * (w - 1)), int(click[1] * (h - 1))
    r = 6
    if not m[max(0, v - r):v + r + 1, max(0, u - r):u + r + 1].any():
        return False, 'mask does not cover the clicked point (tracker locked onto something else)'
    return True, f'mask OK ({frac * 100:.2f}% of image)'


def cloud_check(cloud, spec):
    P = np.asarray(cloud, float)
    if P.ndim != 2 or P.shape[1] != 3 or not np.isfinite(P).all():
        return False, 'invalid object cloud'
    if len(P) < spec['min_pts']:
        return False, f'object cloud too small ({len(P)} pts)'
    ext = np.percentile(P, 97, 0) - np.percentile(P, 3, 0)
    big = float(ext.max())
    lo, hi = spec['extent']
    if not lo <= big <= hi:
        return False, f'object cloud extent {big:.2f} m outside {lo}..{hi} m'
    return True, f'cloud {len(P)} pts, extent {big:.2f} m'


def nearest_dist(P, V, chunk=256):
    """Exact nearest-vertex distance without scipy (P ~1e3, V ~1e4)."""
    V = V.astype(np.float32)
    out = np.empty(len(P), np.float32)
    for i in range(0, len(P), chunk):
        D = ((P[i:i + chunk, None, :].astype(np.float32) - V[None]) ** 2).sum(-1)
        out[i:i + chunk] = np.sqrt(D.min(1))
    return out


def map_carved(mesh_V, cloud, z_min=0.06, near=0.015, max_near_frac=0.30, min_pts=20):
    """Selected object is carved if few of its points (above the table band)
    have a map vertex within `near` (one voxel is 2 cm)."""
    V = np.asarray(mesh_V, float)
    if len(V) < 1000:
        return False, f'map too sparse ({len(V)} vertices)', None
    P = np.asarray(cloud, float)
    P = P[P[:, 2] > z_min]
    if len(P) < min_pts:
        return False, f'too few object points above the table ({len(P)})', None
    d = nearest_dist(P, V)
    frac = float((d < near).mean())
    return frac <= max_near_frac, f'{frac * 100:.0f}% of object points still in map (limit {max_near_frac * 100:.0f}%)', frac


# ---------------- side-effect helpers ----------------
def run(cmd, timeout=30, check=True, input=None):
    p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, input=input)
    if check and p.returncode:
        raise Fail(f'{" ".join(cmd[:3])}... rc={p.returncode}: {(p.stderr or p.stdout).strip()[-300:]}')
    return p


def tower(script, timeout=40, check=True):
    b64 = base64.b64encode(script.encode()).decode()
    return run(['ssh', '-n', '-i', KEY, '-o', 'IdentitiesOnly=yes', '-o', 'BatchMode=yes',
                '-o', 'ConnectTimeout=8', LAB, f'echo {b64} | base64 -d | bash'], timeout=timeout, check=check)


def esdf_at(points):
    """Authoritative nvblox ESDF (what cuRobo reads) at points; read-only service call."""
    b64s = base64.b64encode(ESDF_SCRIPT.encode()).decode()
    pts = base64.b64encode(json.dumps(np.round(np.asarray(points, float), 4).tolist()).encode()).decode()
    p = tower(f"echo {b64s} | base64 -d > /tmp/esdf_points.py && docker cp /tmp/esdf_points.py isaac-build:/tmp/wrist_map_filter/esdf_points.py && "
              f"echo {pts} | base64 -d | docker exec -i isaac-build bash -lc {shlex.quote(ESDF_RUN)}", timeout=60)
    d = json.loads(p.stdout.strip().splitlines()[-1])
    if not d.get('success'):
        raise Fail('ESDF query failed')
    return np.array([np.nan if v is None else v for v in d['esdf']], float)


def esdf_carved(esdf, min_free=0.03, need=0.8):
    """Target voxels must be unobserved (-1000, carved) or clearly free."""
    e = esdf[np.isfinite(esdf)]
    if len(e) < 10:
        return False, f'too few ESDF samples ({len(e)})'
    frac = float(((e <= -100) | (e >= min_free)).mean())
    return frac >= need, f'ESDF: {frac * 100:.0f}% of target points carved (need {need * 100:.0f}%)'


def load_mesh(url=TOWER_MESH):
    b = urllib.request.urlopen(url, timeout=10).read()
    h = np.frombuffer(b, np.uint32, 3)
    return np.frombuffer(b, np.float32, int(h[1]) * 3, 12).reshape(-1, 3)


def live_cloud(wait_s=20.0):
    """Fresh fused object cloud; the viewer needs a few s to follow a new mask."""
    t0, err = time.time(), None
    while True:
        try:
            return _live_cloud_once()
        except Exception as e:  # noqa: BLE001
            err = e
        if time.time() - t0 > wait_s:
            raise Fail(str(err))
        time.sleep(1.0)


def _live_cloud_once():
    d = json.loads(urllib.request.urlopen(LIVE_CLOUD, timeout=5).read())
    age = float(d.get('age_s', 999))
    if not 0 <= age <= 2.5:
        raise Fail(f'live object cloud stale ({age:.1f}s)')
    z1 = np.asarray(d.get('z1') or [], float).reshape(-1, 3)
    z2 = np.asarray(d.get('z2') or [], float).reshape(-1, 3)
    return np.vstack([z1, z2]), z1


class Selector:
    def __init__(self, node):
        self.node = node
        self.lock = threading.Lock()
        self.busy = False
        self.selected = None
        self.generation = 0
        self.phase = 'starting'
        self.reason = 'Starting'
        self.checks = {k: dict(ok=False, detail='not checked') for k in ('mask', 'map', 'grasp')}
        self.y_enabled = False
        self.teleop = (False, 'teleop state not read yet', None)
        self.last_switch = 0.0
        self.steps = []
        self.prompt = None          # last VLM text target (label shown on the card)
        self.vlm = None             # last grounding result (bbox, latency)

    # ---- status ----
    def _persist(self):
        doc = dict(target=self.selected, generation=self.generation, phase=self.phase, reason=self.reason,
                   y_enabled=self.y_enabled, checks=self.checks, stamp=time.time())
        tmp = SEL_FILE.with_suffix('.tmp')
        tmp.write_text(json.dumps(doc, indent=1))
        tmp.replace(SEL_FILE)

    def _set(self, phase, reason, **checks):
        self.phase, self.reason = phase, reason
        for k, v in checks.items():
            self.checks[k] = v
        self.steps = (self.steps + [f'{time.strftime("%H:%M:%S")} {reason}'])[-12:]
        log(f'[{self.selected} g{self.generation}] {phase}: {reason}')
        self._persist()

    def api(self):
        ok, why, flags = self.teleop
        # Teleop state is checked when the user clicks (fresh read in request()).
        can = bool(not self.busy and time.time() - self.last_switch > MIN_SWITCH_INTERVAL_S)
        fresh = True
        live_mask = self.node.mask_status()
        # Y is only really armed while the tracker and map gate are on the selected object
        # (the broker enforces the same); report it that way so a LOST target never reads ready.
        live_ok = (live_mask.get('tracking') == 'TRACKING' and self.selected is not None
                   and live_mask.get('map_state') == f'ACTIVE:{self.selected}')
        y_now = bool(self.y_enabled and live_ok)
        return dict(selected=self.selected, generation=self.generation, phase=self.phase,
                    status=self.phase, reason=self.reason, busy=self.busy, can_select=can,
                    select_block=None if can else ('switch in progress' if self.busy else 'wait a few seconds between switches'),
                    last_teleop_check=None if flags is None else dict(ok=ok, reason=why, age_s=round(time.time() - flags.get('stamp', 0), 1)),
                    y_enabled=y_now, y_armed=self.y_enabled,
                    y_block=None if y_now or not self.y_enabled else 'tracker/map lost the selected object: reselect it',
                    mask=self.checks['mask'], map=self.checks['map'],
                    grasp=self.checks['grasp'], tracking=live_mask,
                    targets={k: dict(label=v['label'], y_validated=k in Y_VALIDATED,
                                     y_lock_reason=Y_LOCK_REASON.get(k)) for k, v in TARGETS.items()},
                    text_target=dict(enabled=True, prompt=self.prompt if self.selected == CUSTOM_ID else None,
                                     vlm=self.vlm if self.selected == CUSTOM_ID else None),
                    steps=self.steps)

    def poll_teleop(self):
        """No periodic ROS polling: spawning `ros2` CLI participants in fr3-ros2 stalled the
        teleop joint callback (0.3 s watchdog) during Y runs on 11-21. Teleop state is read
        on demand only (POST and before the map rebuild)."""
        return

    def adopt_current(self):
        """Adopt the live state at startup (no side effects)."""
        try:
            st = tower('docker exec isaac-build cat /tmp/wrist_map_filter/status.json', check=False).stdout
            state = json.loads(st)['recent']['state']
        except Exception:  # noqa: BLE001
            state = ''
        prev = {}
        if SEL_FILE.exists():
            try:
                prev = json.loads(SEL_FILE.read_text())
                self.generation = int(prev.get('generation', 0))
            except ValueError:
                prev = {}
        tgt = state.partition(':')[2] if ':' in state else None
        self.selected = tgt if tgt in TARGETS else None
        binding_ok = BINDING.exists()
        # Trust only a switch this service completed (or the original drill
        # setup, which predates the selector). Anything else must be reselected.
        completed = prev.get('target') == self.selected and prev.get('phase') == 'ready'
        legacy_drill = not prev and self.selected == 'drill'
        if self.selected and (completed or legacy_drill):
            self.y_enabled = self.selected in Y_VALIDATED and (binding_ok or self.selected != 'drill')
            self._set('ready', f'adopted live map state {state}' + ('' if binding_ok else ' (Y binding parked)'),
                      mask=dict(ok=True, detail=state),
                      map=dict(ok=True, detail=prev.get('checks', {}).get('map', {}).get('detail', 'existing map')),
                      grasp=dict(ok=self.y_enabled, detail=Y_NOTE.get(self.selected) if self.y_enabled else
                                 Y_LOCK_REASON.get(self.selected, 'no binding')))
        else:
            self.y_enabled = False
            self._set('error', f'live state {state or "unknown"} was not completed by the selector; '
                      'select the object again', mask=dict(ok=False, detail=state or 'unknown'),
                      map=dict(ok=False, detail='unverified'), grasp=dict(ok=False, detail='Y locked'))

    # ---- switch ----
    def request(self, target, click):
        if target not in TARGETS:
            raise ValueError('unknown target')
        bbox = click_to_bbox_1000(click, TARGETS[target]['box'])
        with self.lock:
            ok, why, _ = self.teleop
            if self.busy:
                raise Fail('a switch is already running')
            if time.time() - self.last_switch < MIN_SWITCH_INTERVAL_S:
                raise Fail('wait a few seconds between switches (tracker reseed limit)')
            # Fresh read, not the cached poll.
            p = tower(f'docker exec fr3-ros2 bash -lc {shlex.quote(TELEOP_ECHO)}', timeout=20, check=False)
            ok, why, _ = self.teleop = parse_teleop(p.stdout, time.time())
            if not ok:
                raise Fail(why)
            self.busy = True
            self.last_switch = time.time()
            self.generation += 1
            self.selected = target
        threading.Thread(target=self._switch, args=(target, click, bbox), daemon=True).start()
        return self.generation

    def request_prompt(self, prompt):
        prompt = check_prompt(prompt)
        with self.lock:
            if self.busy:
                raise Fail('a switch is already running')
            if time.time() - self.last_switch < MIN_SWITCH_INTERVAL_S:
                raise Fail('wait a few seconds between switches (tracker reseed limit)')
            p = tower(f'docker exec fr3-ros2 bash -lc {shlex.quote(TELEOP_ECHO)}', timeout=20, check=False)
            ok, why, _ = self.teleop = parse_teleop(p.stdout, time.time())
            if not ok:
                raise Fail(why)
            self.busy = True
            self.last_switch = time.time()
            self.generation += 1
            self.selected = CUSTOM_ID
            self.prompt = prompt
            self.vlm = None
        threading.Thread(target=self._switch_text, args=(prompt,), daemon=True).start()
        return self.generation

    def _switch_text(self, prompt):
        """Text target: the normal fail-closed switch; the VLM grounding + seeding happens inside it, after the
        map is paused and vm-tfproj reset (see _text_seed)."""
        self._switch(CUSTOM_ID, None, None, spec=custom_spec(prompt), text_prompt=prompt)

    def _text_seed(self, prompt, target):
        """Ground the prompt and get the tracker TRACKING with exactly ONE seed. Returns (click, bbox).
        Prompt changed -> vm-vlm announces it and vm-coordinator does VLM + seed itself: we only wait for its seed
        (a second seed from here reset the tracker: 2026-11-23 g14/g15, overlay appeared then vanished, LOST, error).
        Same prompt -> nobody reacts: trigger the VLM and seed ourselves (the g13 path)."""
        t_set = time.time()
        changed = self.node.vlm_set_prompt(prompt)
        if changed:
            self._set('seeding', f'new prompt {prompt!r}: vm-coordinator asks the VLM and seeds the tracker '
                                 '(one ZED1 frame goes to the Gemini API); waiting for its seed')
            res = self.node.wait_coordinator_seed(t_set, target)
            self.vlm = dict(bbox=res['bbox'], confidence=res['confidence'], latency_ms=res['latency_ms'])
            log(f'coordinator seed for {prompt!r}: bbox_1000 {res["bbox"]} latency {res["latency_ms"]} ms')
            self._set('seeding', f'seed from the coordinator accepted (box {res["bbox"]}); waiting for TRACKING (up to 60 s)')
            self.node.wait_tracking(time.time(), 60.0)
        else:
            self._set('seeding', f'asking the VLM for {prompt!r} (one ZED1 frame goes to the Gemini API)')
            res = self.node.vlm_ground(prompt)
            self.vlm = dict(bbox=res['bbox'], confidence=res['confidence'], latency_ms=res['latency_ms'])
            log(f'VLM grounded {prompt!r}: bbox_1000 {res["bbox"]} latency {res["latency_ms"]} ms')
            self._set('seeding', f'seeding tracker at bbox_1000 {res["bbox"]}')
            self.node.seed(res['bbox'], target, progress=lambda m: self._set('seeding', m))
        return res['click'], res['bbox']

    def _switch(self, target, click, bbox, spec=None, text_prompt=None):
        spec = spec or TARGETS[target]
        try:
            self.y_enabled = False
            self._set('locking', f'Y locked; switching to {spec["label"]}',
                      mask=dict(ok=False, detail='pending'), map=dict(ok=False, detail='pending'),
                      grasp=dict(ok=False, detail='locked during switch'))
            self._park_binding()
            run(['docker', 'stop', '-t', '10', REGRASP_CONTAINER], timeout=30, check=False)

            self._set('map_paused', 'map integration paused (new target announced)')
            self.node.select_map_target(target)

            # tf_projection keeps the FIRST object it ever saw as an immutable
            # reference and republishes it as seed_pixel for 10 s after every
            # seed (11-21: ball seed initialised on the old pixel -> table mask).
            # A restart makes the clicked object its reference.
            self._set('seeding', 'resetting seed reprojection (vm-tfproj)')
            run(['docker', 'restart', '-t', '5', 'vm-tfproj'], timeout=40)
            time.sleep(8)
            if text_prompt is None:
                self._set('seeding', f'seeding tracker at bbox_1000 {bbox}')
                self.node.seed(bbox, target, progress=lambda m: self._set('seeding', m))
            else:
                click, bbox = self._text_seed(text_prompt, target)
            time.sleep(1.5)
            age, mask = self.node.latest_mask()
            ok, detail = mask_check(mask, click, spec, 1.5, age)
            if not ok:
                raise Fail(f'mask check failed: {detail}')
            self.checks['mask'] = dict(ok=True, detail=detail)
            try:
                cloud, _ = live_cloud()
                cok, cdet = cloud_check(cloud, spec)
            except Exception as e:  # noqa: BLE001
                cloud, cok, cdet = None, False, f'live cloud unavailable: {e}'
            if not cok and target != 'sprite':
                raise Fail(f'object cloud check failed: {cdet}')
            self.checks['mask'] = dict(ok=True, detail=f'{detail}; {cdet}')

            # Last chance to stop before the map is touched.
            p = tower(f'docker exec fr3-ros2 bash -lc {shlex.quote(TELEOP_ECHO)}', timeout=20, check=False)
            ok, why, _ = self.teleop = parse_teleop(p.stdout, time.time())
            if not ok:
                raise Fail(f'stopped before map rebuild: {why}')

            self._set('map_rebuild', 'retargeting depth filter and clearing the map')
            before = len(load_mesh())
            tower('docker exec isaac-build pkill -f /tmp/wrist_map_filter/drill_depth_filter.py; sleep 1; '
                  f'docker exec -d isaac-build bash -lc {shlex.quote(FILTER_CMD.format(target=target))}')
            tower("docker exec isaac-build pkill -INT -f '__node:=nvblox_curobo'; "
                  "for i in $(seq 20); do docker exec isaac-build pgrep -f '__node:=nvblox_curobo' >/dev/null || break; sleep 0.5; done; "
                  "docker exec isaac-build pkill -KILL -f '__node:=nvblox_curobo'; sleep 1; "
                  f'docker exec -d isaac-build bash -lc {shlex.quote(NVBLOX_CMD)}', timeout=60)
            # mesh_server keeps incremental blocks until a clear; after an nvblox
            # restart it would keep serving the old geometry (seen 11-21). Restart it.
            tower("docker exec isaac-build pkill -f 'mesh_server.py --node nvblox_curobo'; sleep 1; "
                  f'docker exec -d isaac-build bash -lc {shlex.quote(MESH_SERVER_CMD)}', timeout=30)
            self._wait_map(before, target, spec)

            if target == 'drill':
                self._restore_binding()
                if START_REGRASP:
                    run(['docker', 'start', REGRASP_CONTAINER], timeout=30)
                self.y_enabled = BINDING.exists()
                self.checks['grasp'] = dict(ok=self.y_enabled, detail=(
                    Y_NOTE['drill'] if self.y_enabled else 'no drill binding to restore; regrasp loop will regenerate'))
            elif target in Y_VALIDATED:
                self.y_enabled = True      # broker plans from the live cloud; drill binding stays parked
                self.checks['grasp'] = dict(ok=True, detail=Y_NOTE[target])
            else:
                self.checks['grasp'] = dict(ok=False, detail=Y_LOCK_REASON.get(
                    target, 'text target: no validated grasp for this object, Y stays locked'))
            self._set('ready', f'{spec["label"]} selected' + ('' if self.y_enabled else ' (Y locked)'))
        except Exception as e:  # noqa: BLE001
            self.y_enabled = False
            self.checks['grasp'] = dict(ok=False, detail='Y locked after failed switch')
            self._set('error', f'{e}; Y locked, map paused until a successful selection')
            log(traceback.format_exc())
        finally:
            self.busy = False

    def _wait_map(self, before, target, spec):
        t0, counts = time.time(), []
        while time.time() - t0 < 90:
            time.sleep(3)
            try:
                n = len(load_mesh())
            except Exception:  # noqa: BLE001
                continue
            counts.append(n)
            self._set('map_rebuild', f'map rebuilding: {n} vertices (before {before})')
            if len(counts) >= 3 and n >= 0.6 * before and max(counts[-3:]) - min(counts[-3:]) <= 0.05 * n:
                break
        else:
            raise Fail(f'map did not rebuild in 90 s (vertices {counts[-1:] or 0}, before {before})')
        mesh = load_mesh()
        try:
            _, z1 = live_cloud()
        except Exception as e:  # noqa: BLE001
            raise Fail(f'cannot verify map carve: {e}') from e
        z1 = z1[z1[:, 2] > 0.05]
        if len(z1) < 10:
            raise Fail('too few ZED1 target points to verify the map')
        ok_e, det_e = esdf_carved(esdf_at(z1[::max(1, len(z1) // 300)]))
        ok_m, det_m, _ = map_carved(mesh, z1)
        if not ok_e and target != 'sprite':
            raise Fail(f'map check failed: {det_e}; mesh {det_m}')
        self.checks['map'] = dict(ok=ok_e, detail=f'rebuilt; {det_e}; mesh {det_m}; other objects stay obstacles')

    def _park_binding(self):
        if BINDING.exists():
            BINDING.replace(STATE / f'grasp_binding.json.parked_{time.strftime("%Y%m%d_%H%M%S")}')

    def _restore_binding(self):
        if BINDING.exists():
            return
        parked = sorted(STATE.glob('grasp_binding.json.parked_*'))
        if parked:
            parked[-1].replace(BINDING)


class RosIO:
    def __init__(self):
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
        from sensor_msgs.msg import Image, CameraInfo
        from std_msgs.msg import String
        from geometry_msgs.msg import PointStamped
        rclpy.init()
        self.rclpy, self.String, self.PointStamped = rclpy, String, PointStamped
        self.node = rclpy.create_node('object_selector')
        rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.mask = (None, 0.0)
        self.frame_stamp = None     # (header.stamp, wall time) of the latest tracker frame (from /vm/view_mask)
        self.tracking = (None, 0.0)
        self.map_state = (None, 0.0)
        self.seed3d = None
        self.info = None
        self.node.create_subscription(Image, '/vm/view_mask', self._mask_cb, qos_profile_sensor_data)
        self.node.create_subscription(Image, '/vm/view_mask', self._mask_cb, rel)
        self.node.create_subscription(String, '/tracking_state', lambda m: setattr(self, 'tracking', (m.data, time.time())), rel)
        self.node.create_subscription(String, '/wrist_map/target_state', lambda m: setattr(self, 'map_state', (m.data, time.time())), rel)
        self.node.create_subscription(PointStamped, '/tracking/seed_3d', lambda m: setattr(self, 'seed3d', m), rel)
        self.node.create_subscription(CameraInfo, '/zed1/zed_node/depth/camera_info', lambda m: setattr(self, 'info', m), qos_profile_sensor_data)
        from std_srvs.srv import SetBool, Trigger
        from rcl_interfaces.srv import SetParameters
        from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
        self._Parameter, self._ParameterValue, self._ParameterType = Parameter, ParameterValue, ParameterType
        self._SetBool, self._Trigger, self._SetParameters = SetBool, Trigger, SetParameters
        self.vlm_param_cli = self.node.create_client(SetParameters, '/vlm_relocalize_node/set_parameters')
        self.vlm_prompt_cli = self.node.create_client(SetBool, '/vlm/set_object_prompt')
        self.vlm_trigger_cli = self.node.create_client(Trigger, '/vlm/trigger_relocalize')
        self.prompt_ids = []        # wall times of /vlm/prompt_change_id (published by vm-vlm when the prompt really changes)
        self.seed_cmds = []         # (wall time, dict) of /perception/seed_command, last 12 (ours and the coordinator's)
        self.node.create_subscription(String, '/vlm/prompt_change_id', lambda m: self.prompt_ids.append(time.time()), rel)
        self.node.create_subscription(String, '/perception/seed_command', self._seed_cmd_cb, rel)
        self.sel_pub = self.node.create_publisher(String, '/wrist_map/target_selection', rel)
        self.seed_pub = self.node.create_publisher(String, '/perception/seed_command', 10)
        self.px_pub = self.node.create_publisher(PointStamped, '/tracking/seed_pixel', 10)
        threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True).start()

    def _mask_cb(self, m):
        if m.encoding != 'mono8':
            return
        arr = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width].copy()
        self.mask = (arr, time.time())
        # /vm/view_mask carries the exact header of the rgb frame the tracker is processing.
        self.frame_stamp = (m.header.stamp, time.time())

    def latest_mask(self):
        arr, t = self.mask
        return time.time() - t, arr

    def mask_status(self):
        arr, t = self.mask
        st, ts = self.tracking
        ms, mt = self.map_state
        return dict(tracking=st, tracking_age_s=round(time.time() - ts, 1) if ts else None,
                    mask_fraction=None if arr is None else round(float((arr > 0).mean()), 5),
                    mask_age_s=round(time.time() - t, 1) if t else None,
                    map_state=ms, map_state_age_s=round(time.time() - mt, 1) if mt else None)

    def _seed_cmd_cb(self, m):
        try:
            d = json.loads(m.data)
        except ValueError:
            return
        self.seed_cmds = (self.seed_cmds + [(time.time(), d)])[-12:]

    def _vlm_call(self, client, request, timeout, what):
        if not client.wait_for_service(timeout_sec=3.0):
            raise Fail(f'VLM node not reachable ({what}); is vm-vlm running?')
        fut = client.call_async(request)
        t0 = time.time()
        while not fut.done():
            if time.time() - t0 > timeout:
                raise Fail(f'VLM {what} timed out after {timeout:.0f} s')
            time.sleep(0.05)
        return fut.result()

    def vlm_set_prompt(self, prompt):
        """Set the vm-vlm prompt. True when the prompt really changed: vm-vlm then publishes /vlm/prompt_change_id and
        vm-coordinator grounds it and seeds the tracker BY ITSELF (do not trigger/seed again). False = same prompt as
        before, nobody reacts and the caller must trigger + seed."""
        t_set = time.time()
        P = self._Parameter(name='new_object_prompt', value=self._ParameterValue(
            type=self._ParameterType.PARAMETER_STRING, string_value=prompt))
        r = self._vlm_call(self.vlm_param_cli, self._SetParameters.Request(parameters=[P]), 8, 'set prompt')
        if not r.results or not r.results[0].successful:
            raise Fail('the VLM node rejected the prompt parameter')
        # success=False just means "same prompt as before".
        self._vlm_call(self.vlm_prompt_cli, self._SetBool.Request(data=True), 8, 'apply prompt')
        t0 = time.time()
        while time.time() - t0 < 2.5:
            if any(t >= t_set for t in self.prompt_ids):
                return True
            time.sleep(0.1)
        return False

    def wait_coordinator_seed(self, t_set, own_label, timeout=75.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            d = pick_coordinator_seed(self.seed_cmds, t_set, own_label)
            if d is not None:
                try:
                    return parse_vlm_result(json.dumps(d))
                except ValueError as e:
                    raise Fail(str(e)) from e
            time.sleep(0.3)
        raise Fail(f'the coordinator did not publish a seed within {timeout:.0f} s (VLM found nothing or is down)')

    def wait_tracking(self, t0, timeout=60.0):
        while time.time() - t0 < timeout:
            st, ts = self.tracking
            if st == 'TRACKING' and time.time() - ts < 2 and ts > t0 - 1:
                return
            time.sleep(0.3)
        raise Fail(f'tracker state {self.tracking[0]!r}, want TRACKING')

    def vlm_ground(self, prompt):
        """Trigger the VLM on the latest ZED1 frame (prompt already set). Returns parse_vlm_result()."""
        t = self._vlm_call(self.vlm_trigger_cli, self._Trigger.Request(), 50, 'grounding')
        if not t.success:
            raise Fail(f'the VLM did not find {prompt!r} in the frame ({str(t.message)[:100]})')
        try:
            return parse_vlm_result(t.message)
        except ValueError as e:
            raise Fail(str(e)) from e

    def select_map_target(self, target):
        for _ in range(5):
            self.sel_pub.publish(self.String(data=target))
            time.sleep(0.3)
        t0 = time.time()
        while time.time() - t0 < 6:
            s, ts = self.map_state
            if s and s.endswith(':' + target) and time.time() - ts < 1.0:
                return
            time.sleep(0.25)
        raise Fail(f'map gate did not accept {target!r} (state {self.map_state[0]!r})')

    def seed(self, bbox, label, progress=None):
        """Same sequence as grasp_loop.s1_seed (proven 11-20/21), without the px bridge.
        progress(msg) reports the real sub-step: the card used to say just 'seeding' for the whole sequence
        (up to ~2 min) even while the tracker already showed TRACKING."""
        note = progress or (lambda m: None)
        seed = json.dumps({'bbox_1000': bbox, 'label': label, 'confidence': 1.0})
        self.seed3d = None
        s3 = None
        for attempt in range(3):
            publish_seed(self.seed_pub.publish, self.String(data=seed))
            log(f'seed sent (try {attempt + 1}) {seed}')
            note(f'seed sent, waiting for the tracker to answer (try {attempt + 1}/3, up to 25 s)')
            t0 = time.time()
            while self.seed3d is None and time.time() - t0 < 25:
                time.sleep(0.2)
            s3 = self.seed3d
            if s3 is not None:
                break
            time.sleep(10)
        if s3 is None:
            raise Fail('tracker did not answer the seed (no seed_3d)')
        info = self.info
        if info is None:
            raise Fail('no ZED1 camera_info')
        K = np.array(info.k).reshape(3, 3)
        u = K[0, 0] * (s3.point.x / s3.point.z) + K[0, 2]
        v = K[1, 1] * (s3.point.y / s3.point.z) + K[1, 2]
        t0 = time.time()
        last_note = 0.0
        while time.time() - t0 < 15:
            if time.time() - last_note >= 4.0:
                last_note = time.time()
                note(f'seed accepted; pixel seed for ZED1 {int(time.time() - t0)}/15 s (fixed window), '
                     f'tracker says {self.tracking[0]}')
            m = self.PointStamped()
            m.header.frame_id = info.header.frame_id
            # The tracker only consumes a pending seed pixel on a frame whose stamp is >= the pixel stamp - 0.5 s
            # (sam2_tracker_node stamp-gate). tf_projection stamps with the frame time; a wall-clock stamp made
            # the gate unreachable once /vm/cable/rgb frames were ~3 s old (lab at load 40, 2026-11-23: seeds from
            # this selector never initialised ZED1 -> LOST for 60 s). Stamp with the latest tracker frame instead.
            fs = self.frame_stamp
            if fs is not None and time.time() - fs[1] < 5.0:
                m.header.stamp = fs[0]
            else:
                m.header.stamp = self.node.get_clock().now().to_msg()
            m.point.x, m.point.y, m.point.z = float(u), float(v), float(s3.point.z)
            self.px_pub.publish(m)
            time.sleep(0.1)
        note('pixel seed done; waiting for a fresh TRACKING (up to 60 s)')
        self.wait_tracking(time.time(), 60.0)


def make_handler(sel):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, code, obj):
            b = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(b)))
            self.end_headers()
            self.wfile.write(b)

        def do_GET(self):
            if self.path.split('?')[0] != '/object-target':
                return self._send(404, dict(ok=False, error='unknown route'))
            return self._send(200, sel.api())

        def do_POST(self):
            if self.path.split('?')[0] != '/object-target':
                return self._send(404, dict(ok=False, error='unknown route'))
            if self.headers.get('X-Dashboard') != '1':
                return self._send(403, dict(ok=False, error='dashboard header required'))
            try:
                n = int(self.headers.get('Content-Length', '0'))
                if not 0 < n <= 1024:
                    raise ValueError('invalid body')
                body = json.loads(self.rfile.read(n))
                if isinstance(body, dict) and set(body) == {'prompt'}:
                    gen = sel.request_prompt(body['prompt'])
                elif isinstance(body, dict) and set(body) == {'target', 'click'}:
                    gen = sel.request(body['target'], body['click'])
                else:
                    raise ValueError('body must be {"target", "click"} or {"prompt"}')
            except (ValueError, TypeError) as e:
                return self._send(400, dict(ok=False, error=str(e)))
            except Fail as e:
                return self._send(409, dict(ok=False, error=str(e)))
            except Exception as e:  # noqa: BLE001
                return self._send(500, dict(ok=False, error=f'{type(e).__name__}: {e}'))
            return self._send(202, dict(ok=True, generation=gen))
    return H


def main():
    ros = RosIO()
    sel = Selector(ros)
    threading.Thread(target=sel.poll_teleop, daemon=True).start()
    sel.adopt_current()
    # The tower dashboard (:8788 mesh_server) proxies here over the lab LAN, so bind that address.
    bind = os.environ.get('SELECTOR_BIND', '127.0.0.1')
    srv = ThreadingHTTPServer((bind, 8898), make_handler(sel))
    log(f'object selector listening on {bind}:8898')
    srv.serve_forever()


if __name__ == '__main__':
    main()
