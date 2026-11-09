"""Shared contracts for the regrasp loop (Phase A).

Dataclasses + JSON (de)serialisers and the ONLY place that names topics,
paths and numeric bars. Pure stdlib: no numpy, no ROS, so every consumer
(including the tests) imports this for free.
"""
import json
import os
import time
from dataclasses import asdict, dataclass, field

# --- topics (domain 8, VM side; the ONLY topic table) ---
DRILL_STATE_TOPIC = '/drill_cloud_state'   # std_msgs/String, JSON, 2 Hz
GRASP_TARGET_TOPIC = '/grasp_target'        # geometry_msgs/PoseStamped, 10 Hz
JOINT_STATES_TOPIC = '/joint_states'

# --- endpoints / paths ---
VIEWER_GRASP_URL = 'http://127.0.0.1:8791/grasp.json'
MESH_URL = 'http://127.0.0.1:18089/mesh.bin'
# Gripper counts as open above this (finger1+finger2, m). Mirrors the
# interim publisher's --open-min so handoff keeps one semantic.
GRIP_OPEN_MIN = 0.075
# Gripper counts as closed below this (finger1+finger2, m). Mirrors the
# TF publisher's --close-below; with GRIP_OPEN_MIN forms the held latch.
GRIP_HELD_BELOW = 0.070
# REGEN stillness gate: a 0.5 s MPC stall is worst with the hand near the
# drill, so GraspGen inference waits until the arm has been still this long.
# Moving = max |dq| over the last 1 s above the bar (joint_states, domain 8).
STILL_RATE = 0.02           # rad/s
STILL_WINDOW_S = 1.0
STILL_HOLD_S = 2.0
RUNS_DIR = '/home/nexus/spot-teleop/fr3_real_repro/grasp_loop/runs'
STATE_DIR = '/home/nexus/spot-teleop/fr3_real_repro/regrasp/state'
# Lock written by grasp_test.sh; under STATE_DIR (not /tmp) so vm-viewer
# sees it through the /home/nexus/spot-teleop mount (11-06 contract change).
LOCK_PATH = os.path.join(STATE_DIR, 'regrasp.lock')
DECISIONS_LOG = os.path.join(STATE_DIR, 'decisions.log')
# TF binding (Murilo's simplification): the loop binds each new valid grasp
# to the target_object TF here; the coord lane's publisher follows TF+offset.
GRASP_BINDING = os.path.join(STATE_DIR, 'grasp_binding.json')
BIND_BASE_FRAME = 'fr3_link0'
TARGET_OBJECT_FRAME = 'target_object'
BIND_MAX_OFFSET_M = 0.35  # refuse: TF on another object / stale

# --- trigger bars (DESIGN.md + 11-06 critique) ---
MOVE_MM = 15.0          # top-8cm centroid move vs the ANCHOR (fixed ref)
YAW_DEG = 10.0          # cloud-PCA yaw vs the anchor
STABLE_S = 2.0          # new pose must hold this long before it counts
MOVE_STABLE_MM = 5.0    # max centroid spread inside the stability window
YAW_STABLE_DEG = 3.0    # max yaw spread inside the stability window
COUNT_FRAC = 0.7        # window-median count vs last good (mask-paired ticks)
OCC_MEDIAN_S = 2.0      # occlusion count gate: median over this window
ZED2_NONE_STALE_S = 5.0  # unpaired zed2_none past this -> untrusted
# --- SE(2) ICP accept bars (reuse152 measured RMSE 4.5 mm, fitness 0.96) ---
ICP_RMSE_MM = 12.0      # above the ~8 mm cloud seam floor, not below it
ICP_FIT = 0.6           # inlier fraction (<10 mm), both directions
ZEXT_MM = 15.0          # |z-extent change| veto (tipped, not slid)
# --- select gates (all three 11-06 picks pass these) ---
MIN_SCORE = 0.25
MIN_VERT = 0.6
WRAP_MM = (10.0, 70.0)
# 150, not 100: the executed idx152 sits at 108 mm off-center. The real
# guard against closing on air is the planner's pts_between_pads>=50.
MAX_OFF_MM = 150.0
# Tip bar: the slip happened with the higher tip 1.8 cm under the top; the
# grasps that held were 3.0-3.8 cm deep. Plans below this never win.
TIP_MIN_CM = 3.0
# Tilt bars (vert = cos(angle from straight-down)): 203025 slipped at a
# 25 deg tilt with the tip at 2.0 cm. Strict first, fallback only when
# nothing strict survives to a clean plan.
VERT_MIN = 0.966       # cos(15 deg)
VERT_FALLBACK = 0.9063  # cos(25 deg)
# Contact-patch footprint (hand frame: X width, Y closing, Z approach,
# coords relative to the TCP): mirrors the planner's hand slabs. The
# contact SIDES adapt per grasp (the drill surface sits wherever it
# sits -- idx152's spans |ry| 0..30 mm, not the 40 mm fingertips): each
# pad takes the outermost shell of its side as the first-contact patch.
PAD_RX = 0.011          # |rx| half-width (m)
PAD_RA = (-0.045, 0.0)  # approach band from the TCP back (m)
PAD_SHELL = 0.010       # shell depth below the side extreme (m)
PAD_PEAK_Q = 0.99       # robust side extreme (quantile, vs strays)
FLAT_TOL = 0.003        # within this of the first-contact plane (m)
FLAT_CELL = 0.005       # pad-area grid for coverage (m)
# --- lock ---
LOCK_STALE_S = 120.0
# --- source acceptance (absolute floor; the relative 70 % gate is in loop) ---
SOURCE_MIN_PTS = 1000
SOURCE_MAX_AGE_S = 6.0
# --- accumulator caps ---
ACC_MAX_TICKS = 8
ACC_MAX_PTS = 15000
# --- cleaning (dual2 recipe) ---
DEDUPE_M = 0.0025
TABLE_Z = 0.025
OUTLIER_R_M = 0.012
OUTLIER_MIN_NBR = 8


@dataclass
class Tick:
    """One fused live cloud poll from the viewer."""
    wall: float
    pts: list                       # Nx3 in fr3_link0
    n_zed2: int = 0
    zed2_src: str = 'none'          # mask | mask-held | none
    cloud_age_s: float = 999.0
    src_run: str = ''


@dataclass
class CloudSummary:
    n: int
    centroid: list
    top: float
    extent: list
    path: str = ''


@dataclass
class IcpResult:
    yaw_deg: float
    t: list                         # [tx, ty, 0.0]
    rmse_mm: float
    fitness: float
    zext_mm: float
    rigid: bool
    reason: str = ''


@dataclass
class LockState:
    """Mirror of /tmp/regrasp.lock written by grasp_test.sh."""
    state: str                      # RUN_ACTIVE | HELD_OBJECT
    pid: int = 0
    ts: float = 0.0
    run_dir: str = ''
    mtime: float = 0.0

    def blocks(self, now=None, pid_live=None):
        """True while regen + /grasp_target must stay off.

        HELD_OBJECT always blocks (survives crashes; cleared by `unlock`).
        RUN_ACTIVE blocks only with a live pid AND a fresh heartbeat;
        anything else is a stale/crashed run and must not brick the loop.
        """
        now = time.time() if now is None else now
        if self.state == 'HELD_OBJECT':
            return True, 'held-object'
        if self.state == 'RUN_ACTIVE':
            live = True if pid_live is None else bool(pid_live(self.pid))
            if live and (now - self.mtime) < LOCK_STALE_S:
                return True, 'run-active'
            return False, 'stale-run-ignored'
        return False, 'unknown-state-ignored'


def read_lock(path=LOCK_PATH, now=None):
    """Read the lock file. Returns (LockState|None, corrupt: bool).

    Missing file = no lock. Corrupt JSON with a FRESH mtime = BLOCK
    (fail-safe: a half-written lock may mean a run just started; the
    operator clears with `unlock`). Corrupt JSON older than LOCK_STALE_S
    is treated as no lock: a crashed writer's residue must not brick the
    loop forever (11-06 heartbeat bug precedent).
    """
    now = time.time() if now is None else now
    try:
        st = os.stat(path)
    except OSError:
        return None, False
    try:
        d = json.load(open(path))
        return LockState(state=str(d.get('state', '')),
                         pid=int(d.get('pid', 0)),
                         ts=float(d.get('ts', 0.0)),
                         run_dir=str(d.get('run_dir', '')),
                         mtime=st.st_mtime), False
    except (ValueError, KeyError, TypeError, OSError):
        if now - st.st_mtime > LOCK_STALE_S:
            return None, False
        return None, True


@dataclass
class DrillState:
    state: str                      # TRACKING | MOVED | REUSE | REGEN | OCCLUDED | LOCKED
    run_id: str = ''
    anchor_id: str = ''
    age_s: float = 0.0
    trusted: bool = True
    n_pts: int = 0
    icp: dict = field(default_factory=dict)
    reason: str = ''

    def to_json(self):
        return json.dumps(asdict(self))


@dataclass
class GraspTarget:
    """fr3_hand pose in fr3_link0 for the MPC 6D field (= plan grasp_pos/hand_R)."""
    pos: list
    quat_xyzw: list
    run_id: str = ''
    stamp: float = 0.0

    def to_json(self):
        return json.dumps(asdict(self))
