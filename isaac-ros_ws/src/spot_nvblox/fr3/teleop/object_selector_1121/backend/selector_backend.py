"""Offline, side-effect-free object-selector backend with identity guards.

Binds one selected target (drill / tennis_ball / sprite) through every stage:
mask -> collision map -> grasp binding -> held model -> Y authorization.
Every record must carry the same (target, generation, selection_id) identity;
anything missing, stale, ambiguous, or drill-specific for a non-drill target
fails closed with a human-readable reason.

No ROS, TF, map, tracker, controller or robot imports or calls. A production
adapter must feed fresh observations in and enforce the returned decision.
"""
from __future__ import annotations

import json
import re
import uuid
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Callable, Optional

import numpy as np


class Target(str, Enum):
    DRILL = 'drill'
    TENNIS_BALL = 'tennis_ball'
    SPRITE = 'sprite'


@dataclass(frozen=True)
class TargetSpec:
    label: str
    mask_fraction: tuple          # (min, max) of image area
    cloud_extent_m: tuple         # (min, max) largest axis-aligned extent
    min_cloud_points: int
    # Only targets flagged here may ever reach Ready. Ball/Sprite grasp
    # ranking, depth and held-sphere generation are not validated yet; do
    # not flip these without replay + live broker evidence for that object.
    planner_validated: bool


SPECS = {
    Target.DRILL: TargetSpec('Drill', (0.004, 0.20), (0.08, 0.40), 1000, True),
    Target.TENNIS_BALL: TargetSpec('Tennis ball', (0.0005, 0.05), (0.04, 0.12), 150, False),
    Target.SPRITE: TargetSpec('Sprite bottle', (0.001, 0.08), (0.06, 0.30), 250, False),
}

# Plan keys that only make sense for the drill planner. Seeing them on a
# non-drill binding means the drill pipeline produced it.
DRILL_ONLY_CHECK_KEYS = ('arm_links_in_drill_box',)

MASK_MAX_AGE_S = 1.5
BINDING_MAX_AGE_S = 30.0
MAP_MAX_AGE_S = 5.0        # map validation must be continuously refreshed
BROKER_MAX_AGE_S = 2.0     # broker preflight must be fresh for Y
FUTURE_TOL_S = 0.05        # clock skew tolerance; anything newer is refused
MAX_CROSS_IOU = 0.10       # selected mask vs any other object's mask
MIN_BOX_FILL = 0.15        # mask pixels inside prompt box / box area
MIN_IN_BOX = 0.90          # fraction of mask pixels inside prompt box
HELD_MIN_COVERAGE = 0.98

# plan_grasp only turns min_obstacle into a problem for S1/S6 (peer review of
# grasp_plan_lib ~L243), so problems == [] is not avoidance evidence for
# S2-S5. Re-check every stage here. Floors are stricter than the current
# (absent) gate, NOT tuned: lowest real value on 11-21 drill plans is
# 0.118 m obstacle / 0.166 m table. Changing them needs replay evidence.
REQUIRED_STAGES = ('S1_home_to_pre', 'S2_pre_to_grasp', 'S3_lift',
                   'S4_put_back', 'S5_grasp_to_pre', 'S6_pre_to_home')
STAGE_OBSTACLE_MIN_M = 0.10
STAGE_TABLE_MIN_M = 0.10
STAGE_JOINT_MARGIN_MIN = 0.05

# Gaps the offline checks cannot close; always reported, never "Ready" text
# that implies complete avoidance.
KNOWN_GAPS = (
    'Stage checks are per waypoint (n_waypoints=1), not swept along the path',
    'Hand/fingers and grasped object are not in the planner obstacle check',
    'Y stages run on direct JTC, not MPC avoidance',
    'nvblox unknown space is treated as free',
)


class SelectionRefused(RuntimeError):
    pass


@dataclass(frozen=True)
class Identity:
    target: str
    generation: int
    selection_id: str


@dataclass(frozen=True)
class MaskRecord:
    identity: Identity
    stamp: float
    mask: np.ndarray                      # HxW bool/uint8
    box_norm: tuple                       # prompt box x1,y1,x2,y2 in [0,1]
    tracker_ok: bool
    other_masks: dict = field(default_factory=dict)   # object id -> HxW mask


@dataclass(frozen=True)
class MapRecord:
    identity: Identity
    stamp: float                          # time map was (re)built/validated
    excluded_ids: frozenset               # ids carved out of collision map
    obstacle_ids: frozenset               # ids present as obstacles
    rebuilt_after_selection: bool         # stale geometry cleared
    target_voxels_absent: bool


@dataclass(frozen=True)
class BindingRecord:
    identity: Identity
    stamp: float
    run: str
    plan: dict
    cloud: np.ndarray                     # Nx3 object cloud used for grasp
    broker_pass: bool
    broker_run: str                       # run the broker actually validated
    broker_stamp: float                   # time of that broker preflight


@dataclass(frozen=True)
class HeldModelRecord:
    identity: Identity
    run: str
    frame: str
    spheres: tuple
    coverage: float


def _labels_in(name: str) -> set:
    tokens = set(re.split(r'[^a-z]+', name.lower()))
    found = set()
    for t in Target:
        words = t.value.split('_')
        if all(w in tokens for w in words) or t.value in name.lower():
            found.add(t.value)
    if 'ball' in tokens:
        found.add(Target.TENNIS_BALL.value)
    if 'bottle' in tokens:
        found.add(Target.SPRITE.value)
    return found


def _iou(a, b) -> float:
    a = np.asarray(a) > 0
    b = np.asarray(b) > 0
    if a.shape != b.shape:
        return 1.0                       # cannot compare -> treat as conflict
    union = np.logical_or(a, b).sum()
    return float(np.logical_and(a, b).sum() / union) if union else 0.0


class ObjectSelector:
    """Single-target selection state with fail-closed evaluation."""

    def __init__(self, clock: Callable[[], float]):
        self._clock = clock
        self.generation = 0
        self.selected: Optional[Identity] = None
        self.selected_at = None
        self.mask = self.map = self.binding = self.held = None
        self.stage_reasons = {}
        self.reason = 'No target selected'
        self.ready = False

    # ---- selection -------------------------------------------------------
    def select(self, target, *, engaged, busy, held) -> Identity:
        try:
            t = Target(target)
        except ValueError:
            raise SelectionRefused(f'Unknown target {target!r}') from None
        if engaged or busy or held:
            raise SelectionRefused('Disengage and release any held object before changing targets')
        self.generation += 1
        self.selected = Identity(t.value, self.generation, uuid.uuid4().hex)
        self.selected_at = self._clock()
        # Every prior record is invalidated; nothing carries over.
        self.mask = self.map = self.binding = self.held = None
        self.ready = False
        self.reason = 'Awaiting mask, map and grasp validation'
        return self.selected

    def submit(self, record):
        """Accept a stage record only if it belongs to the current selection."""
        if self.selected is None:
            raise SelectionRefused('No target selected')
        if record.identity != self.selected:
            raise SelectionRefused(
                f'Record identity {record.identity.target}/gen{record.identity.generation} '
                f'does not match selection {self.selected.target}/gen{self.selected.generation}')
        slot = {MaskRecord: 'mask', MapRecord: 'map', BindingRecord: 'binding',
                HeldModelRecord: 'held'}[type(record)]
        setattr(self, slot, record)
        self.ready = False

    # ---- checks ----------------------------------------------------------
    def _check_mask(self, now):
        m = self.mask
        if m is None:
            return 'Target mask missing'
        if not m.tracker_ok:
            return 'Tracker lost target'
        age = now - m.stamp
        if not (0 <= age <= MASK_MAX_AGE_S):
            return f'Target mask stale or future-stamped ({age:.2f}s)'
        arr = np.asarray(m.mask) > 0
        if arr.ndim != 2 or arr.size == 0:
            return 'Target mask malformed'
        frac = arr.mean()
        lo, hi = SPECS[Target(m.identity.target)].mask_fraction
        if not (lo <= frac <= hi):
            return f'Mask size {frac:.4f} outside {lo}-{hi} for {m.identity.target}'
        b = np.asarray(m.box_norm, float)
        if b.shape != (4,) or not np.isfinite(b).all() or not (0 <= b.min() and b.max() <= 1) \
                or b[0] >= b[2] or b[1] >= b[3]:
            return 'Prompt box invalid'
        h, w = arr.shape
        x1, y1, x2, y2 = (b * [w, h, w, h]).round().astype(int)
        inside = arr[y1:y2, x1:x2].sum()
        if inside / arr.sum() < MIN_IN_BOX:
            return 'Mask drifted outside its prompt box'
        if inside / max(1, (y2 - y1) * (x2 - x1)) < MIN_BOX_FILL:
            return 'Mask fills too little of its prompt box'
        if m.identity.target in m.other_masks:
            return 'Other-object masks include the selected target id'
        for oid, other in m.other_masks.items():
            if _iou(arr, other) > MAX_CROSS_IOU:
                return f'Mask overlaps {oid}; identity ambiguous'
        return None

    def _check_map(self, now):
        mp = self.map
        if mp is None:
            return 'Collision map not validated'
        sel = mp.identity.target
        if mp.excluded_ids != frozenset({sel}):
            return f'Map excludes {sorted(mp.excluded_ids)}, expected only {sel}'
        others = {t.value for t in Target} - {sel}
        if not others <= mp.obstacle_ids:
            return f'Non-target obstacles missing from map: {sorted(others - mp.obstacle_ids)}'
        if sel in mp.obstacle_ids:
            return 'Selected target still listed as obstacle'
        map_age = now - mp.stamp
        if map_age < -FUTURE_TOL_S:
            return 'Map stamp in the future'
        if map_age > MAP_MAX_AGE_S:
            return f'Map validation stale ({map_age:.1f}s)'
        if not (mp.rebuilt_after_selection and mp.stamp >= self.selected_at):
            return 'Map not rebuilt since selection; stale geometry possible'
        if self.mask is not None and mp.stamp < self.mask.stamp - MASK_MAX_AGE_S:
            return 'Map older than the validated mask'
        if not mp.target_voxels_absent:
            return 'Selected target voxels still in map'
        return None

    def _check_binding(self, now):
        bd = self.binding
        sel = Target(self.selected.target)
        if not SPECS[sel].planner_validated:
            return f'Grasp planner not validated for {sel.value}'
        if bd is None:
            return 'Grasp not generated for selected target'
        labels = _labels_in(bd.run)
        if labels and labels != {sel.value}:
            return f'Binding run {bd.run!r} names {sorted(labels)}, not {sel.value}'
        if sel is not Target.DRILL:
            for st, checks in (bd.plan.get('checks') or {}).items():
                if any(k in checks for k in DRILL_ONLY_CHECK_KEYS):
                    return f'Binding produced by drill-specific planner ({st})'
        if bd.plan.get('run') != bd.run:
            return 'Plan file run does not match binding run'
        if bd.broker_run != bd.run or not bd.broker_pass:
            return 'Broker preflight did not pass for this binding'
        if bd.plan.get('problems'):
            return f'Plan problems: {bd.plan["problems"]}'
        stage_problem = check_stages(bd.plan)
        if stage_problem:
            return stage_problem
        if sel is Target.TENNIS_BALL:
            bv = bd.plan.get('ball_validation')
            if not (isinstance(bv, dict) and bv.get('ok') is True and bv.get('run') == bd.run):
                return 'Ball-specific grasp validation missing for this run'
        if bd.stamp < self.selected_at:
            return 'Binding predates selection'
        if now - bd.stamp < -FUTURE_TOL_S:
            return 'Binding stamp in the future'
        if now - bd.stamp > BINDING_MAX_AGE_S:
            return 'Binding stale'
        b_age = now - bd.broker_stamp
        if b_age < -FUTURE_TOL_S:
            return 'Broker stamp in the future'
        if b_age > BROKER_MAX_AGE_S:
            return f'Broker preflight stale ({b_age:.1f}s); re-run before Y'
        if bd.broker_stamp < bd.stamp:
            return 'Broker preflight predates binding'
        cloud = np.asarray(bd.cloud, float)
        spec = SPECS[sel]
        if cloud.ndim != 2 or cloud.shape[1] != 3 or len(cloud) < spec.min_cloud_points \
                or not np.isfinite(cloud).all():
            return f'Grasp cloud too small or malformed for {sel.value}'
        ext = float(np.ptp(cloud, axis=0).max())
        lo, hi = spec.cloud_extent_m
        if not (lo <= ext <= hi):
            return f'Grasp cloud extent {ext:.3f}m implausible for {sel.value}'
        return None

    def _check_held(self, now):
        hm = self.held
        if hm is None:
            return 'Held-object collision model unavailable'
        if self.binding is None or hm.run != self.binding.run:
            return 'Held model belongs to a different binding run'
        if hm.frame != 'fr3_hand' or not hm.spheres:
            return 'Held model malformed'
        for s in hm.spheres:
            if len(s) != 4 or not np.isfinite(s).all() or s[3] <= 0:
                return 'Held model sphere malformed'
        if hm.coverage < HELD_MIN_COVERAGE:
            return f'Held model coverage {hm.coverage:.3f} < {HELD_MIN_COVERAGE}'
        return None

    def evaluate(self) -> bool:
        now = self._clock()
        self.ready = False
        self.stage_reasons = {}
        if self.selected is None:
            self.reason = 'No target selected'
            return False
        first = None
        for name, fn in (('mask', self._check_mask), ('map', self._check_map),
                         ('grasp', self._check_binding), ('held', self._check_held)):
            try:
                r = fn(now)
            except Exception as exc:      # any surprise fails closed
                r = f'{name} check error: {exc}'
            self.stage_reasons[name] = r
            first = first or r
        self.reason = first or 'Ready'
        self.ready = first is None
        return self.ready

    def authorize_y(self, identity: Optional[Identity]) -> tuple:
        """Y may only fire for the exact selection that just evaluated Ready."""
        if identity is None or identity != self.selected:
            return False, 'Y bound to a different or missing selection'
        if not self.evaluate():
            return False, self.reason
        return True, 'Ready'

    # ---- API shape (GET /object-target) ---------------------------------
    def to_api(self) -> dict:
        def stage(name):
            r = self.stage_reasons.get(name, 'not evaluated')
            return {'ok': r is None, 'reason': r}
        sel = self.selected
        status = 'idle' if sel is None else ('ready' if self.ready else 'pending')
        return {
            'selected': sel.target if sel else None,
            'generation': self.generation,
            'selection_id': sel.selection_id if sel else None,
            'status': status,
            'ready': self.ready,
            'reason': self.reason,
            'mask': stage('mask'),
            'map': stage('map'),
            'grasp': stage('grasp'),
            'held': stage('held'),
            'binding_run': self.binding.run if self.binding else None,
            'known_gaps': list(KNOWN_GAPS),
        }


def check_stages(plan) -> Optional[str]:
    """Independent per-stage clearance gate; None if every stage passes."""
    checks = plan.get('checks')
    if not isinstance(checks, dict):
        return 'Plan has no stage checks'
    for st in REQUIRED_STAGES:
        c = checks.get(st)
        if not isinstance(c, dict):
            return f'Stage {st} missing checks'
        vals = {}
        for k in ('min_obstacle_m', 'min_table_clear_m', 'joint_limit_margin_rad'):
            v = c.get(k)
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not np.isfinite(v):
                return f'Stage {st} lacks {k}'
            vals[k] = float(v)
        if vals['min_obstacle_m'] < STAGE_OBSTACLE_MIN_M:
            return f'Stage {st} obstacle clearance {vals["min_obstacle_m"]:.3f} m < {STAGE_OBSTACLE_MIN_M}'
        if vals['min_table_clear_m'] < STAGE_TABLE_MIN_M:
            return f'Stage {st} table clearance {vals["min_table_clear_m"]:.3f} m < {STAGE_TABLE_MIN_M}'
        if vals['joint_limit_margin_rad'] < STAGE_JOINT_MARGIN_MIN:
            return f'Stage {st} joint margin {vals["joint_limit_margin_rad"]:.3f} rad too small'
        for k in DRILL_ONLY_CHECK_KEYS:
            if c.get(k) is True:
                return f'Stage {st} arm links inside target box'
    return None


# ---- loaders for real artifacts (read-only) ------------------------------
def load_binding(run_dir, identity, *, stamp, broker_pass, broker_run, broker_stamp) -> BindingRecord:
    run_dir = Path(run_dir)
    plan = json.loads((run_dir / 'plan.json').read_text())
    cloud = np.load(run_dir / 'cloud.npy')
    return BindingRecord(identity, stamp, run_dir.name, plan, cloud, broker_pass, broker_run,
                         broker_stamp)


def load_held_model(path, identity) -> HeldModelRecord:
    d = json.loads(Path(path).read_text())
    return HeldModelRecord(identity, d['run'], d['frame'],
                           tuple(tuple(s) for s in d['spheres']), float(d['coverage']))
