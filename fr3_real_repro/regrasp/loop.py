"""The regrasp loop: TRACKING/MOVED/REUSE/REGEN/OCCLUDED/LOCKED.

Engine is pure (numpy + disk only) and fully replay-testable; the ROS shell
at the bottom owns subscriptions and the poll clock. The loop NEVER commands
the robot and NEVER publishes /grasp_target (Murilo's simplification): its
outputs are run dirs on disk, /drill_cloud_state, and the TF binding file
(grasp_binding.json) that the coord lane's publisher follows. OCCLUDED and
trust therefore gate REBINDING, never a target stream.
"""
import json
import os
import sys
import time
from collections import deque

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from accumulator import Accumulator
from contracts import (COUNT_FRAC, GRIP_HELD_BELOW, GRIP_OPEN_MIN,
                       LOCK_PATH, LOCK_STALE_S,
                       MESH_URL, OCC_MEDIAN_S, RUNS_DIR, STABLE_S,
                       STILL_HOLD_S, STILL_RATE, STILL_WINDOW_S,
                       ZED2_NONE_STALE_S, DrillState, LockState, read_lock)
from motion import cloud_yaw, eval_trigger, icp_se2, invert_se2, top_cluster
from occlusion import occludes
from publish import grasp_target_from_plan, write_pointer, write_run
from selection import (NoFeasibleGrasp, apply_gates, plan_candidate,
                       rank_pool, select, transform_grasps)

BOOTSTRAP_TICKS = 3
STATE_HZ = 2.0


def _pid_alive(pid):
    if pid is None or pid <= 0:
        return False
    try:
        os.kill(pid, 0)
    except OSError:
        return False
    return True


def still_ok(jq, now):
    """REGEN allowed only with STILL_HOLD_S of continuous stillness ending
    now. Moving pairs age out of the window on their own, so this is pure
    over the ring: no motion state to maintain. Fails closed (no history,
    stale joints, or any moving pair in the window -> False)."""
    if len(jq) < 2:
        return False
    if jq[-1][0] < now - STILL_WINDOW_S:
        return False  # joints stopped flowing
    horizon = now - STILL_HOLD_S
    if jq[0][0] > horizon:
        return False  # insufficient evidence
    pairs = list(zip(jq, list(jq)[1:]))
    for (t0, q0), (t1, q1) in reversed(pairs):
        if t1 <= horizon:
            break
        if t1 <= t0:
            continue  # duplicate stamp proves nothing either way
        rate = float(np.abs(np.asarray(q1) - np.asarray(q0)).max()
                     / (t1 - t0))
        if rate > STILL_RATE:
            return False
    return True


def skip_unpaired_tick(why):
    """poll_tick None-reasons that mean 'zed1-only data between ZED2 mask
    arrivals' (as opposed to a dead viewer). The shell skips these ticks --
    republishes the last state, keeps the held target -- instead of
    latching occlusion. Past ZED2_NONE_STALE_S the shell goes untrusted."""
    return why.startswith('unpaired:')


class Engine:
    def __init__(self, run_base=RUNS_DIR, graspgen_fn=None, mesh_V=None,
                 mesh_url=MESH_URL, cams=(), decide_log=None, pid_live=None,
                 yaw_sink=None):
        self.run_base = run_base
        self.graspgen_fn = graspgen_fn      # (cloud_npy, num) -> grasps_npy
        self.mesh_V = mesh_V                # or lazy-loaded from mesh_url
        self.mesh_url = mesh_url
        self.cams = list(cams)
        self.pid_live = pid_live or _pid_alive
        self._last_n = 0
        self.decide_log = decide_log        # fn(str) or None
        self.yaw_sink = yaw_sink            # fn(dict) or None: live yaw feed
        self._last_yaw_rel = None           # continuity guard for PCA flips
        self.acc = Accumulator()
        self.samples = []                   # (wall, cx, cy, yaw, n)
        self.anchor = None                  # dict(cloud, grasps, run_id, ...)
        self.state = 'TRACKING'
        self.pick = None
        self.plan = None
        self.trusted = False
        self.reason = 'boot'
        self._move_pending = True     # bootstrap is just the first resolve
        self._jq = deque(maxlen=48)   # (wall, q7) for the REGEN stillness gate
        self._pending_ticks = 0
        self.held_age_s = 0.0
        self._held_since = 0.0
        self._held_closed = False  # gripper latch: close freezes, open resumes

    def log(self, d):
        if self.decide_log is not None:
            self.decide_log(json.dumps(d))

    def _top_stats(self, P):
        T = top_cluster(P)
        c = T.mean(axis=0)
        return float(c[0]), float(c[1]), cloud_yaw(P), int(len(P))

    @staticmethod
    def _wrap180(a):
        return (float(a) + 180.0) % 360.0 - 180.0

    def _emit_yaw(self, now):
        """Live yaw feed for the TF publisher: median tick yaw over the last
        2 s, relative to the ANCHOR yaw (same reference the move trigger
        uses). Held ticks (LOCKED/OCCLUDED early returns) never reach here,
        so the file goes stale there and the publisher holds its last
        correction instead of riding noise. Flip-guarded: PCA major-axis
        yaw has a 180 deg sign ambiguity, so a >45 deg single-tick step is
        a flip, not physics (hand spins are far slower) -> hold last. The
        same bar gates the first emit after (re)anchor, where a flipped
        baseline would otherwise report ~180 deg."""
        if self.yaw_sink is None or self.anchor is None:
            return
        win = [s for s in self.samples if now - s[0] <= 2.0]
        if not win:
            return
        ref = win[-1][3]
        rels = [self._wrap180(s[3] - ref) for s in win]
        yaw_med = ref + float(np.median(rels))
        raw = self._wrap180(yaw_med - float(self.anchor['yaw']))
        last = self._last_yaw_rel
        if last is None:
            rel = 0.0 if abs(raw) > 90.0 else raw
        elif abs(self._wrap180(raw - last)) > 45.0:
            rel = last
        else:
            rel = last + self._wrap180(raw - last)
        self._last_yaw_rel = rel
        self.yaw_sink(dict(yaw_deg=round(yaw_med, 2),
                           anchor_yaw_deg=round(float(self.anchor['yaw']), 2),
                           rel_deg=round(rel, 2),
                           rel_rad=round(float(np.radians(rel)), 4),
                           wall=round(now, 2), run_id=self.anchor['run_id'],
                           spread_deg=round(float(max(rels) - min(rels)), 2)))

    def step(self, tick, joints=None, width=None, lock=None,
             lock_corrupt=False, now=None):
        """One tick. Returns (DrillState, GraspTarget|None). Pure + disk."""
        now = time.time() if now is None else now
        if joints is not None:
            self._jq.append((now, np.asarray(joints, float)))
        P = np.asarray(tick.pts, float)
        cx, cy, yaw, n = self._top_stats(P)
        n = len(P)

        # 1. lock: HELD_OBJECT / live RUN_ACTIVE / corrupt all stop target.
        self._last_n = n
        if lock_corrupt:
            return self._hold('LOCKED', now, 'lock-corrupt')
        if lock is not None:
            blocks, why = lock.blocks(now, self.pid_live)
            if blocks:
                return self._hold('LOCKED', now, f'lock-{why}')

        # 1b. held: any live close latches (empty closes included: the
        # target is suppressed either way, and reopen resumes). While
        # latched the drill may ride the hand, so sample nothing, fire
        # nothing, emit no yaw. Hysteresis mirrors the TF publisher
        # (close <0.070, open >=0.075); width None leaves the latch alone.
        if width is not None:
            if width < GRIP_HELD_BELOW:
                self._held_closed = True
            elif width >= GRIP_OPEN_MIN:
                self._held_closed = False
        if self._held_closed:
            return self._hold('LOCKED', now, 'held-closed')

        # 2. occlusion: FK silhouette vs the HELD box, or count collapse.
        if self.anchor is not None:
            if self._occluded(joints, n, now):
                self._move_pending = False    # frozen: re-evaluate on clear
                self.acc.reset()
                return self._hold('OCCLUDED', now, 'occluded')
        last_good = (self.anchor['n'] if self.anchor else n) or n

        # 3. sample + trigger vs the FIXED anchor.
        self.samples.append((now, cx, cy, yaw, n))
        self.samples = [s for s in self.samples if now - s[0] <= STABLE_S + 2]
        self._emit_yaw(now)
        fired, why, _ = ((False, 'no-anchor', None) if self.anchor is None
                         else eval_trigger(self.samples, self.anchor['cxy'],
                                           self.anchor['yaw'], last_good))
        if self.anchor is not None and not fired \
                and not self._move_pending:
            self.acc.add(tick)
            self.state, self.trusted, self.reason = 'TRACKING', True, why
            self.log(dict(t=round(now, 1), state='TRACKING', trusted=True,
                          reason=why, n=n))
            return self._out(now, width)

        # 4. confirmed move (or bootstrap): accumulate POST-MOVE ticks only,
        # then solve once. Anchoring on the mixed pre+post stack inflated
        # anchor_n and tripped the count gate on the very next tick. Once
        # pending, the trigger is not consulted: it resolves, never cancels
        # (a pose that returned just reuses at ~identity: harmless).
        if not self._move_pending:
            self._move_pending = True
            self._pending_ticks = 0
            self.acc.reset()
        self.acc.add(tick)
        self._pending_ticks += 1
        if self._pending_ticks < BOOTSTRAP_TICKS:
            return self._hold('TRACKING', now,
                              'accumulating-post-move' if self.anchor
                              else 'bootstrap', trusted=False)
        self._move_pending = False
        new_cloud = self.acc.cloud()
        if self.anchor is None:
            return self._regen(now, 'bootstrap')
        self.state = 'MOVED'
        res = icp_se2(self.anchor['cloud'], new_cloud)
        if res.rigid and self._icp_agrees_with_pca(res):
            return self._reuse(now, res, new_cloud)
        if res.rigid:
            iyaw, _ = invert_se2(res)
            self.log(dict(t=round(now, 1), state='MOVED',
                          guard='icp-pca-disagree',
                          icp_yaw=round(iyaw, 1)))
            return self._regen(now, 'icp-pca-disagree')
        return self._regen(now, f'tipped ({res.reason})')

    def _icp_agrees_with_pca(self, res):
        """REUSE guard: a rigid ICP fit outside its yaw grid can still lock
        a wrong basin on smooth drill geometry (replay: true +30 deg, ICP
        +12 deg yet rigid). Require the fit within 12 deg of the PCA yaw
        delta (good locks agree to ~3 deg); else REGEN (slow but correct)
        instead of a jumped grasp."""
        if not self.samples or self.anchor is None:
            return True
        pca = self._wrap180(self.samples[-1][3]
                            - float(self.anchor['yaw']))
        iyaw, _ = invert_se2(res)
        return abs(self._wrap180(iyaw - pca)) <= 12.0

    def _occluded(self, joints, n, now):
        # Median over the window (same pattern as the trigger's count gate):
        # one thin tick between mask pairings must not flap the target, but
        # a sustained collapse still latches. Anchor n is per-tick scale.
        recent = [s[4] for s in self.samples
                  if now - s[0] <= OCC_MEDIAN_S] + [n]
        med = float(np.median(recent))
        if self.anchor['n'] and med < COUNT_FRAC * self.anchor['n']:
            return True
        if joints is None or not self.cams:
            return False
        lo = np.array(self.anchor['box_lo'])
        hi = np.array(self.anchor['box_hi'])
        return any(occludes(joints, c, lo, hi, self._fk) for c in self.cams)

    @staticmethod
    def _fk(q, pts=False):
        sys.path.insert(0, '/home/nexus/spot-teleop/fr3_real_repro/grasp_loop')
        from grasp_plan_lib import fk
        return fk(q, pts=pts)

    def _hold(self, state, now, reason, trusted=False):
        if self.state != state:
            self._held_since = now
        self.state, self.trusted, self.reason = state, trusted, reason
        self.held_age_s = now - self._held_since
        self.log(dict(t=round(now, 1), state=state, trusted=trusted,
                      reason=reason, n=self._last_n))
        return self._out(now, None)

    def _out(self, now, width):
        st = DrillState(state=self.state, run_id=self._run_id(),
                        anchor_id=self.anchor['run_id'] if self.anchor else '',
                        age_s=round(self.held_age_s if not self.trusted else 0.0, 1),
                        trusted=self.trusted, n_pts=self._last_n,
                        reason=self.reason)
        tgt = None
        if (self.trusted and self.plan is not None and self.state in
                ('TRACKING', 'REUSE') and width is not None
                and width >= GRIP_OPEN_MIN):
            tgt = grasp_target_from_plan(self.plan, self._run_id())
            tgt.stamp = now
        return st, tgt

    def _run_id(self):
        return self.anchor['run_id'] if self.anchor else ''

    def _new_run_id(self, kind):
        return time.strftime('%Y%m%d_%H%M%S') + f'_drill_loop_{kind}'

    def _set_anchor(self, run_id, cloud, grasps, pick, plan):
        T = top_cluster(cloud)
        c = T.mean(axis=0)
        # Anchor n is the PER-TICK scale (the count gate compares single
        # live ticks against it), not the stacked cloud size. Read before
        # the reset below; fall back to the cloud size if unsettled.
        tick_n = self.acc.tick_mean_n() or int(len(cloud))
        self.anchor = dict(run_id=run_id, cloud=np.asarray(cloud, float),
                           grasps=np.asarray(grasps, float),
                           cxy=(float(c[0]), float(c[1])),
                           yaw=cloud_yaw(cloud), n=tick_n,
                           box_lo=T.min(axis=0).tolist(),
                           box_hi=T.max(axis=0).tolist())
        self.pick, self.plan = pick, plan
        self.samples = []
        self.acc.reset()
        self._last_yaw_rel = None

    def adopt_run(self, run_dir):
        """Adopt a valid run dir as the anchor without regenerating
        (restart continuity: no bootstrap inference, no dashboard flip).
        Refuses plans with problems. Anchor n is approximate per-tick
        scale (loop regens stack BOOTSTRAP_TICKS ticks); the count gate
        treats it as a loose lower bound, FK stays the arm detector."""
        run_dir = run_dir.rstrip('/')
        plan = json.load(open(os.path.join(run_dir, 'plan.json')))
        if plan.get('problems'):
            raise ValueError(
                f'adopt refused: plan problems {plan.get("problems")}')
        pick = json.load(open(os.path.join(run_dir, 'pick.json')))
        cloud = np.load(os.path.join(run_dir, 'cloud.npy'))[:, :3].astype(
            float)
        grasps = np.load(os.path.join(run_dir, 'grasps.npy')).astype(float)
        T = top_cluster(cloud)
        c = T.mean(axis=0)
        run_id = plan.get('run') or os.path.basename(run_dir)
        self.anchor = dict(
            run_id=run_id, cloud=np.asarray(cloud, float),
            grasps=np.asarray(grasps, float),
            cxy=(float(c[0]), float(c[1])), yaw=cloud_yaw(cloud),
            n=max(1, int(len(cloud)) // BOOTSTRAP_TICKS),
            box_lo=T.min(axis=0).tolist(), box_hi=T.max(axis=0).tolist())
        self.pick, self.plan = pick, plan
        self.samples = []
        self.acc.reset()
        self._move_pending = False
        self._last_yaw_rel = None
        self.state, self.trusted, self.reason = \
            'TRACKING', True, 'adopted'
        self.log(dict(t=round(time.time(), 1), state='TRACKING',
                      trusted=True, reason='adopted', run=run_id))
        return run_id

    def _ensure_mesh(self):
        # Map mesh for the planner's clearance checks. Loaded lazily on the
        # first REUSE or REGEN (21:23:52Z crash: after a restart the first
        # event was a REUSE, which used mesh_V=None). '' = ok, else reason.
        if self.mesh_V is not None:
            return ''
        try:
            sys.path.insert(
                0, '/home/nexus/spot-teleop/fr3_real_repro/grasp_loop')
            from grasp_plan_lib import load_mesh
            self.mesh_V = np.asarray(load_mesh(self.mesh_url), float)
            return ''
        except Exception as e:
            return f'mesh-failed: {e}'

    def _reuse(self, now, res, new_cloud):
        # Same select() as REGEN (gates + comfort rule); only the grasp set
        # is carried instead of sampled. Falls through to REGEN when the
        # carried set has nothing plannable.
        yaw, t = invert_se2(res)
        G2 = transform_grasps(self.anchor['grasps'], yaw, t[0], t[1])
        meta = dict(centroid=new_cloud.mean(axis=0).tolist(),
                    extent=(new_cloud.max(axis=0) - new_cloud.min(axis=0)
                            ).tolist(),
                    top=float(new_cloud[:, 2].max()))
        why = self._ensure_mesh()
        if why:
            return self._hold('TRACKING', now, why, trusted=False)
        run_id = self._new_run_id('reuse')
        try:
            from selection import select as _select
            pick, plan, depth, ranked, _ = _select(
                G2, new_cloud, meta, self.mesh_V, run=run_id)
        except NoFeasibleGrasp as e:
            return self._regen(now, f'reuse-plan-empty: {e}')
        pick['source'] = 'regrasp reuse'
        self._write(run_id, new_cloud, G2, pick, ranked, plan, depth,
                    dict(kind='reuse', icp=res.__dict__), False)
        self._set_anchor(run_id, new_cloud, G2, pick, plan)
        self.state, self.trusted = 'REUSE', True
        self.reason = f'reused {res.reason}'
        self.log(dict(t=round(now, 1), state='REUSE', run=run_id,
                      pick=pick['pos'], icp=res.__dict__))
        return self._out(now, GRIP_OPEN_MIN)

    def _regen(self, now, reason):
        # Stillness gate FIRST (before any mesh/inference work): a GPU
        # inference stalls the MPC ~0.5 s, worst with the hand near the
        # drill. REUSE stays allowed (pure CPU). While deferred the move
        # stays pending (retried every tick, acc keeps growing) and the
        # HELD target keeps publishing if this REGEN follows a move.
        if not still_ok(self._jq, now):
            self._move_pending = True  # retry next tick, keep the acc
            why = ('regen-deferred-no-joints'
                   if len(self._jq) < 2
                   or self._jq[-1][0] < now - STILL_WINDOW_S
                   else 'regen-deferred-moving')
            if self.anchor is None:
                return self._hold('TRACKING', now, why, trusted=False)
            self.state, self.trusted, self.reason = \
                'TRACKING', True, why
            self.log(dict(t=round(now, 1), state='TRACKING', trusted=True,
                          reason=why))
            return self._out(now, GRIP_OPEN_MIN)
        why = self._ensure_mesh()
        if why:
            return self._hold('TRACKING', now, why, trusted=False)
        if self.graspgen_fn is None:
            return self._hold('TRACKING', now, 'regen-unavailable',
                              trusted=False)
        cloud = self.acc.cloud()
        if len(cloud) < 200:
            return self._hold('TRACKING', now, 'cloud-too-small',
                              trusted=False)
        run_id = self._new_run_id('regen')
        cn = f'/tmp/regrasp_acc_{run_id}.npy'
        np.save(cn, cloud.astype(np.float32))
        try:
            gn = self.graspgen_fn(cn, 200)
            G = np.load(gn).astype(float)
        except Exception as e:
            return self._hold('TRACKING', now, f'graspgen-failed: {e}',
                              trusted=False)
        finally:
            if os.path.exists(cn):
                os.remove(cn)
        meta = dict(centroid=cloud.mean(axis=0).tolist(),
                    extent=(cloud.max(axis=0) - cloud.min(axis=0)).tolist(),
                    top=float(cloud[:, 2].max()))
        try:
            from selection import select as _select
            pick, plan, depth, ranked, tried = _select(
                G, cloud, meta, self.mesh_V, run=run_id)
        except NoFeasibleGrasp as e:
            return self._hold('TRACKING', now, f'select-empty: {e}',
                              trusted=False)
        self._write(run_id, cloud, G, pick, ranked, plan, depth,
                    dict(kind='regen', trigger=reason), False)
        self._set_anchor(run_id, cloud, G, pick, plan)
        self.state, self.trusted, self.reason = 'TRACKING', True, 'regened'
        self.log(dict(t=round(now, 1), state='REGEN', run=run_id,
                      pick=pick['pos']))
        return self._out(now, GRIP_OPEN_MIN)

    def _write(self, run_id, cloud, G, pick, ranked, plan, depth, info,
               starved):
        write_run(self.run_base, run_id, cloud, G, pick,
                  [dict(idx=e['idx'], score=e['score'], conf=e['conf'],
                        metrics=e['metrics'], weights=e.get('weights', {}))
                   for e in ranked],
                  plan, depth, info, starved)


# ---------------------------------------------------------------- ROS shell
# Only main() below touches ROS. Everything above stays importable and
# replay-testable without it.

ZED_T = {
    'zed1': '/home/nexus/spot-teleop/fr3_real_repro/zed2_align/'
            'zed1_extrinsic_20261106_robotfit7_dz.json',
    'zed2': '/home/nexus/spot-teleop/fr3_real_repro/zed2_align/'
            'zed2_extrinsic_20261106_robotfit7_dz.json',
}
INFO_TOPICS = {
    # Depth infos stand in for RGB: same 640x360 rectified rig, and the
    # 15 px footprint margin dwarfs any K difference.
    'zed1': '/zed1/zed_node/depth/camera_info',
    'zed2': '/zed2/zed_node/depth/camera_info',
}


def main():
    import argparse
    import rclpy
    import tf2_ros
    from sensor_msgs.msg import CameraInfo, JointState
    from std_msgs.msg import String

    import graspgen
    from contracts import (BIND_BASE_FRAME, DECISIONS_LOG, DRILL_STATE_TOPIC,
                           GRASP_BINDING, JOINT_STATES_TOPIC, LOCK_PATH,
                           RUNS_DIR, STATE_DIR, TARGET_OBJECT_FRAME,
                           VIEWER_GRASP_URL)
    from occlusion import CamModel
    from publish import bind_allowed, write_grasp_binding
    from source import poll_tick

    ap = argparse.ArgumentParser()
    ap.add_argument('--poll-hz', type=float, default=2.0)
    ap.add_argument('--viewer-url', default=VIEWER_GRASP_URL)
    ap.add_argument('--runs', default=RUNS_DIR)
    ap.add_argument('--lock', default=LOCK_PATH)
    ap.add_argument('--anchor-run', default='',
                    help='adopt this run dir as the anchor (no bootstrap); '
                         '"binding" adopts the run the binding file points '
                         'to, so a restart never trails the bound run')
    a = ap.parse_args()

    os.makedirs(STATE_DIR, exist_ok=True)
    dlog = open(DECISIONS_LOG, 'a', buffering=1)

    rclpy.init()
    node = rclpy.create_node('regrasp_loop')
    state_pub = node.create_publisher(String, DRILL_STATE_TOPIC, 10)
    tf_buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buf, node)
    js = {}
    node.create_subscription(
        JointState, JOINT_STATES_TOPIC,
        lambda m: js.update(zip(m.name, m.position)), 10)
    infos = {}

    def on_info(cam):
        def cb(m):
            infos.setdefault(cam, m)
        return cb

    for cam, topic in INFO_TOPICS.items():
        node.create_subscription(CameraInfo, topic, on_info(cam), 10)

    def decide_log(line):
        dlog.write(line + '\n')

    try:
        transport = graspgen.LabTransport()
    except FileNotFoundError as e:
        node.get_logger().error(f'{e}; regen unavailable')
        transport = None

    def graspgen_fn(cloud_npy, num):
        return graspgen.infer(cloud_npy, num, transport)

    yaw_path = os.path.join(STATE_DIR, 'live_yaw.json')

    def yaw_sink(d):
        tmp = yaw_path + '.tmp'
        with open(tmp, 'w') as f:
            json.dump(d, f)
        os.replace(tmp, yaw_path)

    eng = Engine(run_base=a.runs,
                 graspgen_fn=graspgen_fn if transport else None,
                 decide_log=decide_log, yaw_sink=yaw_sink)
    if a.anchor_run:
        anchor = a.anchor_run
        if anchor == 'binding':
            try:
                anchor = os.path.dirname(json.load(open(GRASP_BINDING))
                                         ['plan_path'])
            except (OSError, ValueError, KeyError) as e:
                node.get_logger().error(
                    f'binding unreadable ({e}); bootstrapping')
                anchor = ''
        if anchor:
            try:
                run_id = eng.adopt_run(anchor)
                node.get_logger().info(f'adopted anchor {run_id}')
            except (ValueError, FileNotFoundError, KeyError) as e:
                node.get_logger().error(
                    f'adopt failed ({e}); bootstrapping')
    last = {'st': None, 'ok_wall': time.time()}
    # Already bound? Don't rewrite on startup: the file's offset was
    # sampled at the right pose, and the drill may be held/moved now.
    try:
        bound_run = str(json.load(open(GRASP_BINDING)).get('run_id', ''))
    except (OSError, ValueError):
        bound_run = ''
    bound = {'run_id': bound_run, 'fail_run': ''}

    def cam_models():
        if len(infos) < 2:
            return []
        cams = []
        for cam, m in infos.items():
            T = np.array(json.load(open(ZED_T[cam]))['T_base_cam'], float)
            K = np.array(m.k, float).reshape(3, 3)
            cams.append(CamModel(K[0, 0], K[1, 1], K[0, 2], K[1, 2],
                                 m.width, m.height, T[:3, :3], T[:3, 3]))
        return cams

    def poll_cb():
        if not eng.cams:
            eng.cams = cam_models()
        tick, why = poll_tick(a.viewer_url)
        if tick is None:
            if skip_unpaired_tick(why) and last['st'] is not None:
                if time.time() - last['ok_wall'] > ZED2_NONE_STALE_S:
                    st, _ = eng._hold('TRACKING', time.time(),
                                      'zed2-none-stale', trusted=False)
                    last['st'] = st
                    state_pub.publish(String(data=st.to_json()))
                    return
                # Between mask arrivals: keep the last state (no flap).
                state_pub.publish(String(data=last['st'].to_json()))
                return
            st, _ = eng._hold('OCCLUDED', time.time(), f'source-{why}')
            last['st'] = st
            state_pub.publish(String(data=st.to_json()))
            return
        last['ok_wall'] = time.time()
        joints = None
        try:
            joints = np.array([js[f'fr3_joint{i}'] for i in range(1, 8)])
        except KeyError:
            pass
        width = None
        if 'fr3_finger_joint1' in js:
            width = js['fr3_finger_joint1'] + js['fr3_finger_joint2']
        lock, corrupt = read_lock(a.lock)
        st, tgt = eng.step(tick, joints=joints, width=width, lock=lock,
                           lock_corrupt=corrupt)
        last['st'] = st
        state_pub.publish(String(data=st.to_json()))
        # Binder: every NEW trusted grasp rebinds to the target_object TF.
        # Deferred/held ticks keep their run_id, so no rebind; TF or guard
        # failures retry next tick (logged once per run).
        if (eng.anchor is not None and eng.trusted
                and eng.state in ('TRACKING', 'REUSE')
                and eng.anchor['run_id'] != bound['run_id']):
            run_id = eng.anchor['run_id']
            if not bind_allowed(run_id, GRASP_BINDING):
                if bound['fail_run'] != run_id:
                    bound['fail_run'] = run_id
                    decide_log(json.dumps(dict(
                        t=round(time.time(), 1),
                        state='BIND-REFUSED-OLDER', run=run_id)))
                return
            try:
                tr = tf_buf.lookup_transform(
                    BIND_BASE_FRAME, TARGET_OBJECT_FRAME,
                    rclpy.time.Time()).transform.translation
                write_grasp_binding(
                    GRASP_BINDING, eng.plan,
                    os.path.join(a.runs, run_id, 'plan.json'),
                    [tr.x, tr.y, tr.z])
                bound['run_id'] = run_id
                decide_log(json.dumps(dict(t=round(time.time(), 1),
                                           state='BOUND', run=run_id)))
            except Exception as e:
                if bound['fail_run'] != run_id:
                    bound['fail_run'] = run_id
                    decide_log(json.dumps(dict(
                        t=round(time.time(), 1), state='BIND-FAIL',
                        run=run_id, error=f'{type(e).__name__}: {e}')))

    node.create_timer(1.0 / a.poll_hz, poll_cb)
    node.get_logger().info(f'regrasp loop live (binder, runs={a.runs})')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
