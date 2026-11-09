"""Run publishing: atomic run dirs, dashboard pointer, message builders.

No ROS here: loop.py owns the publishers. Run dirs keep the existing
grasp_loop format plus plan.json + loop.json. Dashboard pointer dirs reuse
the proven 11-06 pattern (self-contained copy + NOTE.txt).
"""
import json
import os
import shutil

import numpy as np


def r2q(R):
    """Rotation matrix -> xyzw quat. Verbatim from
    regrasp/interim_grasp_target.py (coord lane) so the loop's /grasp_target
    matches the interim publisher bit-for-bit (that file parses args at
    import, so it cannot be imported)."""
    R = np.asarray(R, float)
    t = np.trace(R)
    if t > 0:
        s = 2 * np.sqrt(t + 1)
        return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                         (R[1, 0] - R[0, 1]) / s, s / 4])
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2 * np.sqrt(1 + R[i, i] - R[j, j] - R[k, k])
    q = np.zeros(4)
    q[i] = s / 4
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return q / np.linalg.norm(q)


def q2r(q):
    """xyzw quat -> 3x3 rotation matrix."""
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def Rz(yaw):
    """Rotation about world Z (base frame = world frame here: no base yaw)."""
    c, s = float(np.cos(yaw)), float(np.sin(yaw))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def apply_yaw_to_grasp(offset, R, yaw_rel):
    """Ride a table yaw without rebinding: rotate the bound offset and hand
    orientation by yaw_rel (rad) about Z. Returns (offset_list, quat_xyzw)."""
    W = Rz(yaw_rel)
    off2 = (W @ np.asarray(offset, float)).tolist()
    return off2, [float(v) for v in r2q(W @ np.asarray(R, float))]


def select_yaw_rel(yaw_doc, binding_run_id, bound_at, last_rel, now,
                   stale_s=2.0):
    """Which yaw correction to apply. Rules: no doc/stale doc -> hold last;
    binding newer than the doc -> 0 (the fresh binding already includes the
    yaw); run mismatch -> 0; else the doc's rel value. Never raises."""
    try:
        if not isinstance(yaw_doc, dict):
            return last_rel
        if str(yaw_doc.get('run_id', '')) != str(binding_run_id or ''):
            return 0.0
        wall = float(yaw_doc.get('wall', 0.0))
        if bound_at and wall < float(bound_at):
            return 0.0
        if now - wall > stale_s:
            return last_rel
        return float(yaw_doc.get('rel_rad', 0.0))
    except (TypeError, ValueError):
        return last_rel


def grasp_target_from_plan(plan, run_id=''):
    """The /grasp_target payload: fr3_hand pose = plan grasp_pos + hand_R."""
    from contracts import GraspTarget
    return GraspTarget(
        pos=[float(v) for v in plan['grasp_pos']],
        quat_xyzw=[float(v) for v in r2q(np.array(plan['hand_R'], float))],
        run_id=run_id)


def write_run(base, run_id, Pc, G, pick, ranked, plan, depth, loop_info,
              starved):
    """Write a complete run dir atomically (tmp + rename). Returns run_dir."""
    Pc = np.asarray(Pc, float)[:, :3]
    tmp = os.path.join(base, run_id + '.tmp')
    final = os.path.join(base, run_id)
    if os.path.exists(tmp):
        shutil.rmtree(tmp)
    os.makedirs(tmp)
    np.save(os.path.join(tmp, 'cloud.npy'), Pc.astype(np.float32))
    np.save(os.path.join(tmp, 'grasps.npy'), np.asarray(G, np.float32))
    json.dump(dict(n=int(len(Pc)),
                   centroid=Pc.mean(axis=0).round(4).tolist(),
                   top=float(Pc[:, 2].max()),
                   extent=(Pc.max(axis=0) - Pc.min(axis=0)).round(4).tolist(),
                   source='regrasp loop accumulator'),
              open(os.path.join(tmp, 'cloud.json'), 'w'), indent=1)
    json.dump(pick, open(os.path.join(tmp, 'pick.json'), 'w'), indent=1)
    json.dump([dict(idx=e['idx'], score=e['score'], conf=e['conf'],
                    **{k: v for k, v in e['metrics'].items()})
               for e in ranked],
              open(os.path.join(tmp, 'rank.json'), 'w'), indent=1)
    json.dump(dict(starved=bool(starved), pool=len(ranked),
                   total=int(np.asarray(G).shape[0]), gate='fixed-tcp_of_pose'),
              open(os.path.join(tmp, 'summary.json'), 'w'), indent=1)
    json.dump(dict(R=q2r(pick['quat_xyzw']).tolist(), pick=pick['pos'],
                   cloud=Pc.tolist(), run=run_id),
              open(os.path.join(tmp, 'grasp_plan_input.json'), 'w'))
    json.dump(plan, open(os.path.join(tmp, 'plan.json'), 'w'), indent=1)
    json.dump(dict(loop_info, depth=depth),
              open(os.path.join(tmp, 'loop.json'), 'w'), indent=1)
    os.rename(tmp, final)
    return final


def write_grasp_binding(path, plan, plan_path, tf_pos, now=None):
    """Bind a planned grasp to the target_object TF sample. Byte-contract
    with regrasp/tools/bind_grasp.py (coord lane, argparse-at-import so it
    cannot be imported): same 9 keys, rounding, guards and atomic replace.
    Raises ValueError on a dirty plan, missing hand_R or a wild offset."""
    import time
    from contracts import BIND_MAX_OFFSET_M
    if plan.get('problems'):
        raise ValueError(f'plan has problems: {plan.get("problems")}')
    if 'hand_R' not in plan:
        raise ValueError('plan has no hand_R (plan it with grasp_plan_lib)')
    hand = np.asarray(plan['grasp_pos'], float)
    tobj = np.asarray(tf_pos, float)
    off = hand - tobj
    if float(np.linalg.norm(off)) > BIND_MAX_OFFSET_M:
        raise ValueError(
            f'offset {float(np.linalg.norm(off)):.3f} m > '
            f'{BIND_MAX_OFFSET_M} m: TF not on this object?')
    b = dict(run_id=plan.get('run'), plan_path=os.path.abspath(plan_path),
             bound_at=time.time() if now is None else now,
             hand_pos=hand.tolist(), hand_R=plan['hand_R'],
             target_object_at_bind=tobj.round(4).tolist(),
             offset=off.round(5).tolist(), depth=plan.get('depth'),
             flip_deg=plan.get('flip_deg'))
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        json.dump(b, f, indent=1)
    os.replace(tmp, path)
    return path


def bind_allowed(candidate_run, binding_path):
    """Monotonic binding: loop run ids start with %Y%m%d_%H%M%S, so plain
    string comparison orders them chronologically. A restart must never
    rebind an OLDER run over a newer binding (11-06: adopted 203025 while
    205015 was bound). Missing/corrupt file never blocks a bind."""
    try:
        d = json.load(open(binding_path))
        current = str(d.get('run_id', ''))
    except (OSError, ValueError):
        return True
    if not current:
        return True
    return str(candidate_run) >= current


def write_pointer(base, run_id, src_run_dir, pick, plan, note):
    """Self-contained dashboard pointer (cloud + pick + plan + NOTE)."""
    final = os.path.join(base, run_id)
    os.makedirs(final)
    for f in ('cloud.npy', 'cloud.json'):
        shutil.copy(os.path.join(src_run_dir, f), os.path.join(final, f))
    json.dump(pick, open(os.path.join(final, 'pick.json'), 'w'), indent=1)
    json.dump(plan, open(os.path.join(final, 'plan.json'), 'w'), indent=1)
    open(os.path.join(final, 'NOTE.txt'), 'w').write(note)
    return final
