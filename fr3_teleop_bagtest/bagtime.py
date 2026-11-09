"""Pure (ROS-free) helpers for the FR3 bag-as-camera teleop test.

This module has no ROS imports so it can be unit-tested on any machine.
ROS entry points (bag_camera.py, run_take.py) import from here.
"""

import json
import math

# The 12 ZED protocol stages and their teleop exercise profiles.
# Profiles match zed_review_exercise in fr3_input_teleop.py.
STAGES = {
    '00_neutro': 'neutral',
    '01_giro_plano_imagem': 'roll',
    '02_inclinar_dedos': 'pitch',
    '03_virar_palma': 'yaw',
    '04_frente_tras': 'height',
    '05_direita_esquerda': 'horizontal',
    '05_direita_esquerda_2': 'horizontal',
    '06_cima_baixo': 'reach',
    '07_abrir_fechar': 'gripper',
    '08_cruz': 'cross',
    '09_gesto_em_movimento': 'translation',
    '10_perda_retorno': 'translation',
    '11_distancias': 'translation',
}

# Per-stage motion spec for the real robot, derived from the MuJoCo sim
# validation (clips_out/zed_protocol_analysis/simulation_review.json).
# Robot TCP is measured in fr3_link0 (meters); spans are peak-to-peak.
# Margins are intentionally loose: the live workspace box clips motion
# (Oct-31 run: 04 reached only z=3cm before the z floor stopped it).
STAGE_SPEC = {
    # stage: (primary axes in xyz indices, min primary span cm,
    #         max off-axis span cm, min orient excursion deg, max orient excursion deg)
    '00_neutro': dict(primary=(), min_primary_cm=0.0, max_off_cm=3.0,
                      min_oriexc_deg=0.0, max_oriexc_deg=8.0),
    '01_giro_plano_imagem': dict(primary=(), min_primary_cm=0.0, max_off_cm=3.0,
                                 min_oriexc_deg=15.0, max_oriexc_deg=180.0),
    '02_inclinar_dedos': dict(primary=(), min_primary_cm=0.0, max_off_cm=3.0,
                              min_oriexc_deg=10.0, max_oriexc_deg=180.0),
    '03_virar_palma': dict(primary=(), min_primary_cm=0.0, max_off_cm=3.0,
                           min_oriexc_deg=10.0, max_oriexc_deg=180.0),
    '04_frente_tras': dict(primary=(2,), min_primary_cm=5.0, max_off_cm=4.0,
                           min_oriexc_deg=0.0, max_oriexc_deg=12.0),
    '05_direita_esquerda': dict(primary=(1,), min_primary_cm=5.0, max_off_cm=4.0,
                                min_oriexc_deg=0.0, max_oriexc_deg=12.0),
    '05_direita_esquerda_2': dict(primary=(1,), min_primary_cm=5.0, max_off_cm=4.0,
                                  min_oriexc_deg=0.0, max_oriexc_deg=12.0),
    '06_cima_baixo': dict(primary=(0,), min_primary_cm=5.0, max_off_cm=4.0,
                          min_oriexc_deg=0.0, max_oriexc_deg=12.0),
    '07_abrir_fechar': dict(primary=(), min_primary_cm=0.0, max_off_cm=3.0,
                            min_oriexc_deg=0.0, max_oriexc_deg=8.0,
                            gripper=True),
    '08_cruz': dict(primary=(0, 1), min_primary_cm=5.0, max_off_cm=4.0,
                    min_oriexc_deg=0.0, max_oriexc_deg=15.0),
    # Partial exercises: the sim itself records small/interrupted motion.
    '09_gesto_em_movimento': dict(primary=(), min_primary_cm=0.0, max_off_cm=6.0,
                                  min_oriexc_deg=0.0, max_oriexc_deg=15.0),
    '10_perda_retorno': dict(primary=(0, 1, 2), min_primary_cm=3.0, max_off_cm=45.0,
                             min_oriexc_deg=0.0, max_oriexc_deg=15.0),
    '11_distancias': dict(primary=(0, 1, 2), min_primary_cm=3.0, max_off_cm=45.0,
                          min_oriexc_deg=0.0, max_oriexc_deg=15.0),
}

# Recognition-pipeline health bounds for re-inference on bag images.
MIN_POSITION_VALID_FRACTION = 0.50   # new inference vs bag duration
MAX_SAMPLE_GAP_S = 3.0               # longest acceptable silence mid-take (teleop recovers < 5 s)
MAX_MEDIAN_REINFER_DRIFT_CM = 12.0   # windowed new-p vs recorded-p (worst legit: 11.5 on 2 m motion)


def rebase_stamp(stamp_ns, first_ns, epoch_ns):
    """Map a recorded stamp onto the live wall clock."""
    return int(epoch_ns) + (int(stamp_ns) - int(first_ns))


def rebase_packet(packet, first_ns, epoch_ns):
    """Copy of a teleop_sample dict with stamps rebased to wall time."""
    out = dict(packet)
    for key in ('stamp_ns', 'orientation_stamp_ns', 'roll_stamp_ns'):
        if out.get(key) is not None:
            out[key] = rebase_stamp(out[key], first_ns, epoch_ns)
    return out


def window_decision(stamp_ns, start_ns, start_s=0.0, end_s=None):
    """Classify one receipt stamp as 'skip', 'deliver' or 'end'."""
    start_limit = int(start_ns) + round(float(start_s) * 1e9)
    if int(stamp_ns) < start_limit:
        return 'skip'
    if end_s is not None:
        end_limit = int(start_ns) + round(float(end_s) * 1e9)
        if int(stamp_ns) > end_limit:
            return 'end'
    return 'deliver'


def validate_window(start_s, end_s, duration):
    """Validate a [start_s, end_s] playback window inside duration."""
    if isinstance(start_s, bool) or isinstance(end_s, bool):
        raise ValueError('start_s and end_s must be finite numbers')
    try:
        start = float(start_s)
        end = float(end_s)
    except (TypeError, ValueError) as error:
        raise ValueError('start_s and end_s must be finite numbers') from error
    if not (math.isfinite(start) and math.isfinite(end)):
        raise ValueError('start_s and end_s must be finite numbers')
    duration = float(duration)
    if end <= duration + 0.0005 + 1e-9:
        end = min(end, duration)
    if not (0.0 <= start < end <= duration):
        raise ValueError('Require 0 <= start_s < end_s <= duration')
    return start, end


def quat_angle_deg(q0, q1):
    """Angle in degrees between two quaternions (x, y, z, w)."""
    dot = abs(sum(a * b for a, b in zip(q0, q1)))
    dot = min(1.0, max(0.0, dot))
    return math.degrees(2.0 * math.acos(dot))


def tcp_metrics(series):
    """series: list of dicts with t, p=[x,y,z] meters, q=[x,y,z,w].

    Returns span_cm (peak-to-peak per axis), orientation excursion from
    the first sample (max deg), and travel_m (path length).
    """
    if not series:
        raise ValueError('empty TCP series')
    xs = [s['p'][0] for s in series]
    ys = [s['p'][1] for s in series]
    zs = [s['p'][2] for s in series]
    span_cm = [100.0 * (max(v) - min(v)) for v in (xs, ys, zs)]
    q0 = series[0]['q']
    oriexc = max(quat_angle_deg(q0, s['q']) for s in series)
    travel = 0.0
    for a, b in zip(series, series[1:]):
        travel += math.dist(a['p'], b['p'])
    return dict(span_cm=span_cm, oriexc_deg=oriexc, travel_m=travel,
                n=len(series), duration_s=series[-1]['t'] - series[0]['t'])


def sample_flow_metrics(stamps, take_span_s):
    """stamps: sorted sample times (s). Returns count, rate, max gap."""
    if not stamps:
        return dict(count=0, rate_hz=0.0, max_gap_s=take_span_s)
    gaps = [b - a for a, b in zip(stamps, stamps[1:])]
    span = max(take_span_s, stamps[-1] - stamps[0] if len(stamps) > 1 else 0.0)
    return dict(count=len(stamps),
                rate_hz=len(stamps) / span if span > 0 else 0.0,
                max_gap_s=max(gaps) if gaps else 0.0)


def compare_take(stage, tcp_series, flow, gripper_widths=None):
    """Compare one robot take against the stage spec.

    tcp_series: list of dicts t/p/q (fr3_link0, meters).
    flow: dict from sample_flow_metrics for the NEW inference samples.
    gripper_widths: optional list of gripper widths (m) for stage 07.
    Returns dict with pass/fail, per-check rows and measured metrics.
    """
    if stage not in STAGE_SPEC:
        raise ValueError('unknown stage: ' + stage)
    spec = STAGE_SPEC[stage]
    m = tcp_metrics(tcp_series)
    checks = []

    def check(name, ok, detail):
        checks.append(dict(name=name, ok=bool(ok), detail=detail))

    check('samples_flowing', flow['count'] >= 10,
          'n=%d rate=%.1fHz max_gap=%.2fs' % (flow['count'], flow['rate_hz'], flow['max_gap_s']))
    if flow['count'] >= 10:
        check('no_long_gap', flow['max_gap_s'] <= MAX_SAMPLE_GAP_S,
              'max_gap=%.2fs limit=%.1fs' % (flow['max_gap_s'], MAX_SAMPLE_GAP_S))
    for axis in spec['primary']:
        nm = 'primary_axis_%s' % 'xyz'[axis]
        check(nm, m['span_cm'][axis] >= spec['min_primary_cm'],
              'span_%s=%.1fcm min=%.1fcm' % ('xyz'[axis], m['span_cm'][axis], spec['min_primary_cm']))
    off = [i for i in (0, 1, 2) if i not in spec['primary']]
    offmax = max((m['span_cm'][i] for i in off), default=0.0)
    check('isolation', offmax <= spec['max_off_cm'],
          'offmax=%.1fcm limit=%.1fcm spans=%s' % (offmax, spec['max_off_cm'],
                                                  [round(v, 1) for v in m['span_cm']]))
    check('orientation_min', m['oriexc_deg'] >= spec['min_oriexc_deg'],
          'exc=%.1fdeg min=%.1fdeg' % (m['oriexc_deg'], spec['min_oriexc_deg']))
    check('orientation_max', m['oriexc_deg'] <= spec['max_oriexc_deg'],
          'exc=%.1fdeg max=%.1fdeg' % (m['oriexc_deg'], spec['max_oriexc_deg']))
    if spec.get('gripper'):
        moved = False
        detail = 'no gripper data'
        if gripper_widths:
            span_mm = 1000.0 * (max(gripper_widths) - min(gripper_widths))
            moved = span_mm >= 5.0
            detail = 'width_span=%.1fmm' % span_mm
        check('gripper_moves', moved, detail)
    ok = all(c['ok'] for c in checks)
    return dict(stage=stage, profile=STAGES[stage], ok=ok, checks=checks,
                span_cm=m['span_cm'], oriexc_deg=m['oriexc_deg'],
                travel_m=m['travel_m'], flow=flow)


def check_line(result):
    """One-line CHECK summary for a compare_take result."""
    fails = [c['name'] for c in result['checks'] if not c['ok']]
    status = 'CHECK %s %s' % (result['stage'], 'PASS' if result['ok'] else 'FAIL:' + ','.join(fails))
    return '%s span=%scm oriexc=%.1fdeg travel=%.2fm n=%d' % (
        status, [round(v, 1) for v in result['span_cm']], result['oriexc_deg'],
        result['travel_m'], result['flow']['count'])


def _mean_p(series):
    n = len(series)
    return [sum(s['p'][i] for s in series) / n for i in range(3)]


def _match_pairs(recorded, fresh):
    """Match recorded samples to nearest fresh samples within 0.25 s."""
    if not recorded or not fresh:
        return []
    fresh_sorted = sorted(fresh, key=lambda s: s['offset_s'])
    pairs = []
    for r in recorded:
        best = min(fresh_sorted, key=lambda f: abs(f['offset_s'] - r['offset_s']))
        if abs(best['offset_s'] - r['offset_s']) <= 0.25:
            pairs.append((r['offset_s'], r, best))
    return pairs


def _median(values):
    values = sorted(values)
    mid = len(values) // 2
    return values[mid] if len(values) % 2 else 0.5 * (values[mid - 1] + values[mid])


def reinfer_drift(recorded, fresh, window_s=10.0):
    """Windowed de-meaned position drift (cm) between recorded samples and
    fresh re-inference on the same bag. Both are lists of dicts with
    offset_s (s since take start) and p=[x,y,z]. Returns the worst window
    median and the pair count.

    De-meaning per window (not globally) is required: teleop anchors on
    Start and only relative hand motion commands the robot, while the two
    tracker versions differ by a ~2 m absolute offset that itself drifts
    ~10 cm over a take (the current tracker outputs ~2x the rate with
    different filter lag). Global de-meaning fails takes whose motion
    shape agrees (measured: 14 cm global vs <=7.5 cm windowed).
    """
    pairs = _match_pairs(recorded, fresh)
    if not pairs:
        return float('nan'), 0
    t0 = min(t for t, _, _ in pairs)
    windows = {}
    for t, r, f in pairs:
        windows.setdefault(int((t - t0) // window_s), []).append((r, f))
    medians = []
    for group in windows.values():
        if len(group) < 3:
            continue
        mean_r = _mean_p([r for r, _ in group])
        mean_f = _mean_p([f for _, f in group])
        dists = [100.0 * math.dist([x - m for x, m in zip(r['p'], mean_r)],
                                   [x - m for x, m in zip(f['p'], mean_f)])
                 for r, f in group]
        medians.append(_median(dists))
    if not medians:
        return float('nan'), len(pairs)
    return max(medians), len(pairs)


def _qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz]


def _qinv(q):
    return [-q[0], -q[1], -q[2], q[3]]


def reinfer_orient_drift(recorded, fresh):
    """Median relative-rotation drift (deg) between recorded and fresh
    orientation trajectories. Both are lists of dicts with offset_s and
    q=[x,y,z,w]. Matches by nearest offset within 0.25 s, then compares
    rotations relative to a searched anchor (absolute palm frames differ
    between tracker versions, and single anchors can hit orientation
    outliers). Returns (median_deg, n_pairs)."""
    have_r = [s for s in recorded if isinstance(s.get('q'), list)]
    have_f = [s for s in fresh if isinstance(s.get('q'), list)]
    if not have_r or not have_f:
        return float('nan'), 0
    pairs = _match_pairs(have_r, have_f)
    if not pairs:
        return float('nan'), 0
    # The inter-version palm-frame offset is an unknown constant rotation,
    # and any single anchor pair can be an orientation outlier (measured:
    # 6/61 pairs off by >20 deg on stage 01). Search candidate anchors for
    # the best median agreement; the median still requires a majority of
    # pairs to agree, so outliers cannot fake a pass.
    pairs.sort(key=lambda p: p[0])
    cands = pairs[::3] + [pairs[-1]]
    best = float('inf')
    for _, ar, af in cands:
        q0r, q0f = ar['q'], af['q']
        angs = [quat_angle_deg(_qmul(r['q'], _qinv(q0r)), _qmul(f['q'], _qinv(q0f)))
                for _, r, f in pairs]
        best = min(best, _median(angs))
    return best, len(pairs)


# Recognition metric per exercise profile: rotation stages command the
# robot from palm orientation (hand translation is ignored), translation
# stages from wrist position, gripper from gestures.
RECOGNITION_METRIC = {
    'neutral': 'position', 'height': 'position', 'horizontal': 'position',
    'reach': 'position', 'cross': 'position', 'translation': 'position',
    'roll': 'orientation', 'pitch': 'orientation', 'yaw': 'orientation',
    'gripper': 'gesture', 'full_6d': 'position',
}
MAX_ORIENT_DRIFT_DEG = 15.0
MAX_WRIST_JUMP_M = 0.30  # teleport between consecutive valid samples = mistrack


def wrist_jump_metrics(samples):
    """Mistrack census on valid new-inference samples (dicts with wall_t, p).

    A real protocol hand never teleports 30 cm between ~0.3 s samples; the
    October reference itself peaks at 0.23 m on the fastest stage, while the
    current tracker throws 0.46-0.80 m lateral jumps the median-based drift
    metric absorbs silently. Returns max jump and count over the gate.
    """
    pts = sorted(((s['wall_t'], s['p']) for s in samples), key=lambda x: x[0])
    jumps = [math.dist(a[1], b[1]) for a, b in zip(pts, pts[1:])]
    big = [j for j in jumps if j > MAX_WRIST_JUMP_M]
    return dict(max_jump_m=round(max(jumps), 3) if jumps else 0.0,
                n_big_jumps=len(big))


def recognition_verdict(stage, new, recorded, flow):
    """Single definition of the dry (recognition-only) verdict.

    new/recorded: parse_samples outputs; flow: sample_flow_metrics over new.
    ok requires sample flow, the per-profile metric match AND zero wrist
    teleports (a median drift can pass while the tracker visibly loses the
    wrist, as exec-04 showed: drift ok, 0.46 m jumps, teleop paused ~50%).
    """
    metric = RECOGNITION_METRIC[STAGES[stage]]
    drift_cm, n_pairs = reinfer_drift(recorded, new)
    orient_drift, n_opairs = reinfer_orient_drift(recorded, new)
    gestures = sorted({s['gesture'] for s in new if s.get('gesture')})
    jumps = wrist_jump_metrics(new)
    flow_ok = len(new) >= 10 and flow['max_gap_s'] <= MAX_SAMPLE_GAP_S
    if metric == 'position':
        metric_ok = drift_cm == drift_cm and drift_cm <= MAX_MEDIAN_REINFER_DRIFT_CM
    elif metric == 'orientation':
        metric_ok = orient_drift == orient_drift and orient_drift <= MAX_ORIENT_DRIFT_DEG
    else:  # gripper: both gestures must be recognized at least once
        metric_ok = 'Open_Palm' in gestures and 'Closed_Fist' in gestures
    jumps_ok = jumps['n_big_jumps'] == 0
    ok = flow_ok and metric_ok and jumps_ok
    return dict(metric=metric, flow_ok=bool(flow_ok), metric_ok=bool(metric_ok),
                jumps_ok=bool(jumps_ok), ok=bool(ok),
                drift_cm=None if drift_cm != drift_cm else round(drift_cm, 2),
                orient_drift_deg=None if orient_drift != orient_drift else round(orient_drift, 1),
                pairs=[n_pairs, n_opairs], gestures=gestures,
                max_jump_m=jumps['max_jump_m'], n_big_jumps=jumps['n_big_jumps'])


def load_sim_review(path):
    """Load simulation_review.json into {stage: run dict}."""
    with open(path) as f:
        review = json.load(f)
    return {r['stage']: r for r in review.get('runs', [])}
