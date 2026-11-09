#!/usr/bin/env python3
"""Re-score a finished dry take with the current recognition metric.

Reads <take>/samples.jsonl (+ summary.json for the stage) and applies the
per-profile recognition check from bagtime. Used when the metric itself was
fixed after a session ran; the original verdict is kept in summary.json.
No ROS needed.

  python3 rescore.py takes/dry_session/01_giro_plano_imagem
"""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from bagtime import recognition_verdict, sample_flow_metrics  # noqa: E402
from run_take import parse_samples  # noqa: E402 (no ROS imports at module level)


def rescore(takedir):
    takedir = Path(takedir)
    summary = json.loads((takedir / 'summary.json').read_text())
    stage = summary['stage']
    new, recorded = parse_samples(takedir / 'samples.jsonl', summary.get('scored_from_wall', 0.0))
    take_span = max([s['wall_t'] for s in new], default=0) - min([s['wall_t'] for s in new], default=0)
    flow = sample_flow_metrics(sorted(s['wall_t'] for s in new),
                               take_span or summary.get('bag_duration_s', 0))
    rescored = recognition_verdict(stage, new, recorded, flow)
    rescored['flow'] = flow
    summary['rescored'] = rescored
    (takedir / 'summary.json').write_text(json.dumps(summary, indent=1))
    print('%s RECOGNITION %s [%s] drift=%s/%s pairs=%s jumps=%d(max %sm) gestures=%s' % (
        stage, 'PASS' if rescored['ok'] else 'FAIL', rescored['metric'], rescored['drift_cm'],
        rescored['orient_drift_deg'], rescored['pairs'], rescored['n_big_jumps'],
        rescored['max_jump_m'], rescored['gestures']))
    return rescored['ok']


if __name__ == '__main__':
    sys.exit(0 if rescore(sys.argv[1]) else 1)
