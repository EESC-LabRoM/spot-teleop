"""Offline comparison only; output is not a deployable collision model."""
import json
from pathlib import Path
import numpy as np

root = Path(__file__).parent / 'fit_inputs'
runs = ('20261121_050322_drill_oneshot_regen', '20261121_051628_drill_loop_regen', '20261121_052029_drill_loop_regen')
rng = np.random.default_rng(5)
for run in runs:
    folder = root / 'grasp_loop/runs' / run
    points = np.load(folder / 'cloud.npy')[:, :3]
    plan = json.loads((folder / 'plan.json').read_text())
    rotation = np.asarray(plan['hand_R'])
    origin = np.asarray(plan['grasp_pos'])
    hand = (points - origin) @ rotation
    model = json.loads((root / 'regrasp/state' / ('held_model_' + run + '.json')).read_text())
    old = np.asarray(model['spheres'])
    old_d = np.linalg.norm(hand[:, None] - old[None, :, :3], axis=2)
    best = None
    for _ in range(40):
        centers = hand[rng.choice(len(hand), 4, replace=False)].copy()
        for __ in range(30):
            distances = np.linalg.norm(hand[:, None] - centers[None, :, :], axis=2)
            nearest = distances.argmin(axis=1)
            updated = np.array([hand[nearest == j].mean(axis=0) if (nearest == j).any() else centers[j] for j in range(4)])
            if np.max(abs(updated - centers)) < 1e-5:
                break
            centers = updated
        distances = np.linalg.norm(hand[:, None] - centers[None, :, :], axis=2)
        nearest = distances.argmin(axis=1)
        radii = np.array([np.percentile(distances[nearest == j, j], 99.5) + .006 if (nearest == j).sum() > 5 else .005 for j in range(4)])
        coverage = (distances <= radii).any(axis=1).mean()
        key = (coverage >= .98, -float(np.sum(radii ** 3)))
        if best is None or key > best[0]:
            best = (key, centers.copy(), radii.copy(), coverage)
    _, centers, radii, coverage = best
    constrained = centers.copy()
    zfar = float(np.percentile(hand[:, 2], 99.5))
    constrained[:, 2] = np.minimum(constrained[:, 2], zfar + .015 - radii)
    constrained_coverage = float((np.linalg.norm(hand[:, None] - constrained[None, :, :], axis=2) <= radii).any(axis=1).mean())
    lift_z = origin + np.array([0, 0, .10])
    low = float(((centers @ rotation.T + lift_z)[:, 2] - radii).min())
    actual_low = float((hand @ rotation.T + lift_z)[:, 2].min())
    print(json.dumps(dict(run=run, points=len(hand), old_r=old[:,3].round(3).tolist(), cap_006_coverage=round(float((old_d <= np.minimum(old[:,3],.06)).any(axis=1).mean()),4), fit_r=radii.round(3).tolist(), fit_coverage=round(float(coverage),4), constrained_coverage=round(constrained_coverage,4), volume_ratio=round(float(np.sum(radii**3)/np.sum(old[:,3]**3)),3), lift_sphere_bottom=round(low,3), observed_bottom=round(actual_low,3))))
