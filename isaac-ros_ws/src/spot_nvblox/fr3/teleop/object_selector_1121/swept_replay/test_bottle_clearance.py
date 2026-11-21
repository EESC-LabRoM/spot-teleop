"""Offline recorded drill route against the Sprite silhouette keep-out AABB."""
import json
from pathlib import Path
import unittest
import numpy as np
from fr3_spheres import FR3Spheres
from replay_plan import Q_HOME, STAGES, CARRIED, sample_stage

HERE = Path(__file__).resolve().parent
REF = HERE / 'ref' / '20261121_053830_drill_loop_regen'
SPRITE_CENTER = np.array([0.7285, -0.2345, 0.1577])
SPRITE_HALF = np.array([0.120, 0.204, 0.261]) / 2


def sphere_aabb_clearance(xyz, radius, center=SPRITE_CENTER, half=SPRITE_HALF):
    delta = np.maximum(np.abs(np.asarray(xyz) - center) - half, 0.0)
    return np.linalg.norm(delta, axis=-1) - np.asarray(radius)


def route_clearance():
    plan = json.loads((REF / 'plan.json').read_text())
    held = np.asarray(json.loads((REF / 'held.json').read_text())['spheres'], float)
    kin = FR3Spheres()
    prev = np.asarray(plan.get('start_q') or Q_HOME, float)
    stage_min = {}
    for stage in STAGES:
        qs, prev = sample_stage(prev, plan['stages'][stage]['q'], 0.01)
        model = held if stage in CARRIED else None
        values = []
        for q in qs:
            spheres, _ = kin.spheres(q, model)
            values.extend(sphere_aabb_clearance(spheres[:, :3], spheres[:, 3]))
        stage_min[stage] = float(np.min(values))
    return stage_min


class SpriteKeepoutRecordedDrillRouteTest(unittest.TestCase):
    def test_recorded_drill_route_clears_sprite_keepout(self):
        result = route_clearance()
        # Conservative static box clearance, not a live ESDF/unknown-space proof.
        self.assertGreaterEqual(min(result.values()), 0.15, result)
        self.assertEqual(set(result), set(STAGES))

    def test_sphere_aabb_distance(self):
        self.assertAlmostEqual(float(sphere_aabb_clearance([[1, 0, 0]], [0.1])[0]),
                               float(np.linalg.norm(np.maximum(np.abs(np.array([1., 0., 0.]) - SPRITE_CENTER) - SPRITE_HALF, 0)) - 0.1))
        self.assertAlmostEqual(float(sphere_aabb_clearance([[0.7285, -0.2345, 0.1577]], [0.1])[0]), -0.1)


if __name__ == '__main__':
    unittest.main()
