import unittest
from dataclasses import replace
from pathlib import Path

import cv2
import numpy as np

from selector_backend import (BindingRecord, Identity, MapRecord, MaskRecord,
                              ObjectSelector, SPECS, SelectionRefused, Target,
                              TargetSpec, load_binding, load_held_model)

HERE = Path(__file__).resolve().parent.parent
RUNS = HERE / 'fit_inputs/grasp_loop/runs'
HELD = HERE / 'fit_inputs/regrasp/state'
DRILL_RUN = '20261121_050322_drill_oneshot_regen'
BALL_MASK = cv2.imread(str(HERE / 'tennis_ball_mask.png'), 0) > 0
SPRITE_MASK = cv2.imread(str(HERE / 'sprite_mask.png'), 0) > 0
BALL_BOX = (0.416, 0.875, 0.473, 0.986)
SPRITE_BOX = (0.578, 0.622, 0.649, 0.847)
H, W = BALL_MASK.shape


def drill_mask():
    m = np.zeros((H, W), bool)
    m[150:260, 100:220] = True          # synthetic, disjoint from ball/sprite
    return m, (100 / W, 150 / H, 220 / W, 260 / H)


class Clock:
    t = 1000.0
    def __call__(self): return self.t


class Base(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.sel = ObjectSelector(self.clock)

    def full_drill(self):
        ident = self.sel.select('drill', engaged=False, busy=False, held=False)
        self.clock.t += 0.5
        m, box = drill_mask()
        self.sel.submit(MaskRecord(ident, self.clock.t, m, box, True,
                                   {'tennis_ball': BALL_MASK, 'sprite': SPRITE_MASK}))
        self.sel.submit(MapRecord(ident, self.clock.t, frozenset({'drill'}),
                                  frozenset({'tennis_ball', 'sprite'}), True, True))
        self.sel.submit(load_binding(RUNS / DRILL_RUN, ident, stamp=self.clock.t,
                                     broker_pass=True, broker_run=DRILL_RUN, broker_stamp=self.clock.t))
        self.sel.submit(load_held_model(HELD / f'held_model_{DRILL_RUN}.json', ident))
        return ident


class ReadyPath(Base):
    def test_real_drill_artifacts_reach_ready(self):
        ident = self.full_drill()
        self.assertTrue(self.sel.evaluate(), self.sel.reason)
        self.assertEqual(self.sel.authorize_y(ident), (True, 'Ready'))
        api = self.sel.to_api()
        self.assertEqual((api['selected'], api['status'], api['ready']), ('drill', 'ready', True))
        for k in ('selected', 'generation', 'status', 'reason', 'mask', 'map', 'grasp'):
            self.assertIn(k, api)


class Selection(Base):
    def test_unknown_target_refused(self):
        with self.assertRaises(SelectionRefused):
            self.sel.select('screwdriver', engaged=False, busy=False, held=False)
        self.assertIsNone(self.sel.selected)

    def test_switch_refused_while_engaged_busy_or_held(self):
        self.sel.select('drill', engaged=False, busy=False, held=False)
        for flags in ((True, False, False), (False, True, False), (False, False, True)):
            with self.assertRaises(SelectionRefused):
                self.sel.select('sprite', engaged=flags[0], busy=flags[1], held=flags[2])
        self.assertEqual(self.sel.selected.target, 'drill')

    def test_reselect_clears_everything_and_old_y_refused(self):
        old = self.full_drill()
        self.assertTrue(self.sel.evaluate())
        self.sel.select('drill', engaged=False, busy=False, held=False)   # same object, new gen
        self.assertFalse(self.sel.evaluate())
        self.assertEqual(self.sel.authorize_y(old)[0], False)
        self.assertIsNone(self.sel.binding)

    def test_stale_generation_records_rejected(self):
        old = self.full_drill()
        self.sel.select('tennis_ball', engaged=False, busy=False, held=False)
        with self.assertRaises(SelectionRefused):
            self.sel.submit(load_binding(RUNS / DRILL_RUN, old, stamp=self.clock.t,
                                         broker_pass=True, broker_run=DRILL_RUN, broker_stamp=self.clock.t))

    def test_forged_identity_rejected(self):
        ident = self.sel.select('tennis_ball', engaged=False, busy=False, held=False)
        for bad in (replace(ident, target='drill'), replace(ident, selection_id='x'),
                    replace(ident, generation=0)):
            with self.assertRaises(SelectionRefused):
                self.sel.submit(MaskRecord(bad, self.clock.t, BALL_MASK, BALL_BOX, True))

    def test_y_with_none_identity_refused(self):
        self.full_drill()
        self.assertFalse(self.sel.authorize_y(None)[0])


class MaskGuards(Base):
    def setUp(self):
        super().setUp()
        self.ident = self.sel.select('tennis_ball', engaged=False, busy=False, held=False)

    def put(self, **kw):
        base = dict(identity=self.ident, stamp=self.clock.t, mask=BALL_MASK, box_norm=BALL_BOX,
                    tracker_ok=True, other_masks={'sprite': SPRITE_MASK})
        base.update(kw)
        self.sel.submit(MaskRecord(**base))
        self.sel.evaluate()
        return self.sel.stage_reasons['mask']

    def test_real_ball_mask_passes_mask_stage(self):
        self.assertIsNone(self.put())

    def test_ball_mask_on_sprite_location_rejected(self):
        # mask from the tracker landed on the bottle while prompt box is the ball's
        self.assertIn('outside its prompt box', self.put(mask=SPRITE_MASK))

    def test_overlap_with_other_object_rejected(self):
        self.assertIn('overlaps', self.put(other_masks={'sprite': BALL_MASK}))

    def test_stale_lost_empty_and_huge_masks_rejected(self):
        self.clock.t += 0
        self.assertIsNotNone(self.put(stamp=self.clock.t - 1.6))
        self.assertIsNotNone(self.put(tracker_ok=False))
        self.assertIsNotNone(self.put(mask=np.zeros_like(BALL_MASK)))
        self.assertIsNotNone(self.put(mask=np.ones_like(BALL_MASK)))
        self.assertIsNotNone(self.put(stamp=self.clock.t + 1.0))   # future stamp

    def test_bad_box_and_self_in_others_rejected(self):
        self.assertIsNotNone(self.put(box_norm=(0.5, 0.5, 0.4, 0.6)))
        self.assertIsNotNone(self.put(box_norm=(0, 0, float('nan'), 1)))
        self.assertIsNotNone(self.put(other_masks={'tennis_ball': SPRITE_MASK}))

    def test_shape_mismatch_other_mask_is_conflict(self):
        self.assertIn('overlaps', self.put(other_masks={'sprite': np.zeros((10, 10), bool)}))


class MapGuards(Base):
    def test_map_must_carve_only_target_and_keep_others(self):
        ident = self.full_drill()
        good = self.sel.map
        cases = [
            replace(good, excluded_ids=frozenset({'drill', 'sprite'})),
            replace(good, excluded_ids=frozenset({'tennis_ball'})),
            replace(good, obstacle_ids=frozenset({'sprite'})),
            replace(good, obstacle_ids=frozenset({'sprite', 'tennis_ball', 'drill'})),
            replace(good, rebuilt_after_selection=False),
            replace(good, stamp=self.sel.selected_at - 1),
            replace(good, target_voxels_absent=False),
        ]
        for c in cases:
            self.sel.submit(c)
            self.assertFalse(self.sel.evaluate(), c)
            self.assertIsNotNone(self.sel.stage_reasons['map'])


class BindingGuards(Base):
    def test_drill_binding_never_used_for_ball_or_sprite(self):
        for tgt in ('tennis_ball', 'sprite'):
            ident = self.sel.select(tgt, engaged=False, busy=False, held=False)
            self.sel.submit(load_binding(RUNS / DRILL_RUN, ident, stamp=self.clock.t,
                                         broker_pass=True, broker_run=DRILL_RUN, broker_stamp=self.clock.t))
            self.sel.evaluate()
            self.assertIsNotNone(self.sel.stage_reasons['grasp'])

    def test_non_drill_refused_even_with_clean_name_until_planner_validated(self):
        ident = self.sel.select('tennis_ball', engaged=False, busy=False, held=False)
        b = load_binding(RUNS / DRILL_RUN, ident, stamp=self.clock.t,
                         broker_pass=True, broker_run=DRILL_RUN, broker_stamp=self.clock.t)
        self.sel.submit(replace(b, run='20261121_x_regen', broker_run='20261121_x_regen'))
        self.sel.evaluate()
        self.assertIn('not validated', self.sel.stage_reasons['grasp'])

    def test_drill_specific_plan_keys_refused_when_planner_flag_flipped(self):
        # Even if someone marks ball as validated, drill-planner output is refused.
        orig = SPECS[Target.TENNIS_BALL]
        SPECS[Target.TENNIS_BALL] = TargetSpec(orig.label, orig.mask_fraction,
                                               orig.cloud_extent_m, orig.min_cloud_points, True)
        try:
            ident = self.sel.select('tennis_ball', engaged=False, busy=False, held=False)
            b = load_binding(RUNS / DRILL_RUN, ident, stamp=self.clock.t,
                             broker_pass=True, broker_run=DRILL_RUN, broker_stamp=self.clock.t)
            plan = dict(b.plan, run='ball_run')
            self.sel.submit(replace(b, run='ball_run', broker_run='ball_run', plan=plan))
            self.sel.evaluate()
            self.assertIn('drill-specific', self.sel.stage_reasons['grasp'])
            # and a drill-sized cloud is implausible for a ball
            stripped = {st: {k: v for k, v in c.items() if k != 'arm_links_in_drill_box'}
                        for st, c in plan['checks'].items()}
            plan2 = dict(plan, checks=stripped,
                         ball_validation={'ok': True, 'run': 'ball_run'})
            self.sel.submit(replace(b, run='ball_run', broker_run='ball_run', plan=plan2))
            self.sel.evaluate()
            self.assertIn('extent', self.sel.stage_reasons['grasp'])
        finally:
            SPECS[Target.TENNIS_BALL] = orig

    def test_broker_plan_and_freshness_guards(self):
        self.full_drill()
        good = self.sel.binding
        cases = [
            replace(good, broker_pass=False),
            replace(good, broker_run='20261121_052029_drill_loop_regen'),
            replace(good, plan=dict(good.plan, run='other')),
            replace(good, plan=dict(good.plan, problems=['palm clearance'])),
            replace(good, stamp=self.sel.selected_at - 1),
            replace(good, cloud=good.cloud[:10]),
            replace(good, cloud=np.full((2000, 3), np.nan)),
        ]
        for c in cases:
            self.sel.submit(c)
            self.assertFalse(self.sel.evaluate())
            self.assertIsNotNone(self.sel.stage_reasons['grasp'])
        self.sel.submit(good)
        self.clock.t += 31
        self.assertFalse(self.sel.evaluate())   # binding (and mask) stale


class HeldGuards(Base):
    def test_held_model_must_match_binding_run(self):
        ident = self.full_drill()
        self.sel.submit(load_held_model(HELD / 'held_model_20261121_052029_drill_loop_regen.json', ident))
        self.assertFalse(self.sel.evaluate())
        self.assertIn('different binding', self.sel.stage_reasons['held'])

    def test_low_coverage_or_bad_sphere_refused(self):
        self.full_drill()
        good = self.sel.held
        for c in (replace(good, coverage=0.97), replace(good, spheres=((0, 0, 0, -1),)),
                  replace(good, frame='world'), replace(good, spheres=())):
            self.sel.submit(c)
            self.assertFalse(self.sel.evaluate())


class FailClosed(Base):
    def test_check_exception_fails_closed(self):
        self.full_drill()
        self.sel.submit(replace(self.sel.binding, plan=None))
        self.assertFalse(self.sel.evaluate())
        self.assertIn('error', self.sel.stage_reasons['grasp'])

    def test_api_before_selection(self):
        api = self.sel.to_api()
        self.assertEqual((api['selected'], api['status'], api['ready']), (None, 'idle', False))


class ContinuousFreshness(Base):
    """Y needs map/broker to stay fresh, not merely post-date selection."""
    def test_map_future_and_max_age(self):
        self.full_drill()
        good = self.sel.map
        self.sel.submit(replace(good, stamp=self.clock.t + 1.0))
        self.assertFalse(self.sel.evaluate())
        self.assertIn('future', self.sel.stage_reasons['map'])
        self.sel.submit(good)
        self.assertTrue(self.sel.evaluate(), self.sel.reason)
        self.clock.t += 5.1
        self.sel.evaluate()
        self.assertIn('stale', self.sel.stage_reasons['map'])

    def test_binding_and_broker_future_refused(self):
        self.full_drill()
        good = self.sel.binding
        for c, word in ((replace(good, stamp=self.clock.t + 1.0), 'future'),
                        (replace(good, broker_stamp=self.clock.t + 1.0), 'future'),
                        (replace(good, broker_stamp=good.stamp - 0.5), 'predates')):
            self.sel.submit(c)
            self.assertFalse(self.sel.evaluate())
            self.assertIn(word, self.sel.stage_reasons['grasp'])

    def test_y_refused_once_broker_ages_even_if_mask_and_map_refreshed(self):
        ident = self.full_drill()
        self.assertTrue(self.sel.authorize_y(ident)[0])
        self.clock.t += 2.5
        self.sel.submit(replace(self.sel.mask, stamp=self.clock.t))
        self.sel.submit(replace(self.sel.map, stamp=self.clock.t))
        ok, why = self.sel.authorize_y(ident)
        self.assertFalse(ok)
        self.assertIn('Broker preflight stale', why)
        self.sel.submit(replace(self.sel.binding, broker_stamp=self.clock.t))
        self.assertTrue(self.sel.authorize_y(ident)[0])

    def test_shipped_planner_flags_only_drill(self):
        self.assertEqual({t.value for t, s in SPECS.items() if s.planner_validated}, {'drill'})


class StageGate(Base):
    """problems == [] is not avoidance evidence; every stage re-checked."""
    def test_all_real_drill_plans_pass_stage_gate(self):
        from selector_backend import check_stages
        import json
        for f in sorted(RUNS.glob('*/plan.json')):
            self.assertIsNone(check_stages(json.loads(f.read_text())), f)

    def test_low_or_missing_s2_to_s5_values_refused_despite_empty_problems(self):
        self.full_drill()
        good = self.sel.binding
        for st in ('S2_pre_to_grasp', 'S3_lift', 'S4_put_back', 'S5_grasp_to_pre'):
            for patch in ({'min_obstacle_m': 0.05}, {'min_table_clear_m': 0.02},
                          {'min_obstacle_m': None}, {'joint_limit_margin_rad': 0.01},
                          {'min_obstacle_m': float('nan')}, {'arm_links_in_drill_box': True}):
                checks = {k: dict(v) for k, v in good.plan['checks'].items()}
                checks[st].update(patch)
                self.sel.submit(replace(good, plan=dict(good.plan, checks=checks, problems=[])))
                self.assertFalse(self.sel.evaluate(), (st, patch))
                self.assertIn(st, self.sel.stage_reasons['grasp'])
            checks = {k: v for k, v in good.plan['checks'].items() if k != st}
            self.sel.submit(replace(good, plan=dict(good.plan, checks=checks)))
            self.assertFalse(self.sel.evaluate())

    def test_known_gaps_always_reported_even_when_ready(self):
        self.full_drill()
        self.assertTrue(self.sel.evaluate())
        gaps = self.sel.to_api()['known_gaps']
        self.assertTrue(any('Hand/fingers' in g for g in gaps))
        self.assertTrue(any('direct JTC' in g for g in gaps))

    def test_ball_requires_ball_validation_record_when_flag_flipped(self):
        orig = SPECS[Target.TENNIS_BALL]
        SPECS[Target.TENNIS_BALL] = TargetSpec(orig.label, orig.mask_fraction,
                                               orig.cloud_extent_m, orig.min_cloud_points, True)
        try:
            ident = self.sel.select('tennis_ball', engaged=False, busy=False, held=False)
            b = load_binding(RUNS / DRILL_RUN, ident, stamp=self.clock.t, broker_pass=True,
                             broker_run=DRILL_RUN, broker_stamp=self.clock.t)
            stripped = {st: {k: v for k, v in c.items() if k != 'arm_links_in_drill_box'}
                        for st, c in b.plan['checks'].items()}
            for bv in (None, {'ok': False, 'run': 'ball_run'}, {'ok': True, 'run': 'other'},
                       {'ok': 'yes', 'run': 'ball_run'}):
                plan = dict(b.plan, run='ball_run', checks=stripped)
                if bv is not None:
                    plan['ball_validation'] = bv
                self.sel.submit(replace(b, run='ball_run', broker_run='ball_run', plan=plan))
                self.sel.evaluate()
                self.assertIn('Ball-specific', self.sel.stage_reasons['grasp'])
        finally:
            SPECS[Target.TENNIS_BALL] = orig


if __name__ == '__main__':
    unittest.main()
