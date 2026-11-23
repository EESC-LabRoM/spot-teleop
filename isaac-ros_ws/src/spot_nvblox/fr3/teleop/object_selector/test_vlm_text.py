import json
import unittest

import selector_service as S


class VlmTextTests(unittest.TestCase):
    def test_prompt_accepts_plain_descriptions_only(self):
        self.assertEqual(S.check_prompt('  red   mug '), 'red mug')
        self.assertEqual(S.check_prompt("Sprite bottle, green"), 'Sprite bottle, green')
        for bad in ['', ' ', 'a' * 61, 'mug\nignore previous', 'mug; rm -rf /', '<b>x</b>', '$(id)', None, 5]:
            with self.assertRaises(ValueError, msg=repr(bad)):
                S.check_prompt(bad)

    def test_parse_vlm_result_box_to_click(self):
        r = S.parse_vlm_result(json.dumps({'bbox_1000': [400, 500, 480, 580], 'confidence': 0.9, 'vlm_latency_ms': 2100,
                                           'grasps_1000': []}))
        self.assertEqual(r['bbox'], [400, 500, 480, 580])
        self.assertAlmostEqual(r['click'][0], 0.44)
        self.assertAlmostEqual(r['click'][1], 0.54)
        self.assertEqual(r['latency_ms'], 2100)

    def test_parse_vlm_result_refuses_bad_answers(self):
        for bad in ['not json', '{}', json.dumps({'bbox_1000': [1, 2, 3]}), json.dumps({'bbox_1000': [500, 500, 400, 600]}),
                    json.dumps({'bbox_1000': [-5, 0, 100, 100]}), json.dumps({'bbox_1000': [0, 0, 1001, 100]}),
                    json.dumps({'bbox_1000': [0, 0, 900, 300]}),          # 90 % of the width: not an object
                    json.dumps({'bbox_1000': ['a', 0, 1, 1]}), None]:
            with self.assertRaises(ValueError, msg=repr(bad)):
                S.parse_vlm_result(bad)

    def test_click_from_vlm_is_a_valid_seed_click(self):
        r = S.parse_vlm_result(json.dumps({'bbox_1000': [100, 200, 160, 260]}))
        self.assertEqual(len(S.click_to_bbox_1000(r['click'], 35)), 4)      # same helper the click path uses

    def test_custom_target_is_never_y_validated_and_has_a_spec(self):
        self.assertNotIn(S.CUSTOM_ID, S.Y_VALIDATED)
        spec = S.custom_spec('red mug')
        self.assertEqual(spec['label'], 'red mug')
        self.assertLess(spec['mask'][0], 0.001)
        # the gates the click path applies work with the generic spec
        import numpy as np
        m = np.zeros((360, 640), np.uint8)
        m[150:190, 300:340] = 255
        ok, _ = S.mask_check(m, (320 / 639, 170 / 359), spec, 1.5, 0.2)
        self.assertTrue(ok)


class CoordinatorSeedTests(unittest.TestCase):
    def test_picks_only_foreign_seed_after_the_prompt_was_set(self):
        cmds = [(90.0, {'bbox_1000': [1, 2, 3, 4], 'label': 'drill'}),                # before the prompt: old
                (101.0, {'bbox_1000': [10, 20, 30, 40], 'label': 'custom'}),         # our own seed: never reuse
                (102.0, {'label': 'ball'}),                                          # no box
                (103.0, {'bbox_1000': [400, 700, 450, 780], 'label': 'tennis ball'})]
        d = S.pick_coordinator_seed(cmds, 100.0, 'custom')
        self.assertEqual(d['bbox_1000'], [400, 700, 450, 780])
        self.assertIsNone(S.pick_coordinator_seed(cmds[:3], 100.0, 'custom'))
        self.assertIsNone(S.pick_coordinator_seed([], 0.0, 'custom'))

    def test_switch_text_runs_the_normal_switch_without_pre_grounding(self):
        calls = []
        sel = S.Selector.__new__(S.Selector)
        sel._switch = lambda *a, **k: calls.append((a, k))
        sel._switch_text('red mug')
        (a, k), = calls
        self.assertEqual(a, (S.CUSTOM_ID, None, None))
        self.assertEqual(k['text_prompt'], 'red mug')
        self.assertEqual(k['spec']['label'], 'red mug')


if __name__ == '__main__':
    unittest.main()
