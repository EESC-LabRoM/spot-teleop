import time
import unittest

import numpy as np

import selector_service as S

GS = ('data: \'{"enabled": true, "phase": "idle", "held": false, "busy": false}\'\n---\n'
      'data: false\n---\n')


class Pure(unittest.TestCase):
    def test_bbox(self):
        self.assertEqual(S.click_to_bbox_1000([0.5, 0.5], 40), [460, 460, 540, 540])
        self.assertEqual(S.click_to_bbox_1000([0.0, 1.0], 40), [0, 960, 40, 1000])
        for bad in ([1.2, 0.5], [float('nan'), 0.2], ['a', 0]):
            with self.assertRaises((ValueError, TypeError)):
                S.click_to_bbox_1000(bad, 40)

    def test_teleop_ok(self):
        ok, why, f = S.parse_teleop(GS, time.time())
        self.assertTrue(ok, why)
        self.assertFalse(f['engaged'])

    def test_teleop_refusals(self):
        self.assertFalse(S.parse_teleop(GS.replace('data: false', 'data: true'), 0)[0])
        self.assertFalse(S.parse_teleop(GS.replace('"held": false', '"held": true'), 0)[0])
        self.assertFalse(S.parse_teleop(GS.replace('"busy": false', '"busy": true'), 0)[0])
        self.assertFalse(S.parse_teleop('', 0)[0])
        self.assertFalse(S.parse_teleop(GS.split('---')[0] + '---', 0)[0])
        self.assertFalse(S.parse_teleop(GS.replace('"held": false', '"held": "no"'), 0)[0])

    def test_mask_check(self):
        m = np.zeros((360, 640), np.uint8)
        m[320:350, 268:298] = 255  # ~0.4% ball-sized blob
        spec = S.TARGETS['tennis_ball']
        click = [283 / 639, 334 / 359]
        self.assertTrue(S.mask_check(m, click, spec, 1.5, 0.2)[0])
        self.assertFalse(S.mask_check(m, [0.1, 0.1], spec, 1.5, 0.2)[0])     # other object
        self.assertFalse(S.mask_check(m, click, spec, 1.5, 3.0)[0])          # stale
        self.assertFalse(S.mask_check(None, click, spec, 1.5, 0.2)[0])
        big = np.full((360, 640), 255, np.uint8)
        self.assertFalse(S.mask_check(big, click, spec, 1.5, 0.2)[0])        # oversized

    def test_cloud_check(self):
        rng = np.random.default_rng(0)
        ball = rng.normal(0, 0.015, (500, 3)) + [0.4, -0.27, 0.07]
        self.assertTrue(S.cloud_check(ball, S.TARGETS['tennis_ball'])[0])
        self.assertFalse(S.cloud_check(ball * [10, 1, 1], S.TARGETS['tennis_ball'])[0])
        self.assertFalse(S.cloud_check(ball[:20], S.TARGETS['tennis_ball'])[0])

    def test_map_carved(self):
        rng = np.random.default_rng(1)
        table = np.c_[rng.uniform(0, 1, (5000, 2)), np.full(5000, 0.029)]
        ball = rng.normal(0, 0.015, (400, 3)) + [0.4, 0.3, 0.08]
        self.assertTrue(S.map_carved(table, ball)[0])
        self.assertFalse(S.map_carved(np.vstack([table, ball]), ball)[0])
        self.assertFalse(S.map_carved(table[:10], ball)[0])

    def test_esdf_carved(self):
        self.assertTrue(S.esdf_carved(np.full(50, -1000.0))[0])      # carved -> unobserved
        self.assertTrue(S.esdf_carved(np.full(50, 0.05))[0])         # clearly free
        self.assertFalse(S.esdf_carved(np.full(50, 0.01))[0])        # object still mapped
        self.assertFalse(S.esdf_carved(np.full(5, np.nan))[0])

    def test_y_policy(self):
        self.assertEqual(S.Y_VALIDATED, {'drill', 'sprite'})
        self.assertIn('tennis_ball', S.Y_LOCK_REASON)
        self.assertNotIn('tennis_ball', S.Y_VALIDATED)


if __name__ == '__main__':
    unittest.main()
