"""Unit tests for bagtime.py (stdlib only; runs on any machine)."""
import json
import math
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bagtime


def series(p_list, q=(0, 0, 0, 1), dt=0.1):
    return [dict(t=i * dt, p=list(p), q=list(q)) for i, p in enumerate(p_list)]


def rot_z(deg):
    r = math.radians(deg) / 2.0
    return [0, 0, math.sin(r), math.cos(r)]


class RebaseTest(unittest.TestCase):
    def test_rebase_stamp(self):
        self.assertEqual(bagtime.rebase_stamp(100, 100, 5000), 5000)
        self.assertEqual(bagtime.rebase_stamp(150, 100, 5000), 5050)

    def test_rebase_packet(self):
        pkt = dict(version=3, stamp_ns=100, orientation_stamp_ns=100,
                   roll_stamp_ns=None, p=[1, 2, 3])
        out = bagtime.rebase_packet(pkt, 100, 5000)
        self.assertEqual(out['stamp_ns'], 5000)
        self.assertEqual(out['orientation_stamp_ns'], 5000)
        self.assertIsNone(out['roll_stamp_ns'])
        self.assertEqual(out['p'], [1, 2, 3])
        self.assertEqual(pkt['stamp_ns'], 100)  # input untouched


class WindowTest(unittest.TestCase):
    def test_decisions(self):
        self.assertEqual(bagtime.window_decision(5, 10, 0.0), 'skip')
        self.assertEqual(bagtime.window_decision(10, 10, 0.0), 'deliver')
        self.assertEqual(bagtime.window_decision(12, 10, 0.0, 5e-9), 'deliver')
        self.assertEqual(bagtime.window_decision(20, 10, 0.0, 5e-9), 'end')
        self.assertEqual(bagtime.window_decision(10**10 + 11, 10, 0.0, 10.0), 'end')

    def test_validate(self):
        self.assertEqual(bagtime.validate_window(1, 5, 10), (1.0, 5.0))
        with self.assertRaises(ValueError):
            bagtime.validate_window(5, 1, 10)
        with self.assertRaises(ValueError):
            bagtime.validate_window(True, 5, 10)


class MetricsTest(unittest.TestCase):
    def test_tcp_metrics(self):
        s = series([[0, 0, 0], [0.1, 0.02, 0], [0.2, 0.01, 0]])
        m = bagtime.tcp_metrics(s)
        self.assertAlmostEqual(m['span_cm'][0], 20.0)
        self.assertAlmostEqual(m['oriexc_deg'], 0.0)
        self.assertGreater(m['travel_m'], 0.19)
        with self.assertRaises(ValueError):
            bagtime.tcp_metrics([])

    def test_quat_angle(self):
        self.assertAlmostEqual(bagtime.quat_angle_deg([0, 0, 0, 1], rot_z(90)), 90.0)
        self.assertAlmostEqual(bagtime.quat_angle_deg([0, 0, 0, 1], [0, 0, 0, 1]), 0.0)

    def test_sample_flow(self):
        f = bagtime.sample_flow_metrics([0.0, 0.1, 0.2, 1.0], 1.0)
        self.assertEqual(f['count'], 4)
        self.assertAlmostEqual(f['max_gap_s'], 0.8)
        self.assertEqual(bagtime.sample_flow_metrics([], 2.0)['count'], 0)


class CompareTest(unittest.TestCase):
    def flow(self, n=50, gap=0.1):
        return dict(count=n, rate_hz=10.0, max_gap_s=gap)

    def test_neutral_pass(self):
        s = series([[0.3, 0.0, 0.48]] * 20)
        r = bagtime.compare_take('00_neutro', s, self.flow())
        self.assertTrue(r['ok'], bagtime.check_line(r))

    def test_neutral_fails_when_moving(self):
        s = series([[0.3 + i * 0.01, 0, 0.48] for i in range(20)])
        r = bagtime.compare_take('00_neutro', s, self.flow())
        self.assertFalse(r['ok'])

    def test_height_pass(self):
        s = series([[0.3, 0.0, 0.48 - i * 0.01] for i in range(20)])
        r = bagtime.compare_take('04_frente_tras', s, self.flow())
        self.assertTrue(r['ok'], bagtime.check_line(r))

    def test_height_fails_wrong_axis(self):
        s = series([[0.3 + i * 0.01, 0, 0.48] for i in range(20)])
        r = bagtime.compare_take('04_frente_tras', s, self.flow())
        self.assertFalse(r['ok'])

    def test_roll_pass(self):
        s = [dict(t=i * 0.1, p=[0.3, 0, 0.48], q=rot_z(i * 5)) for i in range(20)]
        r = bagtime.compare_take('01_giro_plano_imagem', s, self.flow())
        self.assertTrue(r['ok'], bagtime.check_line(r))

    def test_gripper(self):
        s = series([[0.3, 0, 0.48]] * 20)
        r = bagtime.compare_take('07_abrir_fechar', s, self.flow(),
                                 gripper_widths=[0.08, 0.06, 0.02, 0.07])
        self.assertTrue(r['ok'], bagtime.check_line(r))
        r2 = bagtime.compare_take('07_abrir_fechar', s, self.flow(),
                                  gripper_widths=[0.08] * 10)
        self.assertFalse(r2['ok'])

    def test_no_samples_fails(self):
        s = series([[0.3, 0, 0.48]] * 20)
        r = bagtime.compare_take('00_neutro', s, dict(count=2, rate_hz=1.0, max_gap_s=0.1))
        self.assertFalse(r['ok'])

    def test_unknown_stage(self):
        with self.assertRaises(ValueError):
            bagtime.compare_take('12_nope', series([[0, 0, 0]]), self.flow())

    def test_check_line(self):
        s = series([[0.3, 0, 0.48]] * 20)
        line = bagtime.check_line(bagtime.compare_take('00_neutro', s, self.flow()))
        self.assertIn('CHECK 00_neutro PASS', line)


class ReinferTest(unittest.TestCase):
    def test_drift(self):
        rec = [dict(offset_s=i * 0.1, p=[i * 0.01, 0, 0]) for i in range(10)]
        fresh = [dict(offset_s=i * 0.1 + 0.01, p=[i * 0.01, 0, 0]) for i in range(10)]
        med, n = bagtime.reinfer_drift(rec, fresh)
        self.assertEqual(n, 10)
        self.assertAlmostEqual(med, 0.0, places=5)

    def test_absolute_offset_ignored(self):
        # Same motion, 2 m absolute offset (as measured on 00_neutro): drift ~0.
        rec = [dict(offset_s=i * 0.1, p=[-2.3 + i * 0.01, 0.95, -0.4]) for i in range(10)]
        fresh = [dict(offset_s=i * 0.1, p=[-1.1 + i * 0.01, -0.2, 0.0]) for i in range(10)]
        med, n = bagtime.reinfer_drift(rec, fresh)
        self.assertEqual(n, 10)
        self.assertAlmostEqual(med, 0.0, places=5)

    def test_shape_change_detected(self):
        rec = [dict(offset_s=i * 0.1, p=[0.0, 0, 0]) for i in range(10)]
        fresh = [dict(offset_s=i * 0.1, p=[i * 0.05, 0, 0]) for i in range(10)]
        med, n = bagtime.reinfer_drift(rec, fresh)
        self.assertGreater(med, 5.0)

    def test_slow_inter_version_drift_tolerated(self):
        # Same motion, offset drifting 0.5 cm/s over 30 s: windowed stays small.
        rec = [dict(offset_s=i * 0.5, p=[i * 0.02, 0, 0]) for i in range(60)]
        fresh = [dict(offset_s=i * 0.5, p=[2.0 + i * 0.02 + i * 0.005, 0, 0]) for i in range(60)]
        med, n = bagtime.reinfer_drift(rec, fresh)
        self.assertEqual(n, 60)
        self.assertLess(med, 5.0)

    def test_empty(self):
        med, n = bagtime.reinfer_drift([], [dict(offset_s=0, p=[0, 0, 0])])
        self.assertEqual(n, 0)


class OrientDriftTest(unittest.TestCase):
    def test_same_rotation(self):
        rec = [dict(offset_s=i * 0.1, q=rot_z(i * 10)) for i in range(10)]
        fresh = [dict(offset_s=i * 0.1, q=rot_z(i * 10)) for i in range(10)]
        med, n = bagtime.reinfer_orient_drift(rec, fresh)
        self.assertEqual(n, 10)
        self.assertAlmostEqual(med, 0.0, places=4)

    def test_absolute_frame_ignored(self):
        # Same relative motion from a different start frame: drift ~0.
        rec = [dict(offset_s=i * 0.1, q=rot_z(i * 10)) for i in range(10)]
        off = rot_z(45)
        fresh = [dict(offset_s=i * 0.1, q=bagtime._qmul(rot_z(i * 10), off)) for i in range(10)]
        med, n = bagtime.reinfer_orient_drift(rec, fresh)
        self.assertAlmostEqual(med, 0.0, places=3)

    def test_divergence_detected(self):
        rec = [dict(offset_s=i * 0.1, q=rot_z(0)) for i in range(10)]
        fresh = [dict(offset_s=i * 0.1, q=rot_z(i * 10)) for i in range(10)]
        med, n = bagtime.reinfer_orient_drift(rec, fresh)
        self.assertGreater(med, 20.0)

    def test_missing_q(self):
        med, n = bagtime.reinfer_orient_drift([dict(offset_s=0)], [dict(offset_s=0, q=rot_z(0))])
        self.assertEqual(n, 0)

    def test_outlier_anchor_robust(self):
        # A few wild orientation pairs must not skew the whole comparison.
        rec = [dict(offset_s=i * 0.5, q=rot_z(i * 5)) for i in range(20)]
        fresh = [dict(offset_s=i * 0.5, q=rot_z(i * 5)) for i in range(20)]
        fresh[10] = dict(offset_s=5.0, q=rot_z(170))  # outlier at the median
        med, n = bagtime.reinfer_orient_drift(rec, fresh)
        self.assertEqual(n, 20)
        self.assertLess(med, 5.0)


class SimReviewTest(unittest.TestCase):
    def test_load(self):
        doc = dict(runs=[dict(stage='00_neutro', status='done'),
                          dict(stage='04_frente_tras', status='done')])
        with tempfile.NamedTemporaryFile('w', suffix='.json', delete=False) as f:
            json.dump(doc, f)
            path = f.name
        try:
            runs = bagtime.load_sim_review(path)
            self.assertEqual(set(runs), {'00_neutro', '04_frente_tras'})
        finally:
            os.unlink(path)


class WristJumpTest(unittest.TestCase):
    def test_counts_teleports(self):
        s = [dict(wall_t=float(i), p=[i * 0.01, 0, 0]) for i in range(5)]
        s.append(dict(wall_t=5.0, p=[0.5, 0, 0]))  # 0.46 m teleport
        m = bagtime.wrist_jump_metrics(s)
        self.assertEqual(m['n_big_jumps'], 1)
        self.assertAlmostEqual(m['max_jump_m'], 0.46, places=2)

    def test_smooth_passes(self):
        s = [dict(wall_t=float(i), p=[i * 0.02, 0, 0]) for i in range(10)]
        m = bagtime.wrist_jump_metrics(s)
        self.assertEqual(m['n_big_jumps'], 0)
        self.assertAlmostEqual(m['max_jump_m'], 0.02, places=2)

    def test_empty_and_single(self):
        self.assertEqual(bagtime.wrist_jump_metrics([])['n_big_jumps'], 0)
        one = bagtime.wrist_jump_metrics([dict(wall_t=0, p=[0, 0, 0])])
        self.assertEqual((one['n_big_jumps'], one['max_jump_m']), (0, 0.0))


class RecognitionVerdictTest(unittest.TestCase):
    def samp(self, n, dt=0.3, spike_at=None, spike_m=0.0):
        out = []
        for i in range(n):
            out.append(dict(offset_s=i * dt, wall_t=100.0 + i * dt,
                            p=[i * 0.01 + (spike_m if i == spike_at else 0.0), 0, 0],
                            q=[0, 0, 0, 1], gesture='Open_Palm'))
        return out

    def test_position_pass(self):
        v = bagtime.recognition_verdict(
            '04_frente_tras', self.samp(12), self.samp(12),
            dict(count=12, rate_hz=3.0, max_gap_s=0.3))
        self.assertTrue(v['ok'])
        self.assertEqual((v['metric'], v['jumps_ok']), ('position', True))

    def test_spike_fails_despite_good_drift(self):
        # One 0.5 m single-sample spike: the windowed median drift still
        # passes, but the jumps gate must fail (dry-04's 0.80/0.77 m pair).
        v = bagtime.recognition_verdict(
            '04_frente_tras', self.samp(12, spike_at=6, spike_m=0.5), self.samp(12),
            dict(count=12, rate_hz=3.0, max_gap_s=0.3))
        self.assertTrue(v['metric_ok'])
        self.assertFalse(v['jumps_ok'])
        self.assertFalse(v['ok'])
        self.assertEqual(v['n_big_jumps'], 2)  # out and back

    def test_flow_still_gates(self):
        v = bagtime.recognition_verdict(
            '04_frente_tras', self.samp(12), self.samp(12),
            dict(count=12, rate_hz=3.0, max_gap_s=5.0))
        self.assertFalse(v['flow_ok'])
        self.assertFalse(v['ok'])


if __name__ == '__main__':
    unittest.main()
