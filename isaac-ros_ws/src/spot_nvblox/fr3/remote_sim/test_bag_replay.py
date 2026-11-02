import json
import math
import struct
import unittest
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from bag_replay import (
    decode_depth, source_topics, RGB, DEPTH, INFO, RS_RGB, RS_DEPTH, RS_INFO,
    filter_window_messages,
    rebase_packet,
    validate_seek_window,
    window_decision,
)
from teleop_mapping import paired_sample, control_sample, mapping_settings, rotation_delta


class ReplayTests(unittest.TestCase):
    def test_source_profiles_preserve_zed_and_accept_realsense(self):
        self.assertEqual(source_topics({RGB}), (RGB,DEPTH,INFO))
        self.assertEqual(source_topics({RS_RGB}), (RS_RGB,RS_DEPTH,RS_INFO))
        with self.assertRaises(ValueError):source_topics({"/unrelated"})

    def test_millisecond_cut_rounding_clamps_to_exact_bag_end(self):
        self.assertEqual(validate_seek_window(0,23.087,23.086720256),(0.,23.086720256))
        with self.assertRaises(ValueError):validate_seek_window(0,23.088,23.086720256)

    def packet(self):
        return dict(version=3, frame='fr3_link0', orientation_mode='full_6d', stamp_ns=100,
                    orientation_stamp_ns=100, position_valid=True, orientation_valid=True,
                    p=[1., 2., 3.], q=[0., 0., 0., 1.])

    def test_rebase_keeps_pairing_and_original(self):
        packet = self.packet()
        rebased = rebase_packet(packet, lambda stamp: stamp+12345)
        self.assertEqual(rebased['stamp_ns'], rebased['orientation_stamp_ns'])
        self.assertEqual(rebased['stamp_ns']-12355, packet['stamp_ns']-10)
        self.assertEqual(packet['stamp_ns'], 100)
        self.assertIsNotNone(paired_sample(rebased))

    def test_position_only_does_not_become_full_pose(self):
        packet = self.packet()
        packet.update(q=None, orientation_valid=False, orientation_stamp_ns=None)
        rebased = rebase_packet(packet, lambda stamp: stamp+900)
        self.assertIsNone(rebased['orientation_stamp_ns'])
        self.assertIsNone(paired_sample(rebased))

    def test_position_only_moves_position_and_holds_prior_orientation(self):
        packet = self.packet()
        packet.update(q=None, orientation_valid=False, orientation_stamp_ns=None, p=[.2,.1,.3])
        self.assertIsNone(control_sample(packet))
        previous = np.array([0.,0.,0.,1.])
        stamp,p,q,current = control_sample(packet,previous)
        np.testing.assert_allclose(p,[.2,.1,.3]);np.testing.assert_allclose(q,previous)
        self.assertFalse(current)
        packet['position_valid']=False
        self.assertIsNone(control_sample(packet,previous))

    def test_bad_orientation_packet_cannot_impersonate_position_fallback(self):
        packet=self.packet();packet['orientation_stamp_ns']=99
        self.assertIsNone(control_sample(packet,np.array([0.,0.,0.,1.])))

    def test_v2_and_v3_accept_but_invalid_flags_and_unpaired_reject(self):
        for version in (2, 3):
            packet = self.packet(); packet['version'] = version
            self.assertIsNotNone(paired_sample(packet))
        for key, value in [('orientation_stamp_ns', 99), ('position_valid', False),
                           ('orientation_valid', False), ('q', [0., 0., 0., 0.]), ('p', [float('nan'), 0., 0.])]:
            packet = self.packet(); packet[key] = value
            self.assertIsNone(paired_sample(packet))

    def test_compressed_depth_inverse_and_invalid_pixels(self):
        raw = np.array([[0, 1010], [510, 260]], np.uint16)
        ok, png = cv2.imencode('.png', raw); self.assertTrue(ok)
        message = CompressedImage(format='32FC1; compressedDepth png',
                                  data=struct.pack('<iff', 0, 1000., 10.)+png.tobytes())
        depth = decode_depth(message)
        self.assertTrue(np.isnan(depth[0, 0]))
        np.testing.assert_allclose(depth[raw > 0], [1., 2., 4.])
        message.format = '16UC1; compressedDepth png'
        np.testing.assert_allclose(decode_depth(message)[raw > 0], [1.01, .51, .26])

    def test_relative_rotation_gain_and_quaternion_sign(self):
        q0 = np.array([math.sin(.3), 0., 0., math.cos(.3)])
        # A rotation about world Z applied on the left of an X-rotated anchor.
        q = np.array([math.cos(.2)*q0[0], math.sin(.2)*q0[0], math.sin(.2)*q0[3], math.cos(.2)*q0[3]])
        np.testing.assert_allclose(rotation_delta(q, q0, .5), [0., 0., math.sin(.1), math.cos(.1)], atol=1e-10)
        np.testing.assert_allclose(rotation_delta(-q, q0, 0.), [0., 0., 0., 1.], atol=1e-10)

    def test_axis_gains_support_direction_and_scale(self):
        gains, _ = mapping_settings([-1., .5, 2.], 1.)
        np.testing.assert_allclose(gains*np.array([.1, .2, .3]), [-.1, .1, .6])
        for gains, rotation in [([1, 2], 1), ([1, float('nan'), 1], 1), ([1, 1, 1], -1)]:
            with self.assertRaises(ValueError):
                mapping_settings(gains, rotation)

    def test_seek_window_validation_and_skip_deliver_end(self):
        start_ns = 1_000_000_000_000
        duration = 20.0
        start_s, end_s = validate_seek_window(5.0, 10.0, duration)
        self.assertEqual((start_s, end_s), (5.0, 10.0))

        for bad_start, bad_end in [
            (-0.1, 5.0),
            (5.0, 5.0),
            (6.0, 5.0),
            (0.0, 20.1),
            (float('nan'), 5.0),
            (0.0, float('inf')),
            (True, 5.0),
            (None, 5.0),
        ]:
            with self.assertRaises(ValueError):
                validate_seek_window(bad_start, bad_end, duration)

        t_before = start_ns + round((start_s - 0.1) * 1e9)
        t_inside = start_ns + round((start_s + 0.1) * 1e9)
        t_after = start_ns + round((end_s + 0.1) * 1e9)
        t_later = start_ns + round((end_s + 1.0) * 1e9)

        self.assertEqual(window_decision(t_before, start_ns, start_s, end_s), 'skip')
        self.assertEqual(window_decision(t_inside, start_ns, start_s, end_s), 'deliver')
        self.assertEqual(window_decision(t_after, start_ns, start_s, end_s), 'end')

        messages = [
            ('sample', b'early', t_before),
            ('sample', b'inside', t_inside),
            ('sample', b'past_end', t_after),
            ('sample', b'unreachable', t_later),
        ]
        delivered, skipped, ended = filter_window_messages(messages, start_ns, start_s, end_s)
        self.assertEqual(skipped, 1)
        self.assertEqual(len(delivered), 1)
        self.assertEqual(delivered[0][1], b'inside')
        self.assertTrue(ended)


if __name__ == '__main__':
    unittest.main()
