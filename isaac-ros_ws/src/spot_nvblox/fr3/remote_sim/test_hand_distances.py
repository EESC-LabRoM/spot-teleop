import unittest
import numpy as np
from hand_tracking_math import finger_distance_signals

class FingerDistanceTests(unittest.TestCase):
    def hand(self,closed=()):
        p=np.zeros((21,3))
        for finger in range(5):
            ids=list(range(1+finger*4,5+finger*4))
            p[ids]=[[.02,0,0],[.05,0,0],[.08,0,0],[.11,0,0]]
            if finger in closed:p[ids]=[[.02,0,0],[.05,0,0],[.05,.03,0],[.02,.03,0]]
        return p
    def test_open_and_closed_use_metric_distances(self):
        self.assertEqual(finger_distance_signals(self.hand())['gesture'],'Open_Palm')
        self.assertEqual(finger_distance_signals(self.hand((1,2,3,4)))['gesture'],'Closed_Fist')
    def test_one_bad_finger_is_outvoted(self):
        self.assertEqual(finger_distance_signals(self.hand((2,)))['gesture'],'Open_Palm')
    def test_scale_rotation_and_translation_do_not_change_grip(self):
        p=self.hand((1,2,3,4));a=finger_distance_signals(p)
        r=np.array([[0,-1,0],[1,0,0],[0,0,1]])
        b=finger_distance_signals((p@r.T)*1.3+[.4,.2,1.])
        self.assertAlmostEqual(a['grip'],b['grip']);self.assertEqual(a['gesture'],b['gesture'])
    def test_degenerate_geometry_is_not_a_command(self):
        self.assertIsNone(finger_distance_signals(np.zeros((21,3))))
