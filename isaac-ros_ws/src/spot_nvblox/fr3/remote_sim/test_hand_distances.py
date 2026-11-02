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

    def test_hidden_thumb_does_not_disable_four_open_fingers(self):
        p=self.hand();p[1:5]=0.
        self.assertEqual(finger_distance_signals(p)['gesture'],'Open_Palm')
        p=self.hand((1,2,3,4));p[1:5]=0.
        self.assertEqual(finger_distance_signals(p)['gesture'],'Closed_Fist')

    def test_pointing_and_victory_are_neither_open_palm_nor_fist(self):
        self.assertIsNone(finger_distance_signals(self.hand((2,3,4)))['gesture'])
        self.assertIsNone(finger_distance_signals(self.hand((3,4)))['gesture'])

    def test_relaxed_open_fingers_do_not_require_perfectly_straight_chains(self):
        p=self.hand()
        import math
        for finger in range(1,5):
            mcp=1+finger*4
            p[mcp+1]=p[mcp]+[.03,0,0]
            p[mcp+2]=p[mcp+1]+[.03*math.cos(.5),.03*math.sin(.5),0]
            p[mcp+3]=p[mcp+2]+[.03*math.cos(.8),.03*math.sin(.8),0]
        self.assertEqual(finger_distance_signals(p)['gesture'],'Open_Palm')

    def test_three_open_fingers_survive_one_missing_finger(self):
        p=self.hand();p[17:21]=0.
        self.assertEqual(finger_distance_signals(p)['gesture'],'Open_Palm')

    def test_two_visible_fingers_cannot_decide_gripper_intent(self):
        p=self.hand();p[13:21]=0.
        self.assertIsNone(finger_distance_signals(p))
