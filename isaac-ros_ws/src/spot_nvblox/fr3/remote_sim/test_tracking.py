import math
import unittest
import numpy as np
from tracking_gate import TrackingGate
from hand_tracking_math import OneEuroPosition, CAMERA_TO_ROBOT, fused_wrist_depth

class RecoveryTests(unittest.TestCase):
    def setUp(self):self.gate=TrackingGate();self.q=np.array([0.,0.,0.,1.])
    def feed(self,t,p=(0.,0.,0.)):
        self.gate.observe(int(t*1e9),t,np.array(p),self.q)
        return self.gate.step(t+.15)
    def activate(self):
        self.gate.start(1.)
        for t in (1.1,1.3,1.55):active=self.feed(t)
        self.assertTrue(active);self.assertEqual(self.gate.phase,'active')
    def test_click_is_idempotent_and_calibration_requires_new_stable_measurements(self):
        self.gate.start(1.);deadline=self.gate.deadline
        self.gate.start(2.);self.assertEqual(self.gate.deadline,deadline)
        self.assertFalse(self.feed(2.1));self.assertFalse(self.feed(2.3));self.assertTrue(self.feed(2.55))
    def test_duplicates_never_complete_calibration(self):
        self.gate.start(1.)
        self.feed(1.1)
        for t in (1.3,1.5,1.7):self.gate.step(t)
        self.assertEqual(self.gate.phase,'calibrating');self.assertEqual(len(self.gate.window),1)
    def test_early_loss_holds_then_recovers_without_start_and_requires_reanchor(self):
        self.activate();self.assertFalse(self.gate.step(2.3));self.assertEqual(self.gate.phase,'paused')
        self.assertFalse(self.feed(5.0,(1.,0.,0.)));self.assertFalse(self.feed(5.2,(1.,0.,0.)))
        self.assertTrue(self.feed(5.4,(1.,0.,0.)));self.assertEqual(self.gate.phase,'active')
    def test_stop_prevents_automatic_activation_after_recovery(self):
        self.activate();self.gate.stop(2.)
        for t in (2.1,2.3,2.6):self.assertFalse(self.feed(t))
        self.assertEqual(self.gate.phase,'idle')
    def test_permanent_loss_disarms(self):
        self.activate();self.gate.step(2.3);self.gate.step(32.4)
        self.assertEqual(self.gate.phase,'idle')
    def test_jump_does_not_calibrate(self):
        self.gate.start(1.)
        for t,p in ((1.1,(0,0,0)),(1.3,(.2,0,0)),(1.6,(0,0,0))):self.assertFalse(self.feed(t,p))
    def test_nonfinite_pose_cannot_activate(self):
        self.gate.start(1.)
        for t in (1.1,1.3,1.6):
            self.gate.observe(int(t*1e9),t,np.array([float('nan'),0.,0.]),self.q)
            self.assertFalse(self.gate.step(t))
        self.assertIsNone(self.gate.sample)
    def test_stale_sample_cannot_activate(self):
        self.gate.start(1.)
        for t in (1.1,1.3,1.6):
            self.gate.observe(int(t*1e9),t,np.zeros(3),self.q);self.assertFalse(self.gate.step(t+1.))
        self.assertEqual(self.gate.phase,'calibrating')

class DepthAndMappingTests(unittest.TestCase):
    def test_camera_mapping_is_a_rotation_and_forward_is_into_table(self):
        self.assertAlmostEqual(np.linalg.det(CAMERA_TO_ROBOT),1.)
        np.testing.assert_allclose(CAMERA_TO_ROBOT@[0,0,-.1],[.1,0,0])
        np.testing.assert_allclose(CAMERA_TO_ROBOT@[0,-.1,0],[0,0,.1])
    def test_missing_wrist_is_reconstructed_only_from_current_knuckle_depth(self):
        depth,count=fused_wrist_depth([None,.66,.67,.68,.69],[0,.01,.02,.03,.04])
        self.assertAlmostEqual(depth,.65);self.assertEqual(count,4)
    def test_background_at_wrist_does_not_pull_hand_back(self):
        depth,count=fused_wrist_depth([2.,.65,.66,.65,.65],[0]*5)
        self.assertLess(abs(depth-.65),.01);self.assertEqual(count,4)
    def test_no_depth_is_never_filled_from_history(self):
        depth,count=fused_wrist_depth([None]*5,[0]*5)
        self.assertIsNone(depth);self.assertEqual(count,0)
    def test_single_occluded_knuckle_is_not_enough(self):
        self.assertIsNone(fused_wrist_depth([None,.65,None,None,None],[0]*5)[0])
    def test_adaptive_filter_smooths_noise_and_rejects_outlier(self):
        f=OneEuroPosition();f.update([0,0,0],1.)
        values=[f.update([.002*(-1)**i,0,0],1.+i/30.)[0] for i in range(1,30)]
        self.assertLess(np.std(values),.002)
        with self.assertRaises(ValueError):f.update([2,0,0],2.)
    def test_motion_response_does_not_inherit_slow_fixed_frame_ema(self):
        f=OneEuroPosition();f.update([0,0,0],1.)
        for i in range(1,11):value=f.update([i*.025,0,0],1.+i*.1)
        self.assertLess(abs(value[0]-.25),.03)
    def test_filter_resets_after_loss_instead_of_dragging_toward_old_hand(self):
        f=OneEuroPosition();f.update([0,0,0],1.)
        np.testing.assert_allclose(f.update([1,0,0],2.),[1,0,0])

if __name__=='__main__':unittest.main()
