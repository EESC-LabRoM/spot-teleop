import unittest
import numpy as np
from body_wrist_reference import body_wrist_supported,ShoulderRelativeWrist

class BodyWristTests(unittest.TestCase):
    def test_borderline_wrist_uses_current_arm_support(self):
        for visibility in (.35,.5,.649):
            self.assertTrue(body_wrist_supported(visibility,np.array([.2,.1,1.]),np.array([0.,0.,1.]),.9,.99,.5))
    def test_current_finger_depth_can_support_wrist_when_elbow_is_weak(self):
        self.assertTrue(body_wrist_supported(.48,np.array([.2,.1,1.]),np.array([0.,0.,1.]),.29,.99,.5,hand_support=1))
    def test_weak_wrist_rejects_missing_support_or_impossible_reach(self):
        self.assertFalse(body_wrist_supported(.6,np.array([.2,.1,1.])))
        self.assertFalse(body_wrist_supported(.2,np.array([.2,.1,1.]),np.array([0.,0.,1.]),.9,.99,.5))
        self.assertFalse(body_wrist_supported(.6,np.array([1.,0.,1.]),np.array([0.,0.,1.]),.9,.99,.5))
    def test_strong_current_wrist_does_not_require_shoulder(self):
        self.assertTrue(body_wrist_supported(.9,np.array([.2,.1,1.])))
    def test_common_torso_translation_is_removed_without_reference_jump(self):
        f=ShoulderRelativeWrist();s=np.array([0.,0.,1.]);w=np.array([.2,.1,1.])
        np.testing.assert_allclose(f.update(w,0.,s,s),w)
        for i in range(1,21):
            offset=np.array([i*.002,0.,0.]);out=f.update(w+offset,i*.1,s,s+offset)
        np.testing.assert_allclose(out,w,atol=1e-9)
    def test_missing_shoulder_does_not_stop_current_wrist(self):
        f=ShoulderRelativeWrist();s=np.array([0.,0.,1.]);w=np.array([.2,.1,1.])
        f.update(w,0.,s,s)
        np.testing.assert_allclose(f.update(w+[.02,0.,0.],.1,s),w+[.02,0.,0.])
    def test_shoulder_reacquisition_is_bounded(self):
        f=ShoulderRelativeWrist();s=np.array([0.,0.,1.]);w=np.array([.2,.1,1.])
        f.update(w,0.,s,s);out=f.update(w,10.,s,s+[.2,0.,0.])
        self.assertLessEqual(np.linalg.norm(out-w),.01600001)

if __name__=='__main__':unittest.main()
