import unittest,math
import numpy as np
from stable_palm_pose import RobustPalmFrame,AdaptivePalmOrientation,angle

class PalmStabilityTests(unittest.TestCase):
    def palm(self):return np.array([[0,0,1],[.035,.065,1],[.01,.075,1],[-.015,.07,1],[-.035,.055,1]])
    def q(self,a):return np.array([0,0,math.sin(a/2),math.cos(a/2)])
    def test_fit_follows_real_rotation_and_translation(self):
        f=RobustPalmFrame();p=self.palm();f.update(p,np.eye(3))
        a=.7;r=np.array([[math.cos(a),-math.sin(a),0],[math.sin(a),math.cos(a),0],[0,0,1]])
        out,d=f.update(p@r.T+[.3,-.2,.1],None)
        np.testing.assert_allclose(out,r,atol=1e-6);self.assertEqual(d['orientation_inliers'],5)
    def test_one_bad_knuckle_does_not_flip_orientation(self):
        f=RobustPalmFrame();p=self.palm();f.update(p,np.eye(3));p[1]+=[.04,.01,.05]
        out,d=f.update(p,None)
        np.testing.assert_allclose(out,np.eye(3),atol=1e-6);self.assertEqual(d['orientation_inliers'],4)
    def test_unobservable_collinear_palm_is_rejected(self):
        f=RobustPalmFrame();f.update(self.palm(),np.eye(3))
        p=self.palm();p[:,0]=0;p[:,2]=1
        with self.assertRaises(ValueError):f.update(p,None)
    def test_quaternion_sign_change_is_not_a_rotation(self):
        f=AdaptivePalmOrientation();f.update(self.q(.1),1.)
        self.assertLess(angle(f.update(-self.q(.1),1.1),self.q(.1)),1e-6)
    def test_isolated_flip_is_rejected_and_next_good_measurement_recovers(self):
        f=AdaptivePalmOrientation();f.update(self.q(0),1.)
        with self.assertRaises(ValueError):f.update(self.q(math.pi),1.1)
        self.assertLess(angle(f.update(self.q(.02),1.2),self.q(0)),.03)
    def test_long_loss_requires_two_current_measurements(self):
        f=AdaptivePalmOrientation();f.update(self.q(0),1.)
        with self.assertRaises(ValueError):f.update(self.q(1.5),3.)
        self.assertLess(angle(f.update(self.q(1.51),3.15),self.q(1.51)),1e-6)
    def test_reacquires_coherent_turn_at_two_hz(self):
        f=AdaptivePalmOrientation();f.update(self.q(0),1.)
        with self.assertRaises(ValueError):f.update(self.q(1.5),3.)
        # 30 degrees in .5 seconds is a measurable, physically plausible turn.
        self.assertLess(angle(f.update(self.q(2.0),3.5),self.q(2.0)),1e-6)
    def test_incoherent_reacquisition_is_not_confirmed(self):
        f=AdaptivePalmOrientation();f.update(self.q(0),1.)
        with self.assertRaises(ValueError):f.update(self.q(1.5),3.)
        with self.assertRaises(ValueError):f.update(self.q(-1.5),3.5)
    def test_rejected_observation_cannot_be_replayed(self):
        f=AdaptivePalmOrientation();f.update(self.q(0),1.)
        with self.assertRaises(ValueError):f.update(self.q(math.pi),1.1)
        with self.assertRaises(ValueError):f.update(self.q(.02),1.05)
    def test_pending_measurements_must_be_recent(self):
        f=AdaptivePalmOrientation();f.update(self.q(0),1.)
        with self.assertRaises(ValueError):f.update(self.q(1.5),3.)
        with self.assertRaises(ValueError):f.update(self.q(1.51),4.)

    def test_smoothing_reduces_stationary_noise(self):
        f=AdaptivePalmOrientation();out=[]
        for i in range(30):out.append(f.update(self.q(.04*(-1)**i),1+i/8.))
        self.assertLess(np.std([2*math.atan2(q[2],q[3]) for q in out[5:]]),.025)
    def test_continuous_turn_stays_bounded_and_tracks(self):
        f=AdaptivePalmOrientation();last=None
        for i in range(21):
            q=f.update(self.q(i*.08),1+i*.1)
            if last is not None:self.assertLessEqual(angle(last,q),math.radians(120)*.1+1e-6)
            last=q
        self.assertLess(angle(q,self.q(1.6)),.16)

class PalmDepthTests(unittest.TestCase):
    def test_interior_sites_match_current_landmark_coordinates(self):
        from hand_tracking_math import palm_depth_sites,fused_wrist_depth
        pixels=np.arange(42).reshape(21,2).astype(float)
        world=np.zeros((21,3));world[:,2]=np.arange(21)*.001
        sites,z=palm_depth_sites(pixels,world)
        self.assertEqual(len(sites),9)
        np.testing.assert_allclose(sites[5],pixels[[0,5,9]].mean(0))
        # Wrist/MCP silhouette holes: interior depths still support the current wrist.
        measurements=[None]*5+[.7+x for x in z[5:]]
        value,count=fused_wrist_depth(measurements,z)
        self.assertAlmostEqual(value,.7);self.assertEqual(count,4)
