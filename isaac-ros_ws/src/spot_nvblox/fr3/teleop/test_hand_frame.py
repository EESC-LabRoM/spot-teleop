"""Synthetic RGB-D hand-frame checks; NumPy only, no robot or camera."""
import importlib.util
import math
from pathlib import Path
import unittest
import numpy as np

path=Path(__file__).resolve().parents[5]/'spot-ros2_ws/src/arm_pose_estimator/arm_pose_estimator/hand_frame.py'
spec=importlib.util.spec_from_file_location('hand_frame',path); H=importlib.util.module_from_spec(spec);spec.loader.exec_module(H)
PALM=np.array([[0.,0.,0.],[.07,.035,0.],[.08,.012,0.],[.08,-.012,0.],[.07,-.035,0.]])

def axis_rotation(axis,angle):
    q=np.r_[np.asarray(axis)*math.sin(angle/2),math.cos(angle/2)]
    return H.quaternion_matrix(q)

def points(rotation=np.eye(3),shift=np.array([0.,0.,2.])):
    return PALM@rotation.T+shift

class HandFrameTests(unittest.TestCase):
    def test_open_fingers_any_rotation_and_thumb_position(self):
        p=np.zeros((21,3))
        for mcp,x in zip((5,9,13,17),(-.03,-.01,.01,.03)):
            p[mcp:mcp+4]=[[x,.06,0],[x,.09,0],[x,.11,0],[x,.13,0]]
        for r in (np.eye(3),axis_rotation([0,1,0],1.5),axis_rotation([1,0,0],2.4)):
            self.assertTrue(H.fingers_open(p@r.T+[0,0,2]))
        p[1:5]=[.02,.04,0]  # thumb is not required to spread
        self.assertTrue(H.fingers_open(p))

    def test_fist_partial_or_invalid_hand_is_not_open(self):
        p=np.zeros((21,3))
        for mcp,x in zip((5,9,13,17),(-.03,-.01,.01,.03)):
            p[mcp:mcp+4]=[[x,.06,0],[x,.09,0],[x,.11,0],[x,.13,0]]
        for mcp in (5,9,13,17):
            curled=p.copy();curled[mcp+2:mcp+4]=[[0,.08,.02],[0,.06,.01]]
            self.assertFalse(H.fingers_open(curled))
        self.assertFalse(H.fingers_open(np.zeros((21,3))))
        self.assertFalse(H.fingers_open(np.full((21,3),np.nan)))

    def test_continuous_fast_turn_does_not_trip_on_filter_lag(self):
        f=H.OrientationFilter()
        previous=None
        for i in range(25):
            t=i/8.;q=H.matrix_quaternion(axis_rotation([0,1,0],math.radians(180)*t))
            out=f.update(q,t)
            if previous is not None:
                angle=2*math.acos(min(1.,abs(float(previous@out))))
                self.assertLessEqual(angle,math.radians(120)/8.+1e-7)
            previous=out

    def test_response_time_consistent_across_camera_rates(self):
        outputs=[]
        for hz in (8,30):
            f=H.OrientationFilter()
            for i in range(hz+1):out=f.update(H.matrix_quaternion(axis_rotation([0,0,1],.5*i/hz)),i/hz)
            outputs.append(out)
        self.assertLess(2*math.acos(min(1.,abs(float(outputs[0]@outputs[1])))),.04)

    def test_grasp_lock_and_release_do_not_rotate_until_deliberate_turn(self):
        lock=H.GestureOrientationLock();identity=np.array([0.,0.,0.,1.])
        lock.update(identity,'Open_Palm',0.)
        raw=H.matrix_quaternion(axis_rotation([1,0,0],1.))
        np.testing.assert_allclose(lock.update(raw,'Closed_Fist',.1,anchor=identity),identity)
        np.testing.assert_allclose(lock.update(raw,'Closed_Fist',1.),identity)
        np.testing.assert_allclose(lock.update(raw,'Closed_Fist',2.2),identity,atol=1e-12)
        delta=axis_rotation([0,1,0],.3)
        changed=H.matrix_quaternion(delta@H.quaternion_matrix(raw))
        np.testing.assert_allclose(H.quaternion_matrix(lock.update(changed,'Closed_Fist',2.3)),delta,atol=1e-12)

    def test_opening_rebases_fist_shape_and_hold_rejects_replays(self):
        lock=H.GestureOrientationLock();f=H.OrientationFilter();identity=np.array([0.,0.,0.,1.])
        f.update(identity,0.);lock.update(identity,'Open_Palm',0.)
        raw=H.matrix_quaternion(axis_rotation([1,0,0],.6))
        lock.update(raw,'Closed_Fist',.1,anchor=f.q)
        np.testing.assert_allclose(f.hold(.1),identity)
        with self.assertRaises(ValueError):f.hold(.1)
        lock.update(raw,'Closed_Fist',2.2)
        other=H.matrix_quaternion(axis_rotation([0,1,0],.9))
        np.testing.assert_allclose(lock.update(other,'Open_Palm',2.3,anchor=identity),identity)
        np.testing.assert_allclose(lock.update(other,'Open_Palm',2.7),identity,atol=1e-12)

    def test_depth_rejects_background_and_unsupported_patches(self):
        patch=np.full((9,9),6.);patch[2:7,2:7]=1.5
        self.assertAlmostEqual(H.robust_depth(patch,4,4,3,(.35,3.)),1.5)
        patch[:]=np.nan;patch[4,4]=1.5
        self.assertIsNone(H.robust_depth(patch,4,4,2,(.35,3.)))
        self.assertIsNone(H.robust_depth(np.full((9,9),6.),4,4,2,(.35,3.)))
        self.assertIsNone(H.robust_depth(patch,-1,4))

    def test_roi_keeps_wrist_and_image_bounds(self):
        from types import SimpleNamespace as S
        for x,y in ((.5,.5),(.01,.02),(.99,.99)):
            lm=[S(x=x,y=y) for _ in range(33)]
            rx,ry,rw,rh=H.hand_roi(lm,640,360)
            self.assertTrue(0 <= rx <= 640-rw);self.assertTrue(0 <= ry <= 360-rh)
            self.assertTrue(rx <= x*640 <= rx+rw);self.assertTrue(ry <= y*360 <= ry+rh)
            self.assertEqual(rw,rh)

    def test_neutral_palm_is_right_handed(self):
        r=H.palm_rotation(points());np.testing.assert_allclose(r,np.eye(3),atol=1e-12)
        self.assertAlmostEqual(np.linalg.det(r),1.)
    def test_each_axis_follows_known_rigid_rotation(self):
        for axis in np.eye(3):
            for angle in (-2.8,-.6,.6,math.pi):
                r=axis_rotation(axis,angle)
                np.testing.assert_allclose(H.palm_rotation(points(r)),r,atol=1e-12)
    def test_body_camera_rotation_is_removed(self):
        body=axis_rotation([0,1,0],.8)@axis_rotation([0,0,1],-.4)
        hand=axis_rotation([1,0,0],.3)@axis_rotation([0,0,1],.7)
        q=H.tool_orientation(points(body@hand),body)
        np.testing.assert_allclose(H.quaternion_matrix(q),hand@H.HAND_TO_FR3_TCP,atol=1e-12)
    def test_franka_tcp_approach_and_jaw_axes(self):
        r=H.quaternion_matrix(H.tool_orientation(points(),np.eye(3)))
        np.testing.assert_allclose(r[:,2],[1,0,0],atol=1e-12)  # TCP +z follows fingers
        np.testing.assert_allclose(r[:,1],[0,1,0],atol=1e-12)  # TCP +y spans palm
    def test_translation_does_not_rotate_hand(self):
        r=axis_rotation([0,1,0],.4)
        np.testing.assert_allclose(H.palm_rotation(points(r)),H.palm_rotation(points(r,np.array([.4,-.2,3.]))),atol=1e-12)
    def test_quaternion_half_turns_roundtrip(self):
        for axis in np.eye(3):
            r=axis_rotation(axis,math.pi);np.testing.assert_allclose(H.quaternion_matrix(H.matrix_quaternion(r)),r,atol=1e-12)
    def test_invalid_and_degenerate_depth_rejected(self):
        for p in [np.zeros((5,3)),np.full((5,3),np.nan),np.zeros((2,3)),points()*10]:
            with self.assertRaises(ValueError):H.palm_rotation(p)
        p=points();p[1,2]+=.4
        with self.assertRaises(ValueError):H.palm_rotation(p)
    def test_filter_rejects_pose_flip_and_timestamp_replay(self):
        f=H.OrientationFilter();f.update(np.array([0.,0.,0.,1.]),1.)
        with self.assertRaises(ValueError):f.update(np.array([0.,0.,0.,1.]),1.)
        with self.assertRaises(ValueError):f.update(np.array([1.,0.,0.,0.]),1.01)
    def test_quaternion_sign_does_not_create_flip(self):
        f=H.OrientationFilter();q=np.array([.2,0.,0.,math.sqrt(.96)])
        a=f.update(q,1.);b=f.update(-q,1.1);np.testing.assert_allclose(a,b,atol=1e-12)
    def test_filter_speed_cap_and_reacquisition(self):
        f=H.OrientationFilter(max_speed=math.radians(10))
        f.update(np.array([0.,0.,0.,1.]),1.)
        q=H.matrix_quaternion(axis_rotation([0,1,0],.3));out=f.update(q,1.1)
        self.assertLessEqual(2*math.acos(abs(out[3])),math.radians(1)+1e-10)
        out=f.update(q,2.);np.testing.assert_allclose(out,q,atol=1e-12)

if __name__=='__main__':unittest.main(verbosity=2)
