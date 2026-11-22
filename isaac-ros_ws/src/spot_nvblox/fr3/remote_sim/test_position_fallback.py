"""Run in fr3-curobo, isolated domain 193: synthetic RGB-D, no command publisher."""
import json,time,unittest,tempfile
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock
import numpy as np
import rclpy
from sensor_msgs.msg import Image
from operator_hand_tracker import OperatorHandTracker

class WristFallbackTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):rclpy.init(domain_id=193)
    @classmethod
    def tearDownClass(cls):rclpy.shutdown()
    def setUp(self):
        self.n=OperatorHandTracker(calibrate_arm=True,output_prefix='/isolated_wrist_test')
        self.n.detector.close();self.n.detector=Mock()
        self.n.detector.restarts=0
        self.n.crop_detector=Mock();self.n.crop_detector.recognize_for_video.return_value=NS(hand_landmarks=[])
        self.n.detector.recognize_for_video.return_value=NS(hand_landmarks=[],gestures=[],handedness=[])
        self.n.body.close();self.n.body=Mock()
        landmarks=[NS(x=.5,y=.3,visibility=.95) for _ in range(33)]
        landmarks[14]=NS(x=.55,y=.45,visibility=.95)
        landmarks[16]=NS(x=.6,y=.6,visibility=.95)
        self.n.body.process.return_value=NS(pose_landmarks=NS(landmark=landmarks))
        self.n.sample_pub=Mock();self.n.diag_pub=Mock()
        self.n.k=np.array([[400.,0.,320.],[0.,400.,180.],[0.,0.,1.]])
        self.n.arm.scale=1.5;self.n.arm.length=.6;self.n.arm.shoulder=np.array([0.,-.18,1.])
        self.n.hand_identity=(np.array([.16,.09,1.]),time.time())
    def tearDown(self):self.n.destroy_node()
    def frame(self,depth_valid=True):
        stamp=time.time_ns()
        rgb=Image();rgb.width=640;rgb.height=360;rgb.step=640*3;rgb.encoding='bgr8'
        rgb.header.stamp.sec,rgb.header.stamp.nanosec=divmod(stamp,10**9)
        rgb.data=np.zeros((360,640,3),np.uint8).tobytes()
        depth=Image();depth.width=640;depth.height=360;depth.step=640*4;depth.encoding='32FC1'
        depth.data=np.full((360,640),1. if depth_valid else np.nan,np.float32).tobytes()
        self.n.frame(rgb,depth)
        return stamp
    def hand_at(self,x,y):
        self.n.detector.recognize_for_video.return_value=NS(
            hand_landmarks=[[NS(x=x,y=y) for _ in range(21)]],hand_world_landmarks=[],
            handedness=[[NS(score=.99,category_name='Right')]],gestures=[[]])
    def diag(self):return json.loads(self.n.diag_pub.publish.call_args.args[0].data)
    def test_body_wrist_never_drives_and_alerts_instead(self):
        # Pose sees the wrist (visibility .95, current depth) but Hands does not: no command, a visibility alert.
        stamp=self.frame();self.n.sample_pub.publish.assert_not_called()
        d=self.diag();self.assertEqual(d['stamp_ns'],stamp);self.assertFalse(d['valid'])
        self.assertEqual(d['missing'],'fingers');self.assertTrue(d['alert'])
        # ...but the arm's current wrist keeps the hand identity moving, so the hand re-associates where it reappears.
        self.assertTrue(d['identity_from_body']);self.assertAlmostEqual(self.n.hand_identity[1],stamp*1e-9)
        np.testing.assert_allclose(self.n.hand_identity[0],[.16,.09,1.],atol=1e-6)
        self.frame();self.assertEqual(self.n.body.process.call_count,2)   # pose still runs: association stays current
    def test_hand_outside_the_image_is_named(self):
        self.n.body.process.return_value.pose_landmarks.landmark[16].visibility=.1
        self.frame();self.n.sample_pub.publish.assert_not_called();self.assertEqual(self.diag()['missing'],'hand')
    def test_tracked_6d_hand_has_no_alert(self):
        self.n.palm_model=NS(template=object(),update=Mock(return_value=(np.eye(3),dict(orientation_inliers=5,orientation_fit_mm=1.))))
        hand=[NS(x=.6,y=.6) for _ in range(21)];world=[NS(x=0.,y=0.,z=0.) for _ in range(21)]
        self.n.detector.recognize_for_video.return_value=NS(hand_landmarks=[hand],
            hand_world_landmarks=[world],handedness=[[NS(score=.99,category_name='Right')]],gestures=[[]])
        self.frame()
        packet=json.loads(self.n.sample_pub.publish.call_args.args[0].data)
        self.assertTrue(packet['orientation_valid']);self.assertIsNone(self.diag()['alert'])
    def test_bad_palm_keeps_current_hand_wrist_translation(self):
        hand=[NS(x=.6,y=.6) for _ in range(21)]
        world=[NS(x=0.,y=0.,z=0.) for _ in range(21)]
        self.n.detector.recognize_for_video.return_value=NS(hand_landmarks=[hand],
            hand_world_landmarks=[world],handedness=[[NS(score=.99,category_name='Right')]],gestures=[[]])
        self.n.palm_model=NS(template=object(),update=Mock(side_effect=ValueError('bad palm orientation')))
        self.frame()
        self.n.palm_model.update.assert_called_once()
        packet=json.loads(self.n.sample_pub.publish.call_args.args[0].data)
        self.assertEqual(packet['position_source'],'hand_wrist');self.assertFalse(packet['orientation_valid'])
        self.assertEqual(self.diag()['missing'],'palm')

    def test_current_hand_wrist_does_not_require_world_orientation_landmarks(self):
        self.n.detector.recognize_for_video.return_value=NS(
            hand_landmarks=[[NS(x=.6,y=.6) for _ in range(21)]],hand_world_landmarks=[],
            handedness=[[NS(score=.99,category_name='Right')]],gestures=[[]])
        self.n.body.process.return_value=NS(pose_landmarks=None)
        self.frame()
        packet=json.loads(self.n.sample_pub.publish.call_args.args[0].data)
        self.assertEqual(packet['position_source'],'hand_wrist');self.assertFalse(packet['orientation_valid'])

    def test_weak_pose_wrist_without_arm_support_does_not_move(self):
        points=self.n.body.process.return_value.pose_landmarks.landmark
        points[16].visibility=.4;points[12].visibility=.2
        self.frame();self.n.sample_pub.publish.assert_not_called()

    def test_reload_preserves_recent_measured_scale_but_does_not_invent_wrist(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'arm.json';self.n.arm_reload_path=path
            state=dict(saved_at=time.time(),length=.5,shoulder=[0.,-.18,1.],k=self.n.k.tolist())
            path.write_text(json.dumps(state));self.n.k=None;self.n.arm.reset()
            self.n.info(NS(k=np.asarray(state['k']).reshape(-1).tolist()))
            self.assertAlmostEqual(self.n.arm.scale,.855/.5)
            self.assertIsNone(self.n.arm.wrist_reference);self.n.sample_pub.publish.assert_not_called()
            self.assertFalse(path.exists())
            self.n.arm_reload_path=None

    def test_stale_scale_snapshot_is_not_restored(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'arm.json';self.n.arm_reload_path=path
            state=dict(saved_at=time.time()-121.,length=.5,shoulder=[0.,-.18,1.],k=self.n.k.tolist())
            path.write_text(json.dumps(state));self.n.k=None;self.n.arm.reset()
            self.n.info(NS(k=np.asarray(state['k']).reshape(-1).tolist()))
            self.assertIsNone(self.n.arm.scale);self.n.arm_reload_path=None

    def test_current_shoulder_can_reassociate_a_detected_right_hand_without_pose_wrist(self):
        self.n.hand_identity=None;self.n.arm.wrist_reference=None;self.n.arm.reference_stamp=None
        self.n.body.process.return_value.pose_landmarks.landmark[16].visibility=.1
        self.n.detector.recognize_for_video.return_value=NS(
            hand_landmarks=[[NS(x=.6,y=.6) for _ in range(21)]],hand_world_landmarks=[],
            handedness=[[NS(score=.99,category_name='Right')]],gestures=[[]])
        self.frame()
        packet=json.loads(self.n.sample_pub.publish.call_args.args[0].data)
        self.assertEqual(packet['position_source'],'hand_wrist')

    def test_current_full_arm_can_reassociate_after_operator_stands_up(self):
        self.n.hand_identity=None;self.n.arm.shoulder=np.array([0.,-.8,1.])
        self.n.arm.length=.48
        points=self.n.body.process.return_value.pose_landmarks.landmark
        points[12].x=.5;points[12].y=.2
        points[14].x=.6;points[14].y=.4
        points[16].x=.5;points[16].y=.6
        self.hand_at(.5,.6)   # the full current arm re-associates the hand found at its wrist
        self.frame()
        packet=json.loads(self.n.sample_pub.publish.call_args.args[0].data)
        self.assertEqual(packet['position_source'],'hand_wrist')

    def test_no_depth_does_not_publish_position(self):
        self.frame(False);self.n.sample_pub.publish.assert_not_called()
    def test_no_person_calibration_does_not_publish_position(self):
        self.n.arm.scale=None;self.frame();self.n.sample_pub.publish.assert_not_called()
    def test_old_pose_is_not_promoted_when_current_body_is_absent(self):
        self.frame();self.n.sample_pub.publish.reset_mock()
        self.n.body.process.return_value=NS(pose_landmarks=None)
        self.frame();self.n.sample_pub.publish.assert_not_called()
    def test_reacquisition_needs_current_calibrated_operator(self):
        self.n.hand_identity=(np.array([.16,.09,1.]),time.time()-10.);self.hand_at(.6,.6)
        self.frame();self.assertTrue(self.n.sample_pub.publish.called)
        self.n.sample_pub.publish.reset_mock();self.n.hand_identity=None
        self.n.arm.shoulder=np.array([2.,2.,2.])
        self.frame();self.n.sample_pub.publish.assert_not_called()

if __name__=='__main__':unittest.main()
