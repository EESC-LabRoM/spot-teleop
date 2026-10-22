"""Synthetic RGB-D through the actual perception callback, DDS 191, mocked outputs.
Run in fr3-curobo with the Humble/CV/MediaPipe environment. No camera or robot needed.
"""
import json
import sys
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock
import unittest
import numpy as np
import rclpy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from mediapipe.framework.formats import landmark_pb2
ROOT=Path(__file__).resolve().parents[5]
sys.path.insert(0,str(ROOT/'spot-ros2_ws/src/arm_pose_estimator'))
from arm_pose_estimator.wrist_detector import WristDetector
from arm_pose_estimator.hand_frame import hand_roi, quaternion_matrix

class PipelineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(domain_id=191,args=['--ros-args','--params-file',str(ROOT/'isaac-ros_ws/src/spot_nvblox/fr3/zed3/fr3_operator_wrist.yaml'),'-p','online_scale_estimation:=false'])
    @classmethod
    def tearDownClass(cls):rclpy.shutdown()
    def setUp(self):
        self.n=WristDetector();n=self.n
        n.pose.close();n.gesture_recognizer.close()
        self.pose=landmark_pb2.NormalizedLandmarkList(landmark=[landmark_pb2.NormalizedLandmark(x=.5,y=.8,z=0.,visibility=1.) for _ in range(33)])
        for i,(x,y) in {11:(.54,.35),12:(.46,.35),14:(.42,.4),16:(.39,.45),20:(.38,.42)}.items():
            self.pose.landmark[i].x=x;self.pose.landmark[i].y=y
        n.pose=S(process=lambda rgb:S(pose_landmarks=self.pose),close=lambda:None)
        self.gesture='Open_Palm';self.rotation=np.eye(3);self.recognized=[]
        def recognize(image,stamp):
            self.recognized.append((image.numpy_view().shape,stamp))
            x,y,w,h=hand_roi(self.pose.landmark,640,360)
            pixel=np.array([int(self.pose.landmark[16].x*640)+6,int(self.pose.landmark[16].y*360)])
            self.expected_pixel=tuple(pixel)
            lm=[S(x=(pixel[0]-x)/w,y=(pixel[1]-y)/h,z=0.) for _ in range(21)]
            palm=np.array([[0.,0.,0.],[.03,-.07,0.],[.01,-.08,0.],[-.01,-.075,0.],[-.03,-.06,0.]])@self.rotation.T
            world=[S(x=0.,y=0.,z=0.) for _ in range(21)]
            for i,p in zip((0,5,9,13,17),palm):world[i]=S(x=p[0],y=p[1],z=p[2])
            return S(hand_landmarks=[lm],hand_world_landmarks=[world],gestures=[[S(category_name=self.gesture,score=.95)]])
        n.gesture_recognizer=S(recognize_for_video=recognize,close=lambda:None)
        n.wrist_pose_pub=Mock();n.teleop_sample_pub=Mock();n.tf_broadcaster=S(sendTransform=Mock())
        n.overlay_pub=None;n.show_all_landmarks=False
        info=CameraInfo();info.width=640;info.height=360;info.k=[260.,0.,320.,0.,260.,180.,0.,0.,1.]
        n.camera_info_callback(info)
        self.depth=np.full((360,640),6.,dtype=np.float32)
        for i in (11,12,16):
            x,y=int(self.pose.landmark[i].x*640),int(self.pose.landmark[i].y*360)
            self.depth[y-15:y+16,x-15:x+16]=2.
        self.capture_ns=100*10**9
        n.get_clock=lambda:S(now=lambda:Time(nanoseconds=self.capture_ns+10**7))
    def tearDown(self):self.n.destroy_node()
    def frame(self,step=.1):
        self.capture_ns+=int(step*1e9)
        color=self.n.bridge.cv2_to_imgmsg(np.zeros((360,640,3),dtype=np.uint8),'bgr8')
        depth=self.n.bridge.cv2_to_imgmsg(self.depth,'32FC1')
        color.header.stamp=Time(nanoseconds=self.capture_ns).to_msg();depth.header.stamp=color.header.stamp
        self.n.synced_callback(color,depth)
    def test_hidden_hips_do_not_drop_valid_shoulders_and_precise_wrist(self):
        self.frame();n=self.n
        n.wrist_pose_pub.publish.assert_called_once()
        self.assertEqual(self.recognized[0][0],(256,256,3))
        self.assertEqual(n.hand_wrist_pixel,self.expected_pixel)
        np.testing.assert_allclose(n.last_valid_R.T@n.last_valid_R,np.eye(3),atol=1e-6)
        pose=n.wrist_pose_pub.publish.call_args[0][0]
        p=n.deproject_pixel_to_3d(*self.expected_pixel,2.)
        expected=n.last_valid_R.T@(p-n.last_valid_origin)*n.scale_factor+n.shoulder_offset
        np.testing.assert_allclose([pose.pose.position.x,pose.pose.position.y,pose.pose.position.z],expected,atol=1e-6)
        packet=json.loads(n.teleop_sample_pub.publish.call_args[0][0].data)
        self.assertEqual(packet['stamp_ns'],self.capture_ns);self.assertEqual(packet['gesture'],'Open_Palm')
    def test_missing_wrist_depth_never_relabels_cached_pose_fresh(self):
        self.frame();self.n.wrist_pose_pub.publish.reset_mock()
        self.depth[:]=6.;self.frame()
        self.n.wrist_pose_pub.publish.assert_not_called()
    def test_replayed_capture_never_reaches_pose_or_hand_inference(self):
        self.frame();calls=len(self.recognized);self.n.wrist_pose_pub.publish.reset_mock()
        self.frame(step=0.)
        self.assertEqual(len(self.recognized),calls);self.n.wrist_pose_pub.publish.assert_not_called()
    def test_closing_fist_freezes_orientation_with_fresh_gesture_packet(self):
        self.frame();n=self.n
        first=json.loads(n.teleop_sample_pub.publish.call_args[0][0].data)
        self.gesture='Closed_Fist'
        a=.8;self.rotation=np.array([[1.,0.,0.],[0.,np.cos(a),-np.sin(a)],[0.,np.sin(a),np.cos(a)]])
        self.frame()
        second=json.loads(n.teleop_sample_pub.publish.call_args[0][0].data)
        self.assertGreater(second['stamp_ns'],first['stamp_ns']);self.assertEqual(second['gesture'],'Closed_Fist')
        np.testing.assert_allclose(quaternion_matrix(first['q']),quaternion_matrix(second['q']),atol=1e-6)

if __name__=='__main__':unittest.main(verbosity=2)
