"""Exercise real orientation callback with synthetic synchronized hand detections, no ROS/hardware."""
import ast
import importlib.util
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock
import sys
import unittest
import numpy as np
ROOT=Path(__file__).resolve().parents[5]
sys.path.insert(0,str(ROOT/'spot-ros2_ws/src/arm_pose_estimator'))
from arm_pose_estimator.hand_frame import tool_orientation, palm_rotation, quaternion_matrix, OrientationFilter, hand_roi, fingers_open, GestureOrientationLock
source=ROOT/'spot-ros2_ws/src/arm_pose_estimator/arm_pose_estimator/wrist_detector.py'
tree=ast.parse(source.read_text())
fn=next(n for c in tree.body if isinstance(c,ast.ClassDef) for n in c.body if isinstance(n,ast.FunctionDef) and n.name=='_hand_orientation_6d')
space=dict(np=np,Quaternion=lambda **kw:S(**kw),mp=S(solutions=S(hands=S(HAND_CONNECTIONS=[]))),
           tool_orientation=tool_orientation,palm_rotation=palm_rotation,quaternion_matrix=quaternion_matrix,
           SimpleNamespace=S,hand_roi=hand_roi,fingers_open=fingers_open)
exec(compile(ast.Module(body=[fn],type_ignores=[]),str(source),'exec'),space)
callback=space['_hand_orientation_6d']
class GeometryTests(unittest.TestCase):
    def make(self,rotation=np.eye(3)):
        p=np.array([[0.,0.,0.],[.03,-.07,0.],[.01,-.08,0.],[-.01,-.075,0.],[-.03,-.06,0.]])@rotation.T
        worlds=[S(x=0.,y=0.,z=0.) for _ in range(21)]
        for i,v in zip((0,5,9,13,17),p):worlds[i]=S(x=v[0],y=v[1],z=v[2])
        hand=S(landmark=[S(x=.5,y=.5) for _ in range(21)])
        result=S(multi_hand_landmarks=[hand],multi_hand_world_landmarks=[S(landmark=worlds)])
        n=S(gesture_recognizer=None,hand_6d=S(process=lambda rgb:result),last_valid_R=np.eye(3),hand_geometry_source='model',
            mp_drawing=S(draw_landmarks=Mock()),get_depth_at_pixel=Mock(return_value=2.),
            deproject_pixel_to_3d=lambda *a:np.array([0.,0.,2.]),orientation_filter=OrientationFilter(),
            project_3d_to_pixel=lambda p:None)
        landmarks=[S(x=.5,y=.5,visibility=1.) for _ in range(33)]
        return n,landmarks,result,p
    def call(self,n,l):return callback(n,None,np.ones((100,100)),l,None,1.)
    def test_model_uses_wrist_depth_without_small_mcp_depth_patches(self):
        n,l,_,p=self.make();self.assertTrue(self.call(n,l))
        self.assertEqual(n.get_depth_at_pixel.call_count,1)
        expected=tool_orientation(p+[0.,0.,2.],np.eye(3))
        actual=np.array([n.hand_quat.x,n.hand_quat.y,n.hand_quat.z,n.hand_quat.w])
        self.assertAlmostEqual(abs(float(actual@expected)),1.)
    def test_pitch_and_yaw_change_reported_orientation(self):
        n,l,_,_=self.make();self.call(n,l);base=np.array([n.hand_quat.x,n.hand_quat.y,n.hand_quat.z,n.hand_quat.w])
        for r in (np.array([[1,0,0],[0,.8,-.6],[0,.6,.8]]),np.array([[.8,-.6,0],[.6,.8,0],[0,0,1]])):
            n,l,_,_=self.make(r);self.assertTrue(self.call(n,l))
            actual=np.array([n.hand_quat.x,n.hand_quat.y,n.hand_quat.z,n.hand_quat.w])
            self.assertGreater(2*np.arccos(min(1.,abs(float(actual@base)))),np.radians(10))
    def test_replayed_capture_never_reaches_video_recognizer(self):
        n,l,_,_=self.make();n.gesture_recognizer=Mock();n.hand_capture_ms=1000
        self.assertFalse(self.call(n,l));n.gesture_recognizer.recognize_for_video.assert_not_called()
    @unittest.skipUnless(importlib.util.find_spec('mediapipe'), 'run in perception container for MediaPipe/protobuf adapter')
    def test_cropped_task_result_returns_original_pixel_coordinates(self):
        import mediapipe as real_mp
        import cv2
        previous=space['mp'];space['mp']=real_mp;space['cv2']=cv2
        try:
            n,l,result,_=self.make()
            for point in result.multi_hand_landmarks[0].landmark:point.z=0.
            detection=S(hand_landmarks=[result.multi_hand_landmarks[0].landmark],
                        hand_world_landmarks=[result.multi_hand_world_landmarks[0].landmark],
                        gestures=[[S(category_name='Open_Palm',score=.9)]])
            received=[]
            def recognize(image,stamp):
                received.append((image.numpy_view().shape,stamp));return detection
            n.gesture_recognizer=S(recognize_for_video=recognize)
            n.gesture_orientation_lock=GestureOrientationLock()
            n.hand_capture_ms=-1;n.use_hand_roi=True
            self.assertTrue(callback(n,np.zeros((360,640,3),dtype=np.uint8),np.ones((360,640))*2,l,None,1.))
            self.assertEqual(received,[((256,256,3),1000)])
            self.assertEqual(n.hand_wrist_pixel,(320,180))
            self.assertEqual(n.hand_gesture,'Open_Palm')
        finally:space['mp']=previous

    def test_missing_world_hand_or_wrist_depth_is_not_fresh(self):
        n,l,result,_=self.make();result.multi_hand_world_landmarks=[];self.assertFalse(self.call(n,l))
        n,l,_,_=self.make();n.get_depth_at_pixel.return_value=None;self.assertFalse(self.call(n,l))
if __name__=='__main__':unittest.main(verbosity=2)
