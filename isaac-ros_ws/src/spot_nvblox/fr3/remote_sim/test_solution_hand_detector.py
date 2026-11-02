import unittest
from types import SimpleNamespace as S
from unittest.mock import Mock
from solution_hand_detector import SolutionHandDetector

class DetectorTests(unittest.TestCase):
    def test_empty_frame_has_no_pose_or_gripper_intent(self):
        engine=Mock();engine.process.return_value=S(multi_hand_landmarks=None,multi_hand_world_landmarks=None,multi_handedness=None)
        out=SolutionHandDetector(engine).recognize_for_video(None,1)
        self.assertEqual(out.hand_landmarks,[]);self.assertEqual(out.gestures,[])

    def test_unmirrored_right_hand_preserves_metric_points_and_open_intent(self):
        points=[S(x=0.,y=0.,z=0.) for _ in range(21)]
        for finger in range(5):
            for joint in range(4):points[1+finger*4+joint]=S(x=.02+.03*joint,y=.01*finger,z=0.)
        engine=Mock();engine.process.return_value=S(multi_hand_landmarks=[S(landmark=points)],
            multi_hand_world_landmarks=[S(landmark=points)],multi_handedness=[S(classification=[S(label='Left',score=.9)])])
        out=SolutionHandDetector(engine).recognize_for_video(None,1)
        self.assertEqual(out.handedness[0][0].category_name,'Right')
        self.assertIs(out.hand_world_landmarks[0],points)
        self.assertEqual(out.gestures[0][0].category_name,'Open_Palm')

    def test_missing_metric_landmarks_never_invent_depth_or_intent(self):
        engine=Mock();engine.process.return_value=S(multi_hand_landmarks=[S(landmark=[])],multi_hand_world_landmarks=None,multi_handedness=[])
        self.assertEqual(SolutionHandDetector(engine).recognize_for_video(None,1).hand_world_landmarks,[])

class CropCoordinateTests(unittest.TestCase):
    def test_crop_landmarks_return_to_full_camera_coordinates(self):
        from types import SimpleNamespace
        from solution_hand_detector import remap_crop_landmarks
        point=SimpleNamespace(x=.5,y=.25,z=.02)
        result=SimpleNamespace(hand_landmarks=[[point]],hand_world_landmarks=['unchanged'])
        remap_crop_landmarks(result,(100,120,260,280),(360,640,3))
        self.assertAlmostEqual(point.x,180/640);self.assertAlmostEqual(point.y,160/360)
        self.assertEqual(point.z,.02);self.assertEqual(result.hand_world_landmarks,['unchanged'])
