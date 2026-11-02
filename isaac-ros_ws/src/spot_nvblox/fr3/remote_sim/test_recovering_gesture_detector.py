import unittest
from unittest.mock import Mock
from recovering_gesture_detector import RecoveringGestureDetector

class RecoveryTests(unittest.TestCase):
    def test_failed_graph_never_returns_cached_pose_and_recovers_with_new_frame(self):
        now=[0.];broken=Mock();broken.recognize_for_video.side_effect=RuntimeError('Graph has errors')
        fresh=Mock();fresh.recognize_for_video.return_value='fresh sample'
        factory=Mock(side_effect=[broken,fresh])
        d=RecoveringGestureDetector(factory=factory,clock=lambda:now[0])
        with self.assertRaises(RuntimeError):d.recognize_for_video('first',1)
        broken.close.assert_called_once()
        with self.assertRaises(RuntimeError):d.recognize_for_video('too soon',2)
        self.assertEqual(factory.call_count,1)
        now[0]=1.1
        self.assertEqual(d.recognize_for_video('current',3),'fresh sample')
        fresh.recognize_for_video.assert_called_once_with('current',3)
        self.assertEqual(d.restarts,1)

    def test_failed_recreation_is_rate_limited(self):
        now=[0.];broken=Mock();broken.recognize_for_video.side_effect=RuntimeError('broken')
        factory=Mock(side_effect=[broken,ValueError('unavailable')])
        d=RecoveringGestureDetector(factory=factory,clock=lambda:now[0])
        with self.assertRaises(RuntimeError):d.recognize_for_video(None,1)
        now[0]=2.
        with self.assertRaises(RuntimeError):d.recognize_for_video(None,2)
        with self.assertRaises(RuntimeError):d.recognize_for_video(None,3)
        self.assertEqual(factory.call_count,2)
