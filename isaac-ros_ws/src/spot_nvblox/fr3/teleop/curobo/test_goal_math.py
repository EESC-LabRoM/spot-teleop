import unittest,math
from goal_math import tcp_to_hand
class TransformTests(unittest.TestCase):
 def test_hand_offset_follows_rotated_tool_z(self):
  p,q=tcp_to_hand([.4,0,.5],[0,math.sqrt(.5),0,math.sqrt(.5)],[0,0,-.1034],[0,0,0,1])
  self.assertAlmostEqual(p[0],.2966);self.assertAlmostEqual(p[2],.5)
 def test_identity_transform_preserves_pose(self):
  self.assertEqual(tcp_to_hand([1,2,3],[0,0,0,1],[0,0,0],[0,0,0,1]),([1.,2.,3.],[0.,0.,0.,1.]))
 def test_invalid_quaternion_is_rejected(self):
  with self.assertRaises(ValueError):tcp_to_hand([1,2,3],[0,0,0,0],[0,0,0],[0,0,0,1])
