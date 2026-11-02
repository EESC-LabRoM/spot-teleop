import math,unittest
import numpy as np
from cartesian_review import target,PROFILES
from teleop_mapping import _quat_mul
class CartesianReviewTests(unittest.TestCase):
 def setUp(self):self.p=np.array([.3,.1,.48]);self.q=np.array([1.,0,0,0]);self.h=np.zeros(3);self.hq=np.array([0.,0,0,1.])
 def run_target(self,dp,profile,q=None):return target(self.p,self.q,self.h,self.hq,dp,self.hq if q is None else q,[1,1,1],1,profile,deadband=0)
 def test_horizontal_straight_line_no_base_arc_or_rotation(self):
  p,q=self.run_target([.08,.1,.09],'horizontal');np.testing.assert_allclose(p,self.p+[0,.1,0]);np.testing.assert_allclose(q,self.q)
 def test_depth_changes_height_only(self):
  p,q=self.run_target([.1,.08,.09],'height');np.testing.assert_allclose(p,self.p+[0,0,-.1]);np.testing.assert_allclose(q,self.q)
 def test_vertical_changes_reach_only(self):
  p,q=self.run_target([.08,.09,.1],'reach');np.testing.assert_allclose(p,self.p+[.1,0,0]);np.testing.assert_allclose(q,self.q)
 def test_rotation_profiles_never_move_tcp(self):
  for profile in ['roll','pitch','yaw']:
   p,q=self.run_target([.1,.1,.1],profile,[.2,.1,.2,math.sqrt(.91)]);np.testing.assert_allclose(p,self.p)
 def test_neutral_and_gripper_reject_all_pose_motion(self):
  for profile in ['neutral','gripper']:
   p,q=self.run_target([.1,.1,.1],profile,[0,.3,0,math.sqrt(.91)]);np.testing.assert_allclose(p,self.p);np.testing.assert_allclose(q,self.q)
 def test_roll_only_rotates_tool_z(self):
  p,q=self.run_target([.1,.1,.1],'roll',[-math.sin(.3),0,0,math.cos(.3)])
  np.testing.assert_allclose(p,self.p);relative=_quat_mul([-self.q[0],0,0,self.q[3]],q);self.assertAlmostEqual(relative[0],0);self.assertAlmostEqual(relative[1],0)
 def test_cross_leaves_height_fixed(self):
  p,q=self.run_target([.1,.1,.1],'cross');self.assertEqual(p[2],self.p[2])
 def test_combined_rotation_matches_each_isolated_approved_axis(self):
  for profile,axis,gain in [('pitch',[0,1,0],1.2),('yaw',[0,0,1],1.2),('roll',[-1,0,0],1.6)]:
   for sign in [-1,1]:
    q=np.r_[np.array(axis)*math.sin(sign*.2),math.cos(sign*.2)]
    a=target(self.p,self.q,self.h,self.hq,self.h,q,[1,1,1],1.,'full_6d')[1]
    b=target(self.p,self.q,self.h,self.hq,self.h,q,[1,1,1],gain,profile)[1]
    np.testing.assert_allclose(a,b,atol=1e-7)
 def test_combined_neutral_preserves_anchor_pose(self):
  p,q=self.run_target([0,0,0],'full_6d');np.testing.assert_allclose(p,self.p);np.testing.assert_allclose(q,self.q)
 def test_deadband_and_caps(self):
  p,q=target(self.p,self.q,self.h,self.hq,[.001,.001,.001],self.hq,[1,1,1],1,'free');np.testing.assert_allclose(p,self.p)
  p,q=self.run_target([2,2,2],'free');self.assertLessEqual(float(np.max(np.abs(p-self.p))),.200001)
if __name__=='__main__':unittest.main()
