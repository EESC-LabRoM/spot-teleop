import math
import unittest
import numpy as np
from manual_teaching import manual_target,ManualKinematics,quaternion_rotation
from pathlib import Path
from home_pose import HOME_Q,HOME_TCP_P,HOME_TCP_Q


class ManualTeachingTests(unittest.TestCase):
    def test_urdf_fk_matches_measured_mujoco_home(self):
        model=ManualKinematics(Path(__file__).parent.parent/'fr3.urdf')
        p,r,j=model.forward(HOME_Q)
        np.testing.assert_allclose(p,HOME_TCP_P,atol=1e-6)
        np.testing.assert_allclose(r,quaternion_rotation(HOME_TCP_Q),atol=1e-6)

    def test_inverse_kinematics_reaches_manual_steps(self):
        model=ManualKinematics(Path(__file__).parent.parent/'fr3.urdf')
        for axis,amount in [('height',.02),('reach',.02),('base',.1),('roll',.1),('pitch',-.1)]:
            with self.subTest(axis=axis):
                p,q=manual_target(HOME_TCP_P,HOME_TCP_Q,axis,amount)
                joints=model.solve(p,q,HOME_Q);actual,orientation,_=model.forward(joints)
                self.assertLess(np.linalg.norm(actual-p),.0003)
                np.testing.assert_allclose(orientation,quaternion_rotation(q),atol=.001)
    def test_reach_changes_radial_distance_and_keeps_height(self):
        p,q=manual_target([.3,.4,.4],[0,0,0,1],'reach',.02)
        self.assertAlmostEqual(np.linalg.norm(p[:2]),.52)
        self.assertAlmostEqual(p[2],.4)
        np.testing.assert_allclose(q,[0,0,0,1])

    def test_base_turn_rotates_position_and_orientation(self):
        p,q=manual_target([.4,0,.4],[0,0,0,1],'base',math.radians(10))
        np.testing.assert_allclose(p,[.4*math.cos(math.radians(10)),.4*math.sin(math.radians(10)),.4])
        np.testing.assert_allclose(q,[0,0,math.sin(math.radians(5)),math.cos(math.radians(5))])

    def test_roll_keeps_position(self):
        p,q=manual_target([.4,0,.4],[0,0,0,1],'roll',math.radians(-10))
        np.testing.assert_allclose(p,[.4,0,.4]);self.assertAlmostEqual(np.linalg.norm(q),1)
        self.assertLess(q[2],0)

    def test_height_step_and_limits(self):
        p,_=manual_target([.4,0,.4],[0,0,0,1],'height',-.02)
        self.assertAlmostEqual(p[2],.38)
        for axis,amount in [('height',float('nan')),('reach',.2),('base',1.),('unknown',.01)]:
            with self.assertRaises(ValueError):manual_target([.4,0,.4],[0,0,0,1],axis,amount)
        with self.assertRaises(ValueError):manual_target([.4,0,.79],[0,0,0,1],'height',.05)


if __name__=='__main__':unittest.main()
