import math,unittest
import numpy as np
from scipy.spatial.transform import Rotation as R
from rate_yaw import RateYaw
from table_facing import table_tilt_deg
class RateTests(unittest.TestCase):
 def test_hold_twist_then_neutral(self):
  h=R.identity().as_quat();q=R.from_euler('x',180,degrees=True).as_quat();r=RateYaw(h,q);held=R.from_euler('x',30,degrees=True).as_quat()
  for _ in range(200):q=r.step(held,q,.02)
  self.assertGreater(abs(r.yaw),math.radians(150));yaw=r.yaw
  for _ in range(100):q=r.step(h,q,.02)
  self.assertAlmostEqual(r.yaw,yaw);self.assertLess(table_tilt_deg(q),1e-5)
 def test_blocked_robot_limits_accumulation(self):
  h=R.identity().as_quat();q=R.from_euler('x',180,degrees=True).as_quat();r=RateYaw(h,q)
  for _ in range(500):r.step(R.from_euler('x',30,degrees=True).as_quat(),q,.02)
  self.assertLessEqual(abs(r.yaw),math.radians(5)+1e-8)
  r.step(h,q,.02);self.assertAlmostEqual(r.yaw,0)
 def test_sign_swing_and_acceleration(self):
  h=R.identity().as_quat();q=R.from_euler('x',180,degrees=True).as_quat()
  for sign in [-1,1]:
   r=RateYaw(h,q);prev=0
   for _ in range(20):
    r.step(R.from_euler('x',sign*30,degrees=True).as_quat(),q,.02)
    self.assertLessEqual(abs(r.rate-prev),math.radians(90)*.02+1e-9);prev=r.rate
   self.assertEqual(math.copysign(1,r.rate),sign)
  r=RateYaw(h,q);r.step(R.from_euler('y',30,degrees=True).as_quat(),q,.02);self.assertAlmostEqual(r.rate,0)
 def test_antipodal_pause_and_bad_dt(self):
  h=R.identity().as_quat();q=R.from_euler('x',180,degrees=True).as_quat();r=RateYaw(h,q)
  r.step(-h,q,.02);self.assertAlmostEqual(r.rate,0);r.pause();self.assertEqual(r.rate,0)
  for dt in [0,-1,float('nan'),.2]:
   with self.assertRaises(ValueError):r.step(h,q,dt)
if __name__=='__main__':unittest.main()
