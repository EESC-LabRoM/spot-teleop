import time,copy,unittest
import numpy as np
from command_guard import validate,ARM
class CommandGuardTests(unittest.TestCase):
 def setUp(self):
  self.now=time.time_ns();self.q=[0.,-.6,0.,-2.,0.,1.5,0.]
  self.p=dict(version=1,frame='fr3_link0',names=ARM,q=self.q.copy(),v=[0.]*7,feasible=True,grid_covers_robot=True,
   **{k:self.now-1000000 for k in ('stamp_ns','goal_stamp_ns','joint_stamp_ns','esdf_stamp_ns','depth_stamp_ns')})
 def test_current_feasible_hold(self):self.assertEqual(validate(self.p,self.q,self.now)[0],self.q)
 def test_each_stale_dependency_holds(self):
  for key in ('stamp_ns','goal_stamp_ns','joint_stamp_ns','esdf_stamp_ns','depth_stamp_ns'):
   p=copy.deepcopy(self.p);p[key]=self.now-2000000000
   with self.assertRaises(ValueError,msg=key):validate(p,self.q,self.now)
 def test_infeasible_or_uncovered_holds(self):
  for key in ('feasible','grid_covers_robot'):
   p=copy.deepcopy(self.p);p[key]=False
   with self.assertRaises(ValueError):validate(p,self.q,self.now)
 def test_large_first_command_does_not_seed_history(self):
  self.p['q'][0]=.02
  with self.assertRaises(ValueError):validate(self.p,self.q,self.now)
 def test_velocity_acceleration_and_joint_limits(self):
  for key,value in [('v',[1.3]*7),('q',[10.]*7),('q',[float('nan')]*7)]:
   p=copy.deepcopy(self.p);p[key]=value
   with self.assertRaises(ValueError):validate(p,self.q,self.now)
  old=copy.deepcopy(self.p);old['stamp_ns']-=20000000;self.p['v'][0]=.5
  with self.assertRaises(ValueError):validate(self.p,self.q,self.now,old)
 def test_replay_is_rejected(self):
  with self.assertRaises(ValueError):validate(self.p,self.q,self.now,self.p)
 def test_small_fresh_increment(self):
  old=copy.deepcopy(self.p);old['stamp_ns']-=20000000;self.p['q'][0]+=.002;self.p['v'][0]=.1
  self.assertAlmostEqual(validate(self.p,self.q,self.now,old)[0][0],.002)
 def test_wrong_joint_order_holds(self):
  self.p['names']=ARM[::-1]
  with self.assertRaises(ValueError):validate(self.p,self.q,self.now)
if __name__=='__main__':unittest.main()
