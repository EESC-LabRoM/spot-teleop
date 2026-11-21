import unittest
from dataclasses import replace
import numpy as np
from swept_esdf import Evidence,REQUIRED_STAGES,validate

class Tests(unittest.TestCase):
 def setUp(self):
  self.sphere=np.array([[.5,.0,.3,.03]])
  self.e=Evidence({s:[self.sphere] for s in REQUIRED_STAGES},.04,'ball','ball',True,True,True)
  self.query=lambda p:(np.full(len(p),.2),np.ones(len(p),bool))
 def check(self,e=None,query=None):return validate(e or self.e,query or self.query,clearance_m=.01,table_top_m=.029)
 def test_complete_valid_evidence(self):self.assertTrue(self.check()[0])
 def test_obstacle_in_intermediate_stage_refused(self):
  seen=[]
  def q(p):
   seen.append(1);return (np.full(len(p),.035 if len(seen)==2 else .2),np.ones(len(p),bool))
  ok,reason=self.check(query=q);self.assertFalse(ok);self.assertIn('S2_pre_to_grasp',reason)
 def test_unknown_never_free(self):
  ok,reason=self.check(query=lambda p:(np.ones(len(p)),np.zeros(len(p),bool)));self.assertFalse(ok);self.assertIn('unknown',reason)
 def test_absent_object_model_refused(self):self.assertFalse(self.check(replace(self.e,carried_stages_include_object=False))[0])
 def test_non_targets_and_identity_required(self):
  self.assertFalse(self.check(replace(self.e,other_obstacles_present=False))[0]);self.assertFalse(self.check(replace(self.e,map_target='drill'))[0])
 def test_near_table_and_missing_stage_refused(self):
  s=dict(self.e.stages);s['S3_lift']=[np.array([[.5,0,.06,.03]])];self.assertFalse(self.check(replace(self.e,stages=s))[0]);s.pop('S3_lift');self.assertFalse(self.check(replace(self.e,stages=s))[0])
 def test_sampling_step_bounded(self):self.assertFalse(self.check(replace(self.e,max_joint_step=.051))[0])
if __name__=='__main__':unittest.main()
