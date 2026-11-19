import unittest
from server import audit
class AuditTests(unittest.TestCase):
 def live(self):return dict(coverage_pct=1.,mask_age_s=.3)
 def filt(self,stamp=100.,wall=100.):return dict(wall=wall,recent=dict(stamp=stamp,masked_nonzero=0,outside_changed=0))
 def test_old_accepted_frame_with_live_heartbeat_is_not_clean(self):self.assertEqual(audit(self.filt(stamp=1),self.live(),100.2)['leak_check'],'NOT VERIFIED')
 def test_empty_live_mask_is_not_clean(self):self.assertEqual(audit(self.filt(),dict(coverage_pct=0,mask_age_s=.2),100.2)['leak_check'],'NOT VERIFIED')
 def test_stale_heartbeat_is_not_clean(self):self.assertEqual(audit(self.filt(wall=90),self.live(),100.2)['leak_check'],'NOT VERIFIED')
 def test_future_frame_is_not_clean(self):self.assertEqual(audit(self.filt(stamp=105),self.live(),100.2)['leak_check'],'NOT VERIFIED')
 def test_current_exclusion_with_background_preserved_is_verified(self):self.assertEqual(audit(self.filt(),self.live(),100.2)['leak_check'],'Input exclusion verified')
 def test_nonzero_inside_exclusion_cannot_pass(self):
  f=self.filt();f['recent']['masked_nonzero']=1;self.assertEqual(audit(f,self.live(),100.2)['leak_check'],'NOT VERIFIED')
 def test_outside_change_cannot_pass(self):
  f=self.filt();f['recent']['outside_changed']=1;self.assertEqual(audit(f,self.live(),100.2)['leak_check'],'NOT VERIFIED')
if __name__=='__main__':unittest.main()
