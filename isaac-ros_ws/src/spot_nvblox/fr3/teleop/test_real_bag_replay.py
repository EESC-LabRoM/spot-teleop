import json,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch
import real_bag_replay as R
def packet(stamp=100):
 return dict(topic='/fr3_operator/teleop_sample',version=3,orientation_mode='full_6d',
 frame='fr3_link0',stamp_ns=stamp,orientation_stamp_ns=stamp,p=[.1,.2,.3],
 q=[0,0,0,1],position_valid=True,orientation_valid=True)
class ReplayTests(unittest.TestCase):
 def test_rebase_preserves_pose_and_paired_time_without_mutating_source(self):
  x=packet();y=R.rebase(x,100,900)
  self.assertEqual(y['stamp_ns'],900);self.assertEqual(y['orientation_stamp_ns'],900)
  self.assertEqual(y['p'],x['p']);self.assertEqual(x['stamp_ns'],100)
 def test_unknown_orientation_is_not_fabricated(self):
  x=packet();x.update(orientation_valid=False,q=None,orientation_stamp_ns=None)
  y=R.rebase(x,100,900);self.assertIsNone(y['q']);self.assertIsNone(y['orientation_stamp_ns'])
 def test_duplicate_and_invalid_packets_are_rejected(self):
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'samples.jsonl'
   for events in [[packet(),packet(),packet(200)],[packet(),packet(200),dict(packet(300),p=[float('nan'),0,0])]]:
    p.write_text('\n'.join(json.dumps(x) for x in events))
    with self.assertRaises(ValueError):R.packets(p)
 def observer(self,**z):
  return NS(current=lambda:dict(engaged=False,zed=dict(control_backend='curobo',exercise_profile='horizontal',home_running=False,**z)))
 def test_preflight_rejects_wrong_backend(self):
  o=NS(current=lambda:dict(engaged=False,zed=dict(control_backend='cartesian',exercise_profile='horizontal')))
  with self.assertRaises(RuntimeError):R.preflight(o,'horizontal')
 def test_preflight_rejects_multiple_hardware_writers(self):
  with patch.object(R,'get',return_value=dict(armed=False,hardware_publisher_count=2)):
   with self.assertRaises(RuntimeError):R.preflight(self.observer(),'horizontal')
 def test_preflight_requires_disarmed_state(self):
  with patch.object(R,'get',return_value=dict(armed=True,hardware_publisher_count=1)):
   with self.assertRaises(RuntimeError):R.preflight(self.observer(),'horizontal')
 def test_positive_preflight_requires_ten_consecutive_valid_observations(self):
  b=dict(armed=False,hardware_publisher_count=1,command_ready=True)
  with patch.object(R,'get',return_value=b) as get,patch.object(R.time,'sleep'):
   rows=R.preflight(self.observer(),'horizontal')
  self.assertEqual(len(rows),10);self.assertEqual(get.call_count,10)
if __name__=='__main__':unittest.main()
