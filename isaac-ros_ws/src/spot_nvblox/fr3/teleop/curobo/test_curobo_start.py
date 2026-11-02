import unittest
from unittest.mock import Mock
from curobo_start import prepare
class StartTests(unittest.TestCase):
 def state(self,**kw):return dict(command_ready=True,armed=False,controllers_age_ms=20,hardware_publisher_count=1,controllers=dict(fr3_arm_controller='inactive',joint_state_broadcaster='active'),**kw)
 def test_invalid_planner_does_not_switch_or_arm(self):
  switch=Mock();arm=Mock();s=self.state();s.update(command_ready=False,packet_reason='MPC solution infeasible')
  self.assertEqual(prepare(lambda:s,switch,arm,Mock()),(False,'MPC solution infeasible'));switch.assert_not_called();arm.assert_not_called()
 def test_duplicate_hardware_publisher_does_not_switch(self):
  s=self.state();s['hardware_publisher_count']=2;switch=Mock();self.assertFalse(prepare(lambda:s,switch,Mock(),Mock())[0]);switch.assert_not_called()
 def test_prepare_then_arm_only_after_controller_confirmed(self):
  initial=self.state();active=self.state();active['controllers']['fr3_arm_controller']='active';statuses=Mock(side_effect=[initial,active]);switch=Mock(return_value=True);arm=Mock(return_value=(True,'ready'))
  self.assertTrue(prepare(statuses,switch,arm,Mock())[0]);switch.assert_called_once_with(['fr3_arm_controller'],[]);arm.assert_called_once()
 def test_arming_failure_restores_previous_controller_state(self):
  initial=self.state();active=self.state();active['controllers']['fr3_arm_controller']='active';switch=Mock(return_value=True);disarm=Mock()
  self.assertFalse(prepare(Mock(side_effect=[initial,active]),switch,Mock(return_value=(False,'stale depth')),disarm)[0]);disarm.assert_called_once();self.assertEqual(switch.call_args_list[-1].args,([],['fr3_arm_controller']))
 def test_other_active_controller_is_not_taken_over(self):
  s=self.state();s['controllers']['other_robot_controller']='active';switch=Mock();self.assertFalse(prepare(lambda:s,switch,Mock(),Mock())[0]);switch.assert_not_called()
 def test_cancel_before_start_never_switches_or_arms(self):
  switch=Mock();arm=Mock();self.assertFalse(prepare(lambda:self.state(),switch,arm,Mock(),cancelled=lambda:True)[0]);switch.assert_not_called();arm.assert_not_called()
 def test_stop_during_handover_prevents_arming_and_rolls_back(self):
  initial=self.state();active=self.state();active['controllers']['fr3_arm_controller']='active';switch=Mock(return_value=True);arm=Mock();cancelled=Mock(side_effect=[False,False,True])
  self.assertFalse(prepare(Mock(side_effect=[initial,active]),switch,arm,Mock(),cancelled=cancelled)[0]);arm.assert_not_called();self.assertEqual(switch.call_args_list[-1].args,([],['fr3_arm_controller']))
 def test_stop_during_arm_response_disarms_and_restores_controller(self):
  initial=self.state();active=self.state();active['controllers']['fr3_arm_controller']='active';cancel=[False];disarm=Mock();switch=Mock(return_value=True)
  def arm():cancel[0]=True;return True,'ready'
  self.assertFalse(prepare(Mock(side_effect=[initial,active]),switch,arm,disarm,cancelled=lambda:cancel[0])[0]);disarm.assert_called_once();self.assertEqual(switch.call_args_list[-1].args,([],['fr3_arm_controller']))
if __name__=='__main__':unittest.main()
