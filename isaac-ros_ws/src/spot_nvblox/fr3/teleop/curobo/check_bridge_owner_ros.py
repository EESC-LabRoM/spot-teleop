import unittest,time
from types import SimpleNamespace
from joystick_bridge import Bridge
class OwnershipTests(unittest.TestCase):
 def bridge(self):
  n=Bridge.__new__(Bridge);n.controllers_at=time.monotonic();n.controllers={'fr3_arm_controller':'active'};n.jtc=SimpleNamespace(get_subscription_count=lambda:1);n.get_publishers_info_by_topic=lambda _:['one'];return n
 def test_cartesian_not_loaded_is_safe_for_joint_controller(self):self.assertTrue(self.bridge().controller_ready())
 def test_cartesian_active_refuses_ownership(self):
  n=self.bridge();n.controllers['cartesian_impedance_target_controller']='active';self.assertFalse(n.controller_ready())
 def test_two_publishers_refuse_ownership(self):
  n=self.bridge();n.get_publishers_info_by_topic=lambda _:['one','two'];self.assertFalse(n.controller_ready())
if __name__=='__main__':unittest.main()
