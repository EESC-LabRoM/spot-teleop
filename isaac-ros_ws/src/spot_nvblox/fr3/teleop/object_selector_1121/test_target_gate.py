import unittest
from dataclasses import replace
from target_gate import Target, TargetGate, Evidence

class TargetGateTests(unittest.TestCase):
    def setUp(self):
        self.gate = TargetGate()
        self.gate.select('tennis_ball', engaged=False, busy=False, held=False)
        self.good = Evidence(1,'tennis_ball','tennis_ball',.3,.004,True,
                             'tennis_ball',True,True,True,
                             'tennis_ball',1,True,True)
    def test_validated_target_is_ready(self):
        self.assertTrue(self.gate.evaluate(self.good))
    def test_reselection_invalidates_old_grasp(self):
        self.gate.select('sprite', engaged=False, busy=False, held=False)
        self.assertFalse(self.gate.evaluate(self.good))
    def test_stale_wrong_or_lost_mask_refuses(self):
        for value in (replace(self.good,mask_age=1.51), replace(self.good,mask_target='drill'), replace(self.good,tracker_ok=False)):
            self.assertFalse(self.gate.evaluate(value))
    def test_non_target_map_obstacles_required(self):
        self.assertFalse(self.gate.evaluate(replace(self.good,other_obstacles_present=False)))
        self.assertFalse(self.gate.evaluate(replace(self.good,target_absent=False)))
    def test_grasp_must_match_generation_and_target(self):
        self.assertFalse(self.gate.evaluate(replace(self.good,binding_target='drill')))
        self.assertFalse(self.gate.evaluate(replace(self.good,binding_generation=0)))
        self.assertFalse(self.gate.evaluate(replace(self.good,held_model_ready=False)))
    def test_no_switch_while_driving_grasping_or_holding(self):
        for engaged,busy,held in ((True,False,False),(False,True,False),(False,False,True)):
            with self.assertRaises(RuntimeError): self.gate.select('sprite',engaged=engaged,busy=busy,held=held)
        self.assertEqual(self.gate.selected, 'tennis_ball')
    def test_reject_unknown_target(self):
        with self.assertRaises(ValueError): self.gate.select('unknown',engaged=False,busy=False,held=False)

if __name__ == '__main__': unittest.main()
