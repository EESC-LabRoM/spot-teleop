import unittest,ast
from pathlib import Path
from murilo_current import instrument
SOURCE=Path('/home/murilo/franka_ws/hub-curobo/node/curobo_mpc_node.py')
class SourceContractTests(unittest.TestCase):
 def test_current_source_preserves_new_native_trajectory_and_keep_solver(self):
  tree=instrument(SOURCE.read_text(),str(SOURCE));code=ast.unparse(tree)
  self.assertIn('mpc_on_infeasible',code);self.assertIn('self.mpc._step_once',code);self.assertIn('JointTrajectoryPoint()',code);self.assertIn('_emit_guarded(',code)
  compile(tree,str(SOURCE),'exec')
 def test_missing_publish_boundary_fails_closed(self):
  source=SOURCE.read_text().replace('self.cmd_pub.publish(joint_cmd)','self.cmd_pub.publish(new_unknown_contract)')
  with self.assertRaises(ValueError):instrument(source,'unexpected.py')
 def test_duplicate_boundary_fails_closed(self):
  source=SOURCE.read_text().replace('self.cmd_pub.publish(joint_cmd)','self.cmd_pub.publish(joint_cmd)\n            self.cmd_pub.publish(joint_cmd)')
  with self.assertRaises(ValueError):instrument(source,'duplicate.py')
if __name__=='__main__':unittest.main()
