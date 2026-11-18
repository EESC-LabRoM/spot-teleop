import unittest,math
import numpy as np
from operator_reference import OperatorReference
from hand_tracking_math import CAMERA_TO_ROBOT

class ReferenceTests(unittest.TestCase):
 def ready(self):
  r=OperatorReference();s=np.array([.2,-.2,1.2])
  for i in range(20):self.assertTrue(r.observe(CAMERA_TO_ROBOT,s,i*.2))
  self.assertTrue(r.maybe_latch(True));return r,s
 def test_requires_scale_and_distinct_stable_samples(self):
  r=OperatorReference()
  for _ in range(30):r.observe(CAMERA_TO_ROBOT,[0,0,1.2],0)
  self.assertFalse(r.maybe_latch(True))
  for i in range(30):r.observe(CAMERA_TO_ROBOT,[0,0,1.2],i*.2)
  self.assertFalse(r.maybe_latch(False));self.assertTrue(r.maybe_latch(True))
 def test_metric_relative_position_and_bent_arm_normalization(self):
  r,s=self.ready();p=r.project(s+np.array([.1,0,-.2]),4.,1.5)
  np.testing.assert_allclose(p,[.3,.15,0],atol=1e-8)
 def test_body_axes_never_follow_arm_rotation(self):
  r,s=self.ready();before=r.rotation.copy()
  for i in range(10):r.observe(np.eye(3),s,4+i*.2)
  np.testing.assert_array_equal(r.rotation,before)
 def test_missing_body_holds_reference_without_recalibrating(self):
  r,s=self.ready()
  a=r.project(s+[.1,0,0],4.,1.5);b=r.project(s+[.1,0,0],12.,1.5)
  np.testing.assert_allclose(a,b);self.assertGreater(r.status(12.)['body_reference_age_s'],8.)
 def test_common_body_translation_is_smoothly_cancelled(self):
  r,s=self.ready();r.project(s+[.1,0,0],4.,1.)
  shift=np.array([.15,0,0])
  last=None
  for i in range(1,301):
   t=4+i/30
   if i%10==0:r.observe(CAMERA_TO_ROBOT,s+shift,t)
   p=r.project(s+shift+[.1,0,0],t,1.)
   last=p
  self.assertLess(np.linalg.norm(last-np.array([0,.1,0])),.025)
 def test_invalid_inputs_fail_closed(self):
  r,s=self.ready();self.assertFalse(r.observe(np.eye(3)*2,s,4.))
  self.assertFalse(r.observe(CAMERA_TO_ROBOT,s,float('nan')))
  with self.assertRaises(ValueError):r.project(s,4.,float('nan'))
  r.project(s,4.,1.)
  with self.assertRaises(ValueError):r.project(s,4.,1.)
 def test_start_preserves_reference_and_person_reset_clears_it(self):
  import ast,types
  from pathlib import Path
  from rgbd_pose import FixedFramePosition
  from palm_orientation import PalmFilter
  tree=ast.parse(Path(__file__).with_name('operator_hand_tracker_debug.py').read_text())
  cls=next(x for x in tree.body if isinstance(x,ast.ClassDef) and x.name=='OperatorHandTracker')
  method=next(x for x in cls.body if isinstance(x,ast.FunctionDef) and x.name=='reset_combined_pose')
  scope=dict(FixedFramePosition=FixedFramePosition,PalmFilter=PalmFilter,OperatorReference=OperatorReference)
  exec(compile(ast.Module(body=[method],type_ignores=[]),'reset_method','exec'),scope)
  ref,_=self.ready();node=types.SimpleNamespace(body_reference=ref)
  scope['reset_combined_pose'](node)
  self.assertIs(node.body_reference,ref)
  scope['reset_combined_pose'](node,reset_body=True)
  self.assertIsNone(node.body_reference.rotation)
 def test_cold_calibration_never_projects_a_default_frame(self):
  r=OperatorReference()
  with self.assertRaises(ValueError):r.project([.2,-.1,1.],0.,1.315)
 def test_explicit_shoulder_off_keeps_body_translation(self):
  r=OperatorReference(shoulder_mode='off');s=np.array([.2,-.2,1.2])
  for i in range(20):r.observe(CAMERA_TO_ROBOT,s,i*.2)
  self.assertTrue(r.maybe_latch(True))
  np.testing.assert_allclose(r.project(s+[.25,0,0],4.,1.),[0,.25,0])
 def test_contradictory_tracker_configuration_is_refused(self):
  import ast,types
  from pathlib import Path
  tree=ast.parse(Path(__file__).with_name('operator_hand_tracker_debug.py').read_text())
  cls=next(x for x in tree.body if isinstance(x,ast.ClassDef) and x.name=='OperatorHandTracker')
  method=next(x for x in cls.body if isinstance(x,ast.FunctionDef) and x.name=='__init__')
  scope={};exec(compile(ast.Module(body=[method],type_ignores=[]),'constructor_validation','exec'),scope)
  with self.assertRaisesRegex(ValueError,'remove --no-body-frame'):
   scope['__init__'](types.SimpleNamespace(),pose_source='rgbd_body',body_frame=False)
 def test_person_reset_cannot_reload_previous_saved_profile(self):
  import ast,types
  from pathlib import Path
  from arm_calibration import ArmCalibration
  tree=ast.parse(Path(__file__).with_name('operator_hand_tracker_debug.py').read_text())
  cls=next(x for x in tree.body if isinstance(x,ast.ClassDef) and x.name=='OperatorHandTracker')
  method=next(x for x in cls.body if isinstance(x,ast.FunctionDef) and x.name=='reset_calibration')
  stub=lambda *a,**k:types.SimpleNamespace(reset=lambda:None)
  scope={k:stub for k in ['RobustPalmFrame','AdaptivePalmOrientation','DepthAwarePosition','OrientationFilter','GestureOrientationLock','ShoulderRelativeWrist']};scope['PALM_LATCH_FRAMES']=5
  exec(compile(ast.Module(body=[method],type_ignores=[]),'reset_calibration','exec'),scope)
  profile=dict(version=1,arm_length_m=.65,arm_upper_m=.3,arm_forearm_m=.35)
  arm=ArmCalibration();arm.restore(profile,[0,0,1],1.)
  node=types.SimpleNamespace(reset_combined_pose=lambda **kw:None,arm_profile=profile,arm_reload_path=None,arm=arm,yaw_filter=stub(),body_frame_filter=stub(),shoulder_mode='follow')
  scope['reset_calibration'](node,force=False)
  self.assertIs(node.arm_profile,profile);self.assertIsNotNone(arm.scale)
  scope['reset_calibration'](node,force=True)
  self.assertIsNone(node.arm_profile);self.assertIsNone(arm.scale)
 def test_torso_depth_pitch_cannot_mix_height_into_forward(self):
  r=OperatorReference();s=np.array([.2,-.2,1.2]);a=math.radians(25)
  pitch=np.array([[1,0,0],[0,math.cos(a),-math.sin(a)],[0,math.sin(a),math.cos(a)]])
  for i in range(20):self.assertTrue(r.observe(CAMERA_TO_ROBOT@pitch,s,i*.2))
  self.assertTrue(r.maybe_latch(True))
  np.testing.assert_allclose(r.project(s+[0,.1,0],4.,1.),[0,0,-.1],atol=1e-8)
 def test_one_occluded_shoulder_does_not_move_torso_reference(self):
  r=OperatorReference();s=np.array([.2,-.2,1.2]);torso=dict(shoulder_r=s,shoulder_l=np.array([-.2,-.2,1.2]),hip_r=np.array([.15,.3,1.25]),hip_l=np.array([-.15,.3,1.25]))
  for i in range(20):r.observe(CAMERA_TO_ROBOT,s,i*.2,torso=torso)
  self.assertTrue(r.maybe_latch(True));p0=r.project(s+[.1,.1,-.2],4.,1.)
  bad=dict(torso);bad['shoulder_r']=s+[0,0,-.12]
  for i in range(1,61):
   t=4+i/30
   if i%10==0:self.assertTrue(r.observe(CAMERA_TO_ROBOT,bad['shoulder_r'],t,torso=bad))
   np.testing.assert_allclose(r.project(s+[.1,.1,-.2],t,1.),p0,atol=1e-8)
 def test_disagreeing_two_shoulders_hold_previous_reference(self):
  r=OperatorReference();s=np.array([.2,-.2,1.2]);torso=dict(shoulder_r=s,shoulder_l=np.array([-.2,-.2,1.2]))
  for i in range(20):r.observe(CAMERA_TO_ROBOT,s,i*.2,torso=torso)
  self.assertTrue(r.maybe_latch(True))
  bad=dict(torso);bad['shoulder_r']=s+[0,0,-.1]
  self.assertFalse(r.observe(CAMERA_TO_ROBOT,bad['shoulder_r'],4.,torso=bad))
if __name__=='__main__':unittest.main()
