"""Validate the official segment-sum estimator without ROS, cameras or MediaPipe."""
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest
import numpy as np

source=Path(__file__).resolve().parents[5]/'spot-ros2_ws/src/arm_pose_estimator/arm_pose_estimator/wrist_detector.py'
tree=ast.parse(source.read_text())
cls=next(n for n in tree.body if isinstance(n,ast.ClassDef))
method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='update_arm_length')
ns={'np':np};exec(compile(ast.Module(body=[method],type_ignores=[]),str(source),'exec'),ns)

class ScaleTests(unittest.TestCase):
    def setUp(self):
        self.n=SimpleNamespace(arm_length_samples=[],arm_length_window=150,arm_length_min_samples=90,
            arm_length_max_spread=.03,robot_reach=.855,online_scale=None,frame_count=1,
            get_logger=lambda:SimpleNamespace(info=lambda s:None))
    def sample(self,length):
        ns['update_arm_length'](self.n,np.zeros(3),np.array([length*.5,0,0]),np.array([length,0,0]))
    def test_stable_arm_sets_fr3_reach_ratio_only_after_90_samples(self):
        for _ in range(89):self.sample(.65)
        self.assertIsNone(self.n.online_scale)
        self.sample(.65);self.assertAlmostEqual(self.n.online_scale,.855/.65)
    def test_noisy_depth_does_not_set_scale(self):
        for i in range(150):self.sample(.4 if i%2 else .8)
        self.assertIsNone(self.n.online_scale)
    def test_fr3_rejects_stable_short_arm_and_bad_segment(self):
        self.n.arm_length_range=(.45,.95);self.n.arm_segment_range=(.15,.5)
        for _ in range(150):self.sample(.324)
        self.assertIsNone(self.n.online_scale);self.assertEqual(self.n.arm_length_samples,[])
        for _ in range(150):ns['update_arm_length'](self.n,np.zeros(3),np.array([.05,0,0]),np.array([.65,0,0]))
        self.assertIsNone(self.n.online_scale);self.assertEqual(self.n.arm_length_samples,[])

    def test_implausible_lengths_rejected(self):
        for l in [0.,.1,2.,float('nan')]:self.sample(l)
        self.assertEqual(self.n.arm_length_samples,[])

if __name__=='__main__':unittest.main(verbosity=2)
