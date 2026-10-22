import unittest
import numpy as np
from arm_calibration import ArmCalibration

class ArmCalibrationTests(unittest.TestCase):
    def observe(self,c,length,stamp):
        return c.observe([0,0,1],[length/2,0,1],[length,0,1],stamp)
    def calibrated(self,length=.65):
        c=ArmCalibration()
        for i in range(45):self.observe(c,length,i*.2)
        return c
    def test_person_specific_scale_needs_distinct_stable_samples(self):
        c=ArmCalibration()
        for _ in range(100):self.observe(c,.65,1.)
        self.assertIsNone(c.scale)
        c=self.calibrated();self.assertAlmostEqual(c.scale,.855/.65)
        self.assertAlmostEqual(self.calibrated(.8).scale,.855/.8)
    def test_outliers_segments_and_nonfinite_rejected(self):
        c=ArmCalibration()
        for i in range(100):self.observe(c,.2,i)
        self.assertFalse(c.samples)
        self.assertFalse(c.observe([0,0,1],[float('nan'),0,1],[.6,0,1],101))
    def test_dominant_consensus_discards_depth_outliers(self):
        c=ArmCalibration()
        for i in range(8):self.observe(c,.8,i*.2)
        for i in range(20):self.observe(c,.65,2+i*.2)
        self.assertAlmostEqual(c.length,.65)

    def test_noisy_calibration_does_not_arm(self):
        c=ArmCalibration()
        for i in range(90):self.observe(c,.5 if i%2 else .8,i*.2)
        self.assertIsNone(c.scale)
    def test_monitoring_never_changes_latched_scale_and_reset_requires_new_person(self):
        c=self.calibrated();scale=c.scale
        for i in range(21):self.observe(c,.8,100+i)
        self.assertEqual(c.scale,scale);self.assertAlmostEqual(c.check_length,.8)
        c.reset();self.assertIsNone(c.scale);self.assertEqual(c.samples,[])
    def test_bent_arm_uses_both_segments(self):
        c=ArmCalibration()
        for i in range(45):c.observe([0,0,1],[.3,0,1],[.3,.35,1],i*.2)
        self.assertAlmostEqual(c.scale,.855/.65)

if __name__=='__main__':unittest.main()
