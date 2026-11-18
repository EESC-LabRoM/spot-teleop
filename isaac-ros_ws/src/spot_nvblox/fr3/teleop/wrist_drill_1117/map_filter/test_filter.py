import unittest,numpy as np
from drill_depth_filter import carve,Pairing
class Tests(unittest.TestCase):
 def test_exclusion_and_outside(self):
  d=np.ones((100,100),np.float32);d[0,0]=np.nan;m=np.zeros((100,100),np.uint8);m[45:55,45:55]=255
  out,r=carve(d,m,3);self.assertFalse(np.any(out[r]));self.assertTrue(np.array_equal(out[~r],d[~r],equal_nan=True));self.assertGreater(r.sum(),100)
 def test_empty_and_bad_shape_refused(self):
  for m in [np.zeros((100,100),np.uint8),np.ones((100,100),np.uint8),np.ones((100,200),np.uint8)]:
   with self.assertRaises(ValueError):carve(np.ones((100,100),np.float32),m)
 def test_source_stamp_gates(self):
  g=Pairing();self.assertTrue(g.valid(10,10.01,10.2));self.assertFalse(g.valid(10,9.9,10.2));self.assertFalse(g.valid(10,10,11));self.assertFalse(g.valid(10,10,9.9));g.last=10;self.assertFalse(g.valid(10,10,10.1));self.assertFalse(g.valid(float('nan'),10,10.1))
if __name__=='__main__':unittest.main()
