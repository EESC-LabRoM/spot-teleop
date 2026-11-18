"""Exact consumer mapping with the new calibrated sample, without ROS or hardware."""
import os,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'runtime'))
from arm_calibration import ArmCalibration
from operator_reference import OperatorReference
from hand_tracking_math import CAMERA_TO_ROBOT
consumer=ROOT.parent/'wrist_reach_fix_1117'
sys.path.insert(0,str(consumer))
from consumer_kernel import Kernel
os.environ.update(TRANSLATION_GAIN='1',VERTICAL_GAIN='1.5',PRESERVE_ANCHOR='0')
a=ArmCalibration();r=OperatorReference();s=np.array([.2,-.2,1.2])
for i in range(30):
 t=i*.2;a.observe(s,s+[.30,0,0],s+[.30,.35,0],t);r.observe(CAMERA_TO_ROBOT,s,t)
assert r.maybe_latch(a.scale is not None)
p=np.array([.31,0,.49]);q=np.array([1.,0,0,0]);k=Kernel(consumer/'fr3_input_teleop_wrist_1117.py',p,q)
k.n.zed_input_position_gain=1.0  # No fixed-camera de-amplification after arm normalization.
for i in range(400):
 t=i*.02;drop=.03*min(1.,max(0.,(t-2)/2))
 wrist=s+np.array([.2,.1+drop,-.3]);sample=r.project(wrist,6+t,a.scale)
 packet=dict(stamp_ns=int((1000+t)*1e9),p=sample.tolist(),q=[0,0,0,1],orientation_valid=True)
 p,q=k.step(t,packet,t,p,q)
expected=.49-.03*a.scale*1.5
assert abs(p[2]-expected)<.001,(p,expected)
print('PASS exact consumer applies arm scale once; 30mm hand ->',round((.49-p[2])*1000,2),'mm TCP at scale',a.scale)
