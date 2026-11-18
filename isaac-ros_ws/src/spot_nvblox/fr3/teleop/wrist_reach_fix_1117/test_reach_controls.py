import os
os.environ.update(TRANSLATION_GAIN="1",VERTICAL_GAIN="1.5",YAW_RATE="15",PRESERVE_ANCHOR="0")
from pathlib import Path
ROOT=Path(__file__).resolve().parent
import math,numpy as np
from scipy.spatial.transform import Rotation as R
from rate_yaw import RateYaw
q=np.array([1.,0,0,0]);wrist=R.from_rotvec([math.radians(25),0,0]).as_quat();control=RateYaw([0,0,0,1],q,15)
for _ in range(3000):q=control.step(wrist,q,.02)
assert abs(control.yaw-control.initial_yaw)<=math.radians(60)+1e-8
before=control.yaw
for _ in range(200):q=control.step([0,0,0,1],q,.02)
assert abs(control.yaw-before)<1e-8
for _ in range(6000):q=control.step(R.from_rotvec([-math.radians(25),0,0]).as_quat(),q,.02)
assert abs(control.yaw-control.initial_yaw)<=math.radians(60)+1e-8
print('PASS sustained pronation bounded at60deg, neutral hold, reverse bounded')
from consumer_kernel import Kernel
p=np.array([.31,0,.49]);q=np.array([1.,0,0,0]);k=Kernel(str(ROOT/'fr3_input_teleop_wrist_1117.py'),p,q)
for i in range(300):
 t=i*.02;packet=dict(stamp_ns=int((1000+t)*1e9),p=[0,0,-1.315*.10*min(1,t/2)],q=[0,0,0,1],orientation_valid=True);p,q=k.step(t,packet,t,p,q)
assert p[2]<.36,p
print('PASS exact consumer100mm hand drop produces~150mm robot drop',p)

import os
os.environ['TRANSLATION_GAIN']='2'
os.environ['VERTICAL_GAIN']='1.5'
p=np.array([.31,0,.49]);q=np.array([1.,0,0,0]);k=Kernel(str(ROOT/'fr3_input_teleop_wrist_1117.py'),p,q)
for i in range(400):
 t=i*.02;packet=dict(stamp_ns=int((1000+t)*1e9),p=[1.315*.02*min(1,t/2),0,-1.315*.03*min(1,t/2)],q=[0,0,0,1],orientation_valid=True);p,q=k.step(t,packet,t,p,q)
anchor=k.n.zed_anchor[2]/1.315
expected=np.array([.31,0,.49])+(np.array([.02,0,-.03])-anchor)*np.array([2.,2.,3.])
assert np.linalg.norm(p-expected)<.001,(p,expected)
print('PASS 20mm hand x ->40mm robot x;30mm hand z ->90mm robot z',p)
