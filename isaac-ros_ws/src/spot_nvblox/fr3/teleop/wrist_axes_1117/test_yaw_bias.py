import math,numpy as np
from wrist_yaw_bias import wrist_yaw_reference as f
q=[0,1,0,0];t=[0,math.cos(.3),math.sin(.3),0]
a=f(.5,q,t,-2.9,2.9);assert a<.5
assert abs(a-f(.5,-np.array(q),-np.array(t),-2.9,2.9))<1e-10
assert f(.5,[1,0,0,0],[1,0,0,0],-2.9,2.9)==.5
assert -2.9+math.radians(5)<=f(-2.89,q,t,-2.9,2.9)
for bad in [[0,0,0,0],[float('nan'),0,0,0]]:
 try:f(.5,bad,t,-2.9,2.9)
 except ValueError:pass
 else:raise AssertionError('invalid quaternion accepted')
print('PASS: yaw sign, antipodal invariance, non-down fallback, limits, invalid quaternion')
