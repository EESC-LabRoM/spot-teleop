"""Body calibration must not silently weaken post-loss operator association."""
from pathlib import Path
import ast,types,sys
import numpy as np
ROOT=Path(__file__).resolve().parent;sys.path.insert(0,str(ROOT/'runtime'))
from arm_calibration import ArmCalibration
src=ast.parse((ROOT/'runtime/operator_hand_tracker_debug.py').read_text())
cls=next(x for x in src.body if isinstance(x,ast.ClassDef) and x.name=='OperatorHandTracker')
method=next(x for x in cls.body if isinstance(x,ast.FunctionDef) and x.name=='associate')
ns=dict(np=np,AssociationMiss=ValueError);exec(compile(ast.Module(body=[method],type_ignores=[]),'associate_guard','exec'),ns)
arm=ArmCalibration();arm.restore(dict(version=1,arm_length_m=.65,arm_upper_m=.3,arm_forearm_m=.35),[0,0,1],1.)
node=types.SimpleNamespace(arm=arm,body_wrist=None,body_operator_stamp=-1,hand_identity=None,body_shoulder=None)
result=types.SimpleNamespace(hand_landmarks=[object()],handedness=[[types.SimpleNamespace(score=.99,category_name='Right')]])
try:ns['associate'](node,result,None,20_000_000_000,640,360,{})
except ValueError as e:assert 'confirmar o operador' in str(e),e
else:raise AssertionError('stale identity was silently accepted')
print('PASS post-loss hand classification alone cannot impersonate confirmed operator identity')
