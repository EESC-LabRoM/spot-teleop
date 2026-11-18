import ast,types,time,math,unittest
from pathlib import Path
class Freshness(unittest.TestCase):
 def test_actual_callback_partial_stale_nonfinite(self):
  source=Path(__file__).with_name('direct_topic_fwd.py');tree=ast.parse(source.read_text());fn=next(x for x in tree.body if isinstance(x,ast.FunctionDef) and x.name=='on_js');st={'meas':{},'measured_at':0.};scope=dict(st=st,time=time,math=math,n=types.SimpleNamespace(get_clock=lambda:types.SimpleNamespace(now=lambda:types.SimpleNamespace(nanoseconds=100_000_000_000))))
  exec(compile(ast.fix_missing_locations(ast.Module(body=[fn],type_ignores=[])),str(source),'exec'),scope)
  def msg(names,q,sec):return types.SimpleNamespace(name=names,position=q,header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=sec,nanosec=0)))
  names=[f'fr3_joint{i}' for i in range(1,8)]
  scope['on_js'](msg(['fr3_finger_joint1'],[.04],100));self.assertFalse(st['meas'])
  scope['on_js'](msg(names,[0]*7,99));self.assertFalse(st['meas'])
  scope['on_js'](msg(names,[float('nan')]*7,100));self.assertFalse(st['meas'])
  scope['on_js'](msg(names,[.1]*7,100));self.assertEqual(len(st['meas']),7);t=st['measured_at']
  scope['on_js'](msg(['fr3_finger_joint1'],[.04],100));self.assertEqual(st['measured_at'],t)
if __name__=='__main__':unittest.main()
