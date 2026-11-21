"""Local selector HTTP contract; no ROS, map or robot dependencies."""
import importlib.util
import json
import sys
import threading
import time
import unittest
import urllib.error
import urllib.request
from pathlib import Path
root=Path(__file__).resolve().parent.parent
sys.path.insert(0,str(root/'backend'))
from selector_backend import ObjectSelector
from selector_http import SelectorService,make_server
spec=importlib.util.spec_from_file_location('candidate_dashboard',Path(__file__).with_name('lab_live_server.py'))
dashboard=importlib.util.module_from_spec(spec);spec.loader.exec_module(dashboard)

class Tests(unittest.TestCase):
 def test_target_flow_and_stale_state_refusal(self):
  state={'engaged':False,'busy':False,'held':False,'stamp':time.time()}
  service=SelectorService(ObjectSelector(time.time),lambda:state,time.time)
  server=make_server(service)
  thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
  url='http://127.0.0.1:%d/object-target'%server.server_port
  try:
   status=json.load(urllib.request.urlopen(url,timeout=2));self.assertTrue(status['can_select'])
   for target in ('drill','tennis_ball','sprite'):
    req=urllib.request.Request(url,data=json.dumps({'target':target}).encode(),headers={'Content-Type':'application/json'},method='POST')
    with urllib.request.urlopen(req,timeout=2) as r:data=json.load(r);self.assertEqual(r.status,202)
    self.assertEqual(data['selected'],target);self.assertFalse(data['ready']);self.assertEqual(data['status'],'pending')
   state['stamp']=time.time()-10
   req=urllib.request.Request(url,data=b'{"target":"drill"}',headers={'Content-Type':'application/json'},method='POST')
   with self.assertRaises(urllib.error.HTTPError) as e:urllib.request.urlopen(req,timeout=2)
   self.assertEqual(e.exception.code,409)
   status=json.load(urllib.request.urlopen(url,timeout=2));self.assertEqual(status['selected'],'sprite');self.assertFalse(status['can_select'])
  finally:server.shutdown();server.server_close();thread.join(timeout=2)
if __name__=='__main__':unittest.main()
