import importlib.util
import io
import json
import unittest
from pathlib import Path
from unittest.mock import patch
p = Path(__file__).with_name('lab_live_server.py')
spec=importlib.util.spec_from_file_location('candidate_dashboard',p)
m=importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
class Fake:
    def __init__(self, body=b'', header='1'):
        self.headers={'Content-Length':str(len(body)),'X-Dashboard':header}; self.rfile=io.BytesIO(body); self.result=None
    def _send(self,status,content_type,body): self.result=(status,content_type,body); return self.result
class Up:
    status=200
    def __enter__(self): return self
    def __exit__(self,*args): pass
    def read(self,n): return b'{"selected":"drill","status":"ready"}'
class Tests(unittest.TestCase):
    def test_missing_service_is_unavailable(self):
        f=Fake()
        with patch.object(m.urllib.request,'urlopen',side_effect=OSError('offline')): m.H._object_target_proxy(f,'GET')
        self.assertEqual(f.result[0],503)
    def test_wrong_target_is_not_sent_upstream(self):
        f=Fake(b'{"target":"unsafe"}')
        with patch.object(m.urllib.request,'urlopen') as up: m.H._object_target_proxy(f,'POST'); up.assert_not_called()
        self.assertEqual(f.result[0],400)
    def test_header_required(self):
        f=Fake(b'{"target":"drill"}',header='')
        with patch.object(m.urllib.request,'urlopen') as up: m.H._object_target_proxy(f,'POST'); up.assert_not_called()
        self.assertEqual(f.result[0],403)
    def test_valid_select_uses_fixed_loopback(self):
        f=Fake(b'{"target":"tennis_ball"}')
        with patch.object(m.urllib.request,'urlopen',return_value=Up()) as up:
            m.H._object_target_proxy(f,'POST')
            req=up.call_args.args[0];self.assertEqual(req.full_url,'http://127.0.0.1:8898/object-target');self.assertEqual(json.loads(req.data),{'target':'tennis_ball'})
        self.assertEqual(f.result[0],200)
if __name__=='__main__': unittest.main()
