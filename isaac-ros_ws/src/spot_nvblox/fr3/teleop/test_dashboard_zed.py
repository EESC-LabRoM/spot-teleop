"""HTTP button checks with a fake backend: no calls to robot services."""
import json
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
from http.server import ThreadingHTTPServer
import urllib.request
import urllib.error
from fr3_web_dashboard import make_handler, Dash

class ButtonTests(unittest.TestCase):
    def setUp(self):
        self.node=SimpleNamespace(rp=None,control_zed=Mock(return_value=(True,'ok')))
        self.server=ThreadingHTTPServer(('127.0.0.1',0),make_handler(self.node))
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True);self.thread.start()
        self.url='http://127.0.0.1:'+str(self.server.server_port)+'/zed-control'
    def tearDown(self):
        self.server.shutdown();self.server.server_close();self.thread.join()
    def post(self,action='engage',guard=True,origin=None):
        h={'Content-Type':'application/json'}
        if guard:h['X-Dashboard']='1'
        if origin:h['Origin']=origin
        r=urllib.request.Request(self.url,data=json.dumps({'action':action}).encode(),headers=h,method='POST')
        try:
            with urllib.request.urlopen(r,timeout=3) as f:return f.status,f.read()
        except urllib.error.HTTPError as e:return e.code,e.read()
    def test_explicit_enable_and_disable_are_forwarded(self):
        self.assertEqual(self.post()[0],200);self.node.control_zed.assert_called_with('engage')
        self.assertEqual(self.post('disengage')[0],200);self.node.control_zed.assert_called_with('disengage')
    def test_start_and_home_are_explicit_guarded_actions(self):
        self.assertEqual(self.post('start')[0],200);self.node.control_zed.assert_called_with('start')
        self.assertEqual(self.post('home')[0],200);self.node.control_zed.assert_called_with('home')
        self.node.control_zed.reset_mock()
        self.assertEqual(self.post('home',guard=False)[0],403)
        self.node.control_zed.assert_not_called()
    def test_recalibration_request_is_guarded(self):
        self.assertEqual(self.post('recalibrate')[0],200)
        self.node.control_zed.assert_called_with('recalibrate')
        self.node.control_zed.reset_mock()
        self.assertEqual(self.post('recalibrate',guard=False)[0],403)
        self.node.control_zed.assert_not_called()

    def test_person_reset_stops_before_sending_reset_and_starting(self):
        calls=[]
        fake=SimpleNamespace(_control_zed=lambda action: (calls.append(action) or (True,'ok')))
        with patch('socket.socket') as sock:
            sock.return_value.__enter__.return_value.sendto.side_effect=lambda *args: calls.append('reset')
            self.assertEqual(Dash._control_zed(fake,'recalibrate'),(True,'ok'))
        self.assertEqual(calls,['disengage','reset','start'])

    def test_person_reset_refuses_to_start_if_stop_fails(self):
        fake=SimpleNamespace(_control_zed=Mock(return_value=(False,'stop unavailable')))
        with patch('socket.socket') as sock:
            self.assertEqual(Dash._control_zed(fake,'recalibrate'),(False,'stop unavailable'))
            sock.assert_not_called()
        fake._control_zed.assert_called_once_with('disengage')

    def test_missing_guard_or_foreign_origin_cannot_enable(self):
        self.assertEqual(self.post(guard=False)[0],403)
        self.assertEqual(self.post(origin='https://example.invalid')[0],403)
        self.node.control_zed.assert_not_called()
    def test_bad_action_cannot_call_backend(self):
        self.assertEqual(self.post('arbitrary')[0],400);self.node.control_zed.assert_not_called()
    def test_tracking_refusal_is_shown_as_conflict(self):
        self.node.control_zed.return_value=(False,'tracking lost')
        code,body=self.post();self.assertEqual(code,409);self.assertEqual(json.loads(body)['message'],'tracking lost')
    def test_model_disallows_start_on_other_source_and_allows_stop(self):
        fake=SimpleNamespace(lock=threading.Lock(),input_source='joystick',engaged=False,zed_status=None,
                             zed_engage_cli=Mock(),zed_stop_cli=Mock())
        self.assertEqual(Dash._control_zed(fake,'engage'),(False,'Select ZED3 first'))
        fake.zed_engage_cli.call_async.assert_not_called()
        response=SimpleNamespace(success=True,message='disengaged')
        future=SimpleNamespace(result=lambda:response)
        future.add_done_callback=lambda fn:fn(future)
        fake.zed_stop_cli.call_async.return_value=future
        self.assertEqual(Dash._control_zed(fake,'disengage'),(True,'disengaged'))

if __name__=='__main__':unittest.main(verbosity=2)
