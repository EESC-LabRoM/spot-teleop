"""Start eligibility and guarded dashboard preparation; all services are mocks."""
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock
from fr3_input_teleop import Fr3JoyTeleop
from fr3_web_dashboard import Dash, make_handler
from curobo_start import prepare


def standby():
    return dict(armed=False,mode_ready=False,command_ready=True,controllers_age_ms=20,
                hardware_publisher_count=1,controllers=dict(fr3_arm_controller='inactive'))


class StartRegressionTests(unittest.TestCase):
    def test_healthy_standby_start_eligibility_is_separate_from_home(self):
        n=NS(mpc_routed=True,curobo_status=standby(),curobo_status_at=time.monotonic())
        self.assertIsNone(Fr3JoyTeleop._zed_start_ready(n))
        self.assertIsNotNone(Fr3JoyTeleop._zed_controller_ready(n))
        n.curobo_status_at-=1.
        self.assertEqual(Fr3JoyTeleop._zed_start_ready(n),'cuRobo bridge absent/stale')

    def test_standby_stale_feedback_invalid_packet_and_owner_block_start(self):
        for fault in ('feedback','packet','owner'):
            status=standby()
            if fault in ('feedback','packet'):
                status.update(command_ready=False,packet_reason='robot joint feedback stale' if fault=='feedback' else 'MPC solution infeasible')
            else:status['hardware_publisher_count']=2
            n=NS(mpc_routed=True,curobo_status=status,curobo_status_at=time.monotonic())
            self.assertIsNotNone(Fr3JoyTeleop._zed_start_ready(n))
            switch=Mock();arm=Mock();self.assertFalse(prepare(lambda:status,switch,arm,Mock())[0])
            switch.assert_not_called();arm.assert_not_called()

    def test_start_and_stop_start_use_real_guarded_prepare(self):
        status=standby();calls=[]
        def switch(on,off):
            calls.append('switch');status['controllers']['fr3_arm_controller']='active';return True
        def arm():calls.append('arm');status.update(armed=True,mode_ready=True);return True,'ready'
        def disarm(enabled=False):calls.append('disarm');status.update(armed=False,mode_ready=False);return True,'stopped'
        client=Mock();f=NS(result=lambda:NS(success=True,message='ok'));f.add_done_callback=lambda cb:cb(f)
        client.call_async.return_value=f
        n=NS(zed_status=dict(control_backend='curobo'),zed_status_at=time.monotonic(),zed_cancel=threading.Event(),
             zed_start_cli=client,zed_stop_cli=client,_curobo_enable=disarm,
             _prepare_curobo=lambda:prepare(lambda:status,switch,arm,disarm))
        self.assertEqual(calls,[])  # Construction/page observation never prepares.
        self.assertTrue(Dash._control_zed(n,'start')[0]);self.assertEqual(calls,['switch','arm'])
        self.assertTrue(Dash._control_zed(n,'disengage')[0]);self.assertTrue(Dash._control_zed(n,'start')[0])
        self.assertEqual(calls,['switch','arm','disarm','arm'])

    def test_stale_bridge_report_cannot_prepare(self):
        n=NS(zed_status=dict(control_backend='curobo',start_reason='cuRobo bridge absent/stale'),
             zed_status_at=time.monotonic(),_prepare_curobo=Mock())
        self.assertEqual(Dash._control_zed(n,'start'),(False,'cuRobo bridge absent/stale'))
        n._prepare_curobo.assert_not_called()

    def test_stale_dashboard_cannot_prepare(self):
        n=NS(zed_status=dict(control_backend='curobo'),zed_status_at=time.monotonic()-3.,_prepare_curobo=Mock())
        self.assertFalse(Dash._control_zed(n,'start')[0]);n._prepare_curobo.assert_not_called()

    def test_page_get_never_calls_control_or_prepare(self):
        import urllib.request
        from http.server import ThreadingHTTPServer
        from unittest.mock import patch, mock_open
        n=NS(control_zed=Mock(),_prepare_curobo=Mock())
        server=ThreadingHTTPServer(('127.0.0.1',0),make_handler(n))
        thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        try:
            with patch('builtins.open',mock_open(read_data=b'<html>test</html>')):
                with urllib.request.urlopen(f'http://127.0.0.1:{server.server_port}/') as response:
                    self.assertEqual(response.status,200)
            n.control_zed.assert_not_called();n._prepare_curobo.assert_not_called()
        finally:
            server.shutdown();server.server_close();thread.join()

    def test_page_uses_start_eligibility_and_does_not_prepare_on_load(self):
        html=Path(__file__).with_name('lab_dashboard.html').read_text()
        line=next(line for line in html.splitlines() if "$('zedactivate').disabled = !backendReady" in line)
        self.assertIn('start_reason',line);self.assertNotIn('controller_reason',line)
        home=next(line for line in html.splitlines() if "$('zedhome').disabled" in line)
        self.assertIn('controller_reason',home)
        connect=html[html.index('function connect()'):]
        self.assertNotIn('zedControl(',connect)
