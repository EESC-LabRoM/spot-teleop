"""Actual bridge methods with mock transport/TF; teleop uses isolated DDS191."""
import copy
import json
import sys
import time
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock
import numpy as np
sys.path.insert(0,str(Path(__file__).parent/'curobo'))
from joystick_bridge import Bridge
from command_guard import ARM
import test_fr3_input as input_tests


def bridge():
    now=time.time_ns()
    q=[0.,-.6,0.,-2.,0.,1.5,0.]
    p=dict(version=1,frame='fr3_link0',names=ARM,q=q.copy(),v=[0.]*7,feasible=True,grid_covers_robot=True,
           **{k:now-1000000 for k in ('stamp_ns','goal_stamp_ns','joint_stamp_ns','esdf_stamp_ns','depth_stamp_ns')})
    clock=NS(now=lambda:NS(nanoseconds=now))
    tr=NS(header=NS(stamp=NS(sec=now//10**9,nanosec=now%10**9)),
          transform=NS(translation=NS(x=.4,y=0.,z=.5),rotation=NS(x=0.,y=0.,z=0.,w=1.)))
    b=NS(armed=True,engaged=True,engaged_at=time.monotonic(),source='zed',status=dict(phase='paused'),status_at=time.monotonic(),
         command=None,q=q,q_at=time.monotonic(),last_packet=p,last_sent=None,last_previous=None,
         controllers=dict(fr3_arm_controller='active'),controllers_at=time.monotonic(),reason='active',sent=0,
         tf=NS(lookup_transform=Mock(return_value=tr)),sock=Mock(),goal_port=12345,jtc=Mock(),state_pub=Mock(),
         get_clock=lambda:clock,get_publishers_info_by_topic=lambda topic:[object()])
    b.sock.recv.side_effect=BlockingIOError
    b.jtc.get_subscription_count.return_value=1
    for name in ('current_packet','controller_reason','controller_ready','send_goal','output','publish_status'):
        setattr(b,name,getattr(Bridge,name).__get__(b))
    return b


class BridgeTests(unittest.TestCase):
    def test_paused_goal_uses_fresh_measured_pose_and_no_output(self):
        b=bridge();b.send_goal();b.output()
        b.sock.sendto.assert_called_once();goal=json.loads(b.sock.sendto.call_args.args[0])
        self.assertEqual(goal['p'],[.4,0.,.5]);self.assertEqual(goal['stamp_ns'],b.get_clock().now().nanoseconds)
        b.jtc.publish.assert_not_called()

    def test_stale_hand_or_status_only_sends_measured_hold(self):
        from geometry_msgs.msg import PoseStamped
        for stale_status in (False,True):
            b=bridge();b.status['phase']='active';b.command=PoseStamped()
            b.command.header.frame_id='fr3_link0';b.command.pose.position.x=9.
            stamp=b.get_clock().now().nanoseconds-(0 if stale_status else 300000000)
            b.command.header.stamp.sec,b.command.header.stamp.nanosec=divmod(stamp,10**9)
            if stale_status:b.status_at-=1.
            b.send_goal();goal=json.loads(b.sock.sendto.call_args.args[0])
            self.assertEqual(goal['p'],[.4,0.,.5]);self.assertEqual(goal['stamp_ns'],b.get_clock().now().nanoseconds)

    def test_stale_measured_hold_is_never_restamped(self):
        b=bridge();b.tf.lookup_transform.return_value.header.stamp.sec-=1
        b.send_goal();b.sock.sendto.assert_not_called();b.output();b.jtc.publish.assert_not_called()

    def test_status_matches_valid_continuation_and_sent_packet(self):
        b=bridge();b.source='joystick';b.last_sent=copy.deepcopy(b.last_packet)
        b.last_sent['stamp_ns']-=20000000;b.last_sent['q'][0]=.01;b.last_packet['q'][0]=.02
        b.publish_status();self.assertTrue(b.http_state['mode_ready'],b.http_state)
        b.output();b.jtc.publish.assert_called_once()
        b.publish_status();self.assertTrue(b.http_state['mode_ready'],b.http_state)

    def test_paused_faults_still_refuse_readiness(self):
        for fault,reason in [('feedback','robot joint feedback stale'),('packet','MPC solution infeasible'),('owner','unexpected joint command publisher ownership')]:
            b=bridge()
            if fault=='feedback':b.q_at-=1.
            if fault=='packet':b.last_packet['feasible']=False
            if fault=='owner':b.get_publishers_info_by_topic=lambda topic:[1,2]
            b.publish_status();self.assertFalse(b.http_state['mode_ready'])
            self.assertEqual(b.http_state['ready_reason'],reason)
            b.output();b.jtc.publish.assert_not_called()


class CoupledPauseTests(unittest.TestCase):
    setUpClass=classmethod(input_tests.InputTests.setUpClass.__func__)
    tearDownClass=classmethod(input_tests.InputTests.tearDownClass.__func__)
    setUp=input_tests.InputTests.setUp
    tearDown=input_tests.InputTests.tearDown
    sample=input_tests.InputTests.sample
    packet3=input_tests.InputTests.packet3
    tick=input_tests.InputTests.tick

    def status(self,b):
        b.publish_status();self.n.curobo_status=b.http_state;self.n.curobo_status_at=time.monotonic()

    def test_pause_beyond_goal_timeout_and_fresh_recovery(self):
        b=bridge();b.status['phase']='active';self.n.mpc_routed=True;self.status(b)
        self.packet3(age=.30);self.assertTrue(self.n.engage()[0]);self.n._publish.reset_mock()
        self.n.zed_received-=1.;self.tick();self.assertEqual(self.n.zed_phase,'paused')
        # Simulate 600 ms of pause with fresh measured TF and new planner results.
        start=b.get_clock().now().nanoseconds
        for step in range(1,7):
            now=start+step*100000000;b.get_clock=lambda now=now:NS(now=lambda:NS(nanoseconds=now))
            tr=b.tf.lookup_transform.return_value;tr.header.stamp.sec,tr.header.stamp.nanosec=divmod(now,10**9)
            b.status['phase']=self.n.zed_phase;b.send_goal()
            self.assertTrue(b.sock.sendto.called,'planner hold heartbeat missing')
            goal=json.loads(b.sock.sendto.call_args.args[0]);self.assertEqual(goal['p'],[.4,0.,.5])
            for key in ('stamp_ns','joint_stamp_ns','esdf_stamp_ns','depth_stamp_ns'):b.last_packet[key]=now-1000000
            b.last_packet['goal_stamp_ns']=goal['stamp_ns'];b.output();self.status(b);self.tick()
            self.assertTrue(self.n.engaged,self.n.zed_note);b.jtc.publish.assert_not_called()
        self.n._publish.assert_not_called()
        self.packet3(x=.28,age=.15);self.tick();self.assertEqual(self.n.zed_phase,'paused')
        self.packet3(x=.33);self.tick();self.assertEqual(self.n.zed_phase,'active')
        np.testing.assert_allclose(self.n.cmd_p,[.4,0.,.4]);self.n._publish.assert_called_once()

    def test_reviewed_full_6d_mapping_and_speed_are_preserved(self):
        from fr3_input_teleop import qexp, qangle
        from cartesian_review import target
        self.n.zed_mapping_profile='review';self.n.zed_review_exercise='full_6d'
        self.n.speed_scale=.1
        self.packet3(q=[0.,0.,0.,1.]);self.assertTrue(self.n.engage()[0])
        anchor=self.n.zed_anchor
        self.packet3(x=.23,q=qexp(np.array([.2,.15,.1])).tolist());self.tick()
        expected_p,expected_q=target(*anchor,self.n.zed_sample[0],self.n.zed_sample[1],
                                     [self.n.zed_scale]*3,1.,'full_6d')
        np.testing.assert_allclose(self.n.tgt_p,expected_p)
        self.assertLess(qangle(self.n.tgt_q,expected_q),1e-7)
        self.assertEqual(self.n.speed_scale,.1)

    def test_paused_hard_faults_disengage_teleop(self):
        for fault in ('feedback','packet','owner'):
            b=bridge();self.n.mpc_routed=True;self.status(b)
            self.packet3();self.assertTrue(self.n.engage()[0]);self.n._publish.reset_mock()
            self.n.zed_received-=1.;self.tick();self.assertEqual(self.n.zed_phase,'paused')
            if fault=='feedback':b.q_at-=1.
            if fault=='packet':b.last_packet['feasible']=False
            if fault=='owner':b.get_publishers_info_by_topic=lambda topic:[1,2]
            self.status(b);self.tick();self.assertFalse(self.n.engaged)
            self.assertEqual(self.n.zed_note,b.http_state['ready_reason']);self.n._publish.assert_not_called()

    def test_precise_fault_and_obsolete_note_clear_without_masking_new_fault(self):
        b=bridge();b.last_packet['feasible']=False;self.n.mpc_routed=True;self.status(b)
        self.assertEqual(self.n._zed_controller_ready(),'MPC solution infeasible')
        self.n.engaged=True;self.tick();self.assertEqual(self.n.zed_note,'MPC solution infeasible')
        b.last_packet['feasible']=True;self.status(b);self.n._zed_status()
        self.assertNotEqual(self.n.zed_note,'MPC solution infeasible')
        b.q_at-=1.;self.status(b);self.n._zed_status()
        self.assertEqual(self.n._zed_controller_ready(),'robot joint feedback stale')
