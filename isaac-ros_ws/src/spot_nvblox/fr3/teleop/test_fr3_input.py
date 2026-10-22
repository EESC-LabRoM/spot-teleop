"""Run in fr3-ros2: isolated DDS domains 191/192, fake robot poses, no hardware writes."""
import math
import json
import socket
import time
import unittest
from unittest.mock import Mock, patch
import numpy as np
import rclpy
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped
import fr3_input_teleop as T


class InputTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(domain_id=191, args=['--ros-args', '-p', 'zed_orientation_mode:=roll', '-p', 'zed_port:=47814', '-p', 'joy_device:=""',
                                     '-p', 'remote_joy_topic:=/joy_remote'])

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.n = T.Fr3JoyTeleop()
        self.n._measured = lambda *a: (np.array([.4, 0., .4]) if not a else np.zeros(3), np.array([0., 0., 0., 1.]), 0.)
        self.n._publish = Mock()
        self.sub = self.n.create_subscription(PoseStamped, '/cartesian_impedance_target_controller/equilibrium_pose', lambda m: None, 1)
        self.n.ctrl_active, self.n.ctrl_checked = True, time.monotonic()
        self.n.set_parameters([Parameter('input_source', value='zed')])
        self.n._use_layout('xinput')
        self.n._home_running = lambda: False

    def tearDown(self):
        self.n.destroy_node()

    def sample(self, x=.2, q=None, frame='fr3_link0', age=0.):
        m = PoseStamped(); m.header.frame_id = frame
        stamp = self.n.get_clock().now().nanoseconds - int(age * 1e9)
        m.header.stamp.sec, m.header.stamp.nanosec = divmod(stamp, 10**9)
        m.pose.position.x = x; m.pose.position.z = .4
        q = q or [0., 0., 0., 1.]
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = q
        self.n._on_zed(m); self.n.roll_received = time.monotonic()
        return m

    def engage(self):
        self.sample(); ok, why = self.n.engage(); self.assertTrue(ok, why)

    def tick(self):
        self.n.t_last = time.monotonic() - .02; self.n.tick()

    def test_start_holds_and_switches_an_active_joystick(self):
        self.n.set_parameters([Parameter('input_source',value='joystick')]);self.n.engaged=True
        self.n._publish.reset_mock();self.assertTrue(self.n.start_zed()[0])
        self.assertFalse(self.n.engaged);self.assertEqual(self.n.input_source,'zed');self.assertEqual(self.n.zed_phase,'calibrating')
        self.n._publish.assert_not_called()

    def test_person_calibration_request_once_and_start_idempotent(self):
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as receiver:
            receiver.bind(('127.0.0.1',0));receiver.settimeout(.05)
            self.n.zed_calibration_port=receiver.getsockname()[1]
            self.sample();self.assertTrue(self.n.start_zed()[0])
            self.assertEqual(json.loads(receiver.recv(512)),dict(action='calibrate_arm',version=1))
            self.assertIsNone(self.n.zed_sample)
            self.assertTrue(self.n.start_zed()[0])
            with self.assertRaises(socket.timeout):receiver.recv(512)
        self.n._publish.assert_not_called()

    def test_diagnostics_never_make_tracking_live(self):
        stamp=self.n.get_clock().now().nanoseconds
        packet=dict(kind='tracking',version=1,stamp_ns=stamp,diagnostics=dict(valid=True,arm_samples=45))
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sender:
            sender.sendto(json.dumps(packet).encode(),('127.0.0.1',47814))
        self.n._receive_zed()
        self.assertFalse(self.n._zed_live());self.assertEqual(self.n.zed_diagnostics['arm_samples'],45)
        self.n._publish.assert_not_called()

    def test_longer_recovery_window_holds_then_expires(self):
        self.n.zed_recovery_timeout=5.
        self.engage();self.n._publish.reset_mock();self.n.zed_received-=2.;self.tick()
        self.assertTrue(self.n.engaged);self.assertEqual(self.n.zed_phase,'paused');self.n._publish.assert_not_called()
        self.n.zed_received-=4.;self.n.zed_loss_since-=4.;self.tick()
        self.assertFalse(self.n.engaged);self.n._publish.assert_not_called()

    def test_udp_bridge_receives_pose_without_robot_publish(self):
        stamp = self.n.get_clock().now().nanoseconds
        d = dict(version=1,stamp_ns=stamp,roll_stamp_ns=stamp,frame='fr3_link0',p=[.2,0.,.4],q=[0.,0.,0.,1.])
        sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        sock.sendto(json.dumps(d).encode(),('127.0.0.1',47814));sock.close()
        self.n._receive_zed()
        self.assertTrue(self.n._zed_live());self.n._publish.assert_not_called()

    def test_malformed_udp_cannot_enable_tracking(self):
        sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        sock.sendto(b'{"version":1,"p":[]}',('127.0.0.1',47814));sock.close()
        self.n._receive_zed();self.assertFalse(self.n._zed_live())

    def test_full_6d_pitch_and_yaw_use_base_axes_with_rotated_tool(self):
        self.n.zed_orientation_mode='full_6d'
        anchored=T.qexp(np.array([math.pi/2,0.,0.]))
        self.n._measured=lambda *a:(np.array([.4,0.,.4]) if not a else np.zeros(3),anchored if not a else np.array([0.,0.,0.,1.]),0.)
        self.engage()
        for axis in (np.array([0.,1.,0.]),np.array([0.,0.,1.])):
            self.sample(q=T.qexp(.2*axis).tolist());self.tick()
            expected=T.qmul(T.qexp(.2*axis),anchored)
            # The limited command and target must move toward a base-axis delta, not a tool-axis delta.
            self.assertLess(T.qangle(self.n.tgt_q,expected),T.qangle(anchored,expected))
            wrong=T.qmul(anchored,T.qexp(.2*axis))
            self.assertLess(T.qangle(self.n.tgt_q,expected),T.qangle(self.n.tgt_q,wrong))

    def test_full_6d_rejects_legacy_roll_packet(self):
        self.n.zed_orientation_mode='full_6d'
        stamp=self.n.get_clock().now().nanoseconds
        d=dict(version=1,stamp_ns=stamp,roll_stamp_ns=stamp,frame='fr3_link0',p=[.2,0.,.4],q=[0.,0.,0.,1.])
        sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);sock.sendto(json.dumps(d).encode(),('127.0.0.1',47814));sock.close()
        self.n._receive_zed();self.assertFalse(self.n._zed_live())

    def test_full_6d_accepts_only_paired_orientation(self):
        self.n.zed_orientation_mode='full_6d'
        stamp=self.n.get_clock().now().nanoseconds
        d=dict(version=2,orientation_mode='full_6d',stamp_ns=stamp,orientation_stamp_ns=stamp,frame='fr3_link0',p=[.2,0.,.4],q=[0.,0.,0.,1.])
        sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);sock.sendto(json.dumps(d).encode(),('127.0.0.1',47814))
        self.n._receive_zed();self.assertTrue(self.n._zed_live())
        d['orientation_stamp_ns']=stamp-1;sock.sendto(json.dumps(d).encode(),('127.0.0.1',47814));sock.close()
        self.n._receive_zed();self.assertFalse(self.n._zed_live())

    def test_zed_uses_joystick_horizontal_and_vertical_ceilings(self):
        self.n.v_lin=.6;self.n.v_z=.4;self.n.speed_scale=.5;self.n.max_step=.02
        self.engage();self.n.zed_sample=(np.array([1.,1.,1.]),np.array([0.,0.,0.,1.]))
        p=self.n.cmd_p.copy();self.tick();step=self.n.cmd_p-p
        self.assertAlmostEqual(np.linalg.norm(step[:2]),.6*.5*.02,delta=.00015)
        self.assertAlmostEqual(step[2],.4*.5*.02,delta=.00015)
        self.assertLessEqual(np.linalg.norm(step),self.n.max_step)

    def test_zed_rotation_uses_joystick_angular_ceiling(self):
        self.n.w_ang=math.radians(90);self.n.rot_scale=.5;self.n.max_slew=math.radians(4)
        self.engage();self.sample(q=T.qexp(np.array([0.,.5,0.])).tolist())
        self.tick();angle=T.qangle(self.n.cmd_q,np.array([0.,0.,0.,1.]))
        self.assertAlmostEqual(angle,math.radians(90)*.5*.02,delta=math.radians(.025))

    def test_zero_dashboard_scales_stop_zed_translation_and_rotation(self):
        self.engage();self.n.speed_scale=0.;self.n.rot_scale=0.
        self.sample(x=.7,q=T.qexp(np.array([.2,.2,.2])).tolist())
        p=self.n.cmd_p.copy();q=self.n.cmd_q.copy();self.tick()
        np.testing.assert_allclose(self.n.cmd_p,p);np.testing.assert_allclose(self.n.cmd_q,q)

    def test_speed_dashboard_reports_same_limits_for_both_inputs(self):
        self.n.scale_pub=Mock()
        self.n.input_source='joystick';self.n._pub_scale();a=self.n.scale_pub.publish.call_args[0][0].data
        self.n.input_source='zed';self.n._pub_scale();b=self.n.scale_pub.publish.call_args[0][0].data
        np.testing.assert_allclose(a,b)

    def test_zed_button_cannot_engage_joystick_after_source_switch(self):
        self.sample();self.n.set_parameters([Parameter('input_source',value='joystick')])
        self.assertFalse(self.n.engage_zed()[0]);self.n._publish.assert_not_called()

    def test_zed_button_uses_existing_engage_checks(self):
        self.assertFalse(self.n.engage_zed()[0])
        self.sample();self.assertTrue(self.n.engage_zed()[0])

    def test_no_camera_cannot_engage(self):
        self.assertFalse(self.n.engage()[0])
        self.n._publish.assert_not_called()

    def test_engage_has_no_pose_jump(self):
        self.engage(); np.testing.assert_allclose(self.n.cmd_p, [.4, 0., .4])
        self.tick(); np.testing.assert_allclose(self.n.cmd_p, [.4, 0., .4])

    def test_relative_motion_and_speed_cap(self):
        self.engage(); self.sample(x=.7)
        p = self.n.cmd_p.copy(); self.tick()
        self.assertLess(self.n.cmd_p[0], p[0])
        self.assertLessEqual(np.linalg.norm(self.n.cmd_p-p), self.n.v_lin*self.n.speed_scale*.023)
        self.assertLessEqual(np.linalg.norm(self.n.tgt_p-self.n.cmd_p), .051)

    def test_forward_toward_table_backward_away_and_other_axes_preserved(self):
        self.engage()
        self.assertEqual(self.n.zed_forward_sign,-1.)
        self.n.zed_sample=(np.array([.22,.02,.42]),np.array([0.,0.,0.,1.]))
        self.tick()
        np.testing.assert_allclose(self.n.tgt_p,[.38,.02,.42],atol=1e-8)
        self.sample(x=.18);self.tick()
        np.testing.assert_allclose(self.n.tgt_p,[.42,0.,.4],atol=1e-8)

    def test_axis_sign_cannot_change_during_a_session(self):
        self.engage()
        result=self.n.set_parameters([Parameter('zed_forward_sign',value=1.)])
        self.assertFalse(result[0].successful);self.assertEqual(self.n.zed_forward_sign,-1.)

    def test_roll_relative_to_robot_and_rate_limited(self):
        self.engage(); self.sample(q=[math.sin(.5),0.,0.,math.cos(.5)])
        self.tick()
        self.assertGreater(T.qangle(self.n.cmd_q, np.array([0.,0.,0.,1.])), 0.)
        self.assertLessEqual(T.qangle(self.n.cmd_q, np.array([0.,0.,0.,1.])), self.n.w_ang*self.n.rot_scale*.023)

    def test_stale_camera_disengages_and_does_not_resume(self):
        self.engage(); self.n.zed_received -= 2; self.n._publish.reset_mock()
        self.tick(); self.assertFalse(self.n.engaged); self.n._publish.assert_not_called()
        self.sample(); self.tick(); self.assertFalse(self.n.engaged)

    def test_stale_hand_disengages(self):
        self.engage(); self.n.roll_received -= 2; self.tick(); self.assertFalse(self.n.engaged)

    def test_start_selects_zed_and_waits_for_new_stable_frames(self):
        self.n.set_parameters([Parameter('input_source',value='joystick')])
        self.sample()
        self.assertTrue(self.n.start_zed()[0])
        self.assertEqual(self.n.input_source,'zed')
        self.assertFalse(self.n.engaged)
        self.n._publish.assert_not_called()
        now=time.monotonic()
        for i in range(5):
            self.sample();self.n._advance_zed_start(now+i*.18)
        self.assertTrue(self.n.engaged)
        self.assertEqual(self.n.zed_phase,'active')
        np.testing.assert_allclose(self.n.cmd_p,[.4,0.,.4])

    def test_stop_cancels_pending_calibration(self):
        self.n.start_zed();self.n.stop_teleop()
        for i in range(6):
            self.sample();self.n._advance_zed_start(time.monotonic()+i*.2)
        self.assertFalse(self.n.engaged);self.n._publish.assert_not_called()

    def test_start_timeout_cannot_arm_later(self):
        self.n.start_zed();self.n.zed_deadline=0.
        self.n._advance_zed_start(time.monotonic());self.sample();self.tick()
        self.assertEqual(self.n.zed_phase,'idle');self.assertFalse(self.n.engaged)

    def test_single_short_gap_keeps_original_reference(self):
        self.engage();anchor=self.n.zed_anchor[2].copy();self.n._publish.reset_mock()
        self.n.zed_received-=.4;self.tick();self.assertEqual(self.n.zed_phase,'paused')
        self.sample(x=.21);self.tick()
        self.assertEqual(self.n.zed_phase,'active');np.testing.assert_allclose(self.n.zed_anchor[2],anchor)
        self.n._publish.assert_not_called()

    def test_short_loss_holds_without_commands_then_reanchors(self):
        self.engage();self.n._publish.reset_mock()
        self.n.zed_received-=.5;self.tick()
        self.assertTrue(self.n.engaged);self.assertEqual(self.n.zed_phase,'paused')
        self.n._publish.assert_not_called()
        self.sample(x=.6)
        now=time.monotonic();p,q=self.n.zed_sample
        self.n.zed_stable=[(now-.7+i*.2,p.copy(),q.copy()) for i in range(4)]
        self.n.zed_stable_stamp=-1
        self.tick()
        self.assertEqual(self.n.zed_phase,'active')
        np.testing.assert_allclose(self.n.cmd_p,[.4,0.,.4])
        np.testing.assert_allclose(self.n.zed_anchor[2],p)

    def test_gesture_is_paired_with_fresh_full_pose_and_confidence(self):
        self.n.zed_orientation_mode='full_6d'
        stamp=self.n.get_clock().now().nanoseconds
        d=dict(version=2,orientation_mode='full_6d',stamp_ns=stamp,orientation_stamp_ns=stamp,
               frame='fr3_link0',p=[.2,0.,.4],q=[0.,0.,0.,1.],gesture='Closed_Fist',gesture_score=.9)
        sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        sock.sendto(json.dumps(d).encode(),('127.0.0.1',47814));self.n._receive_zed()
        self.assertEqual(self.n.zed_gesture,'Closed_Fist');self.assertEqual(self.n.zed_gesture_stamp,stamp)
        d.update(stamp_ns=stamp+1,orientation_stamp_ns=stamp+1,gesture_score=.2)
        sock.sendto(json.dumps(d).encode(),('127.0.0.1',47814));self.n._receive_zed();sock.close()
        self.assertIsNone(self.n.zed_gesture);self.n._publish.assert_not_called()

    def test_gesture_commands_only_stable_edges_while_active(self):
        self.engage();self.n._gripper=Mock();now=time.monotonic()
        for i in range(9):
            self.n.zed_gesture='Closed_Fist';self.n.zed_gesture_stamp+=1
            self.n._zed_gripper(now+i*.15)
        self.n._gripper.assert_called_once_with(True)
        for i in range(4):
            self.n.zed_gesture='Open_Palm';self.n.zed_gesture_stamp+=1
            self.n._zed_gripper(now+1.5+i*.15)
        self.assertEqual(self.n._gripper.call_count,2)
        self.n._gripper.assert_called_with(False)

    def test_gestures_never_run_while_stopped_calibrating_paused_or_stale(self):
        self.engage();self.n._gripper=Mock();now=time.monotonic()
        for phase in ('idle','calibrating','paused'):
            self.n.zed_phase=phase
            for i in range(5):
                self.n.zed_gesture='Closed_Fist';self.n.zed_gesture_stamp+=1
                self.n._zed_gripper(now+i*.2)
        self.n.zed_phase='active';self.n.zed_received-=1.
        for i in range(5):
            self.n.zed_gesture_stamp+=1;self.n._zed_gripper(now+i*.2)
        self.n._gripper.assert_not_called()

    def test_uncertain_or_replayed_gesture_cannot_trigger(self):
        self.engage();self.n._gripper=Mock();now=time.monotonic()
        for i in range(10):
            self.n.zed_gesture='Closed_Fist' if i%2 else None
            self.n.zed_gesture_stamp+=1;self.n._zed_gripper(now+i*.15)
        self.n.zed_gesture='Closed_Fist'
        for i in range(10):self.n._zed_gripper(now+2+i*.15)
        self.n._gripper.assert_not_called()

    def test_home_cancels_calibration_and_stop_signals_home(self):
        self.n.start_zed()
        with patch.object(T.subprocess,'Popen') as launch:
            self.assertTrue(self.n.start_home()[0])
            launch.assert_called_once_with(['python3',self.n.home_script], stdout=self.n.home_output, stderr=T.subprocess.STDOUT)
            self.n.home_output.close(); self.n.home_output=None
        self.assertEqual(self.n.zed_phase,'idle');self.assertFalse(self.n.engaged)
        self.n._publish.assert_not_called()
        self.n._home_running=lambda:True
        self.n.home_proc=Mock()
        self.n.stop_teleop()
        self.n.home_proc.send_signal.assert_called_once_with(T.signal.SIGINT)

    def test_home_failure_remains_visible_after_process_exits(self):
        import tempfile
        with tempfile.NamedTemporaryFile(mode='w+') as log:
            log.write('HOME FAILED: robot_mode 4: stopped\n'); log.flush()
            self.n.home_output=log;self.n.home_proc=Mock()
            self.n.home_proc.poll.return_value=1;self.n.home_proc.returncode=1
            self.assertFalse(T.Fr3JoyTeleop._home_running(self.n))
            self.assertIn('robot_mode 4',self.n.home_result)
            self.assertIsNone(self.n.home_proc)
            self.assertFalse(T.Fr3JoyTeleop._home_running(self.n))
            self.assertIn('robot_mode 4',self.n.home_result)

    def test_source_switch_cancels_pending_start(self):
        self.n.start_zed();self.n.set_parameters([Parameter('input_source',value='joystick')])
        self.sample();self.tick()
        self.assertFalse(self.n.engaged);self.assertEqual(self.n.zed_phase,'idle')

    def test_invalid_samples_rejected(self):
        for kw in [{'x':float('nan')}, {'frame':'body'}, {'q':[0.,0.,0.,0.]}, {'age':2.}, {'age':-1.}]:
            self.sample(**kw); self.assertIsNone(self.n.zed_sample, kw)

    def test_replayed_sample_does_not_renew(self):
        m = self.sample(); old = self.n.zed_received; time.sleep(.01)
        self.n._on_zed(m); self.assertEqual(self.n.zed_received, old)

    def test_source_change_disengages(self):
        self.engage(); self.n.set_parameters([Parameter('input_source', value='joystick')])
        self.assertFalse(self.n.engaged); self.assertEqual(self.n.input_source, 'joystick')

    def test_local_back_stops_zed(self):
        self.engage(); self.n.buttons = {6:1}; self.tick(); self.assertFalse(self.n.engaged)

    def test_robot_stale_disengages(self):
        self.engage(); self.n._measured = lambda *a: (np.zeros(3),np.array([0.,0.,0.,1.]),1.)
        self.tick(); self.assertFalse(self.n.engaged)

    def test_competing_writer_refuses_engage(self):
        self.sample()
        other = self.n.create_publisher(PoseStamped, '/cartesian_impedance_target_controller/equilibrium_pose', 1)
        self.assertFalse(self.n.engage()[0]); self.n.destroy_publisher(other)

    def test_inactive_controller_refuses_engage(self):
        self.sample(); self.n.ctrl_active = False; self.assertFalse(self.n.engage()[0])

    def test_joystick_still_integrates(self):
        self.n.set_parameters([Parameter('input_source', value='joystick')]); self.assertTrue(self.n.engage()[0])
        self.n.axes = {1:-1.}; p = self.n.cmd_p.copy(); self.tick()
        self.assertGreater(self.n.cmd_p[0],p[0])

if __name__ == '__main__': unittest.main(verbosity=2)
