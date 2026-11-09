"""Unit tests for run_take.py pure helpers (no ROS needed)."""
import json
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import run_take


class SelfSafeTest(unittest.TestCase):
    def test_brackets_first_char(self):
        self.assertEqual(run_take.self_safe('fr3_bagtest'), '[f]r3_bagtest')
        # The guard must not match its own pgrep command line.
        self.assertNotIn('fr3_bagtest', '[f]r3_bagtest')


class ParseSamplesTest(unittest.TestCase):
    def write(self, lines):
        f = tempfile.NamedTemporaryFile('w', suffix='.jsonl', delete=False)
        for line in lines:
            f.write(json.dumps(line) + '\n')
        f.close()
        self.addCleanup(os.unlink, f.name)
        return f.name

    def test_new_and_recorded(self):
        path = self.write([
            dict(wall_t=100.0, topic='/fr3_bagtest/teleop_sample',
                 data=dict(position_valid=True, orientation_valid=True, p=[1, 2, 3],
                           q=[0, 0, 0, 1], gesture='Open_Palm')),
            dict(wall_t=100.1, topic='/fr3_bagtest/teleop_sample',
                 data=dict(position_valid=False, p=[0, 0, 0])),
            dict(wall_t=100.05, topic='/fr3_bagtest/teleop_sample',
                 data=dict(position_valid=True, orientation_valid=True, p=[1, 2, 3])),
            dict(wall_t=100.0, topic='/fr3_bagtest/recorded_sample',
                 data=dict(position_valid=True, p=[1.01, 2, 3])),
        ])
        from pathlib import Path
        new, recorded = run_take.parse_samples(Path(path))
        self.assertEqual(len(new), 2)
        self.assertEqual(len(recorded), 1)
        self.assertAlmostEqual(new[0]['offset_s'], 0.0)
        self.assertAlmostEqual(recorded[0]['offset_s'], 0.0)
        self.assertTrue(new[0]['orientation_valid'])
        self.assertEqual(new[0]['q'], [0, 0, 0, 1])
        # orientation claimed valid but no quaternion: treated as invalid
        self.assertFalse(new[1]['orientation_valid'])
        self.assertIsNone(new[1]['q'])

    def test_scored_from_filters_warmup(self):
        path = self.write([
            dict(wall_t=90.0, topic='/fr3_bagtest/teleop_sample',
                 data=dict(position_valid=True, p=[9, 9, 9])),
            dict(wall_t=100.0, topic='/fr3_bagtest/teleop_sample',
                 data=dict(position_valid=True, p=[1, 2, 3])),
        ])
        from pathlib import Path
        new, _ = run_take.parse_samples(Path(path), scored_from=95.0)
        self.assertEqual(len(new), 1)
        self.assertEqual(new[0]['p'], [1, 2, 3])

    def test_missing_file(self):
        from pathlib import Path
        new, recorded = run_take.parse_samples(Path('/nonexistent/x.jsonl'))
        self.assertEqual((new, recorded), ([], []))


class ParseRobotTest(unittest.TestCase):
    def test_series_grips_engaged(self):
        f = tempfile.NamedTemporaryFile('w', suffix='.jsonl', delete=False)
        rows = [
            dict(wall_t=1.0, topic='/franka_robot_state_broadcaster/current_pose',
                 data=dict(frame='fr3_link0', p=[0.3, 0, 0.48], q=[0, 0, 0, 1])),
            dict(wall_t=1.1, topic='/franka_robot_state_broadcaster/current_pose',
                 data=dict(frame='fr3_link0', p=[0.31, 0, 0.48], q=[0, 0, 0, 1])),
            dict(wall_t=1.0, topic='/franka_gripper/joint_states',
                 data=dict(name=['a'], position=[0.05])),
            dict(wall_t=1.0, topic='/teleop/engaged', data=True),
        ]
        for r in rows:
            f.write(json.dumps(r) + '\n')
        f.close()
        self.addCleanup(os.unlink, f.name)
        from pathlib import Path
        series, grips, engaged = run_take.parse_robot(Path(path := f.name))
        self.assertEqual(len(series), 2)
        self.assertEqual(grips, [0.05])
        self.assertTrue(engaged)

    def test_scored_to_cuts_home_return(self):
        f = tempfile.NamedTemporaryFile('w', suffix='.jsonl', delete=False)
        rows = [
            dict(wall_t=1.0, topic='/franka_robot_state_broadcaster/current_pose',
                 data=dict(frame='fr3_link0', p=[0.3, 0, 0.48], q=[0, 0, 0, 1])),
            dict(wall_t=2.0, topic='/franka_robot_state_broadcaster/current_pose',
                 data=dict(frame='fr3_link0', p=[0.3, 0, 0.43], q=[0, 0, 0, 1])),
            dict(wall_t=3.0, topic='/franka_robot_state_broadcaster/current_pose',
                 data=dict(frame='fr3_link0', p=[0.3, 0, 0.49], q=[0, 0, 0, 1])),
            dict(wall_t=1.5, topic='/teleop/engaged', data=True),
        ]
        for r in rows:
            f.write(json.dumps(r) + '\n')
        f.close()
        self.addCleanup(os.unlink, f.name)
        from pathlib import Path
        series, _, engaged = run_take.parse_robot(Path(f.name), scored_from=0.0, scored_to=2.0)
        self.assertEqual([s['p'][2] for s in series], [0.48, 0.43])
        self.assertTrue(engaged)

    def test_bad_frame_rejected(self):
        f = tempfile.NamedTemporaryFile('w', suffix='.jsonl', delete=False)
        f.write(json.dumps(dict(wall_t=1.0, topic='/franka_robot_state_broadcaster/current_pose',
                                data=dict(frame='map', p=[0, 0, 0], q=[0, 0, 0, 1]))) + '\n')
        f.close()
        self.addCleanup(os.unlink, f.name)
        from pathlib import Path
        with self.assertRaises(RuntimeError):
            run_take.parse_robot(Path(f.name))


class ParsePostStopTest(unittest.TestCase):
    def write(self, rows):
        f = tempfile.NamedTemporaryFile('w', suffix='.jsonl', delete=False)
        for r in rows:
            f.write(json.dumps(r) + '\n')
        f.close()
        self.addCleanup(os.unlink, f.name)
        from pathlib import Path
        return Path(f.name)

    def pose(self, t, topic, p, frame='fr3_link0'):
        return dict(wall_t=t, topic=topic, data=dict(frame=frame, p=p, q=[0, 0, 0, 1]))

    def test_hold_clean(self):
        path = self.write([
            dict(wall_t=1.0, topic='/teleop/engaged', data=False),
            dict(wall_t=2.0, topic='/teleop/engaged', data=True),
            self.pose(2.0, '/teleop/command_tool', [0.30, 0.0, 0.50]),
            self.pose(2.0, '/franka_robot_state_broadcaster/current_pose', [0.30, 0.0, 0.49]),
            self.pose(3.0, '/teleop/command_tool', [0.30, 0.0, 0.52]),
            self.pose(3.0, '/franka_robot_state_broadcaster/current_pose', [0.30, 0.0, 0.515]),
            dict(wall_t=4.0, topic='/teleop/engaged', data=False),
            self.pose(5.0, '/franka_robot_state_broadcaster/current_pose', [0.30, 0.0, 0.516]),
            self.pose(6.0, '/franka_robot_state_broadcaster/current_pose', [0.30, 0.0, 0.514]),
        ])
        post = run_take.parse_post_stop(path, scored_from=0.0,
                                        t_stop_confirmed=4.0, t_home_start=9.0)
        self.assertTrue(post['ok'])
        self.assertEqual(post['t_disengage'], 4.0)
        self.assertAlmostEqual(post['tracking_err_at_stop_cm'], 0.5)
        self.assertAlmostEqual(post['post_stop_drift_cm'], 0.1)
        self.assertEqual(post['commands_after_stop'], 0)
        self.assertEqual(post['n_hold_points'], 2)
        self.assertEqual(post['target_frame'], 'fr3_link0')
        line = run_take.post_stop_line(post)
        self.assertIn('cmds_after_stop=0', line)

    def test_stale_drift_visible(self):
        # TCP keeps travelling toward the last target after disengage.
        path = self.write([
            dict(wall_t=2.0, topic='/teleop/engaged', data=True),
            self.pose(3.0, '/teleop/command_tool', [0.30, 0.0, 0.60]),
            self.pose(3.0, '/franka_robot_state_broadcaster/current_pose', [0.30, 0.0, 0.50]),
            dict(wall_t=4.0, topic='/teleop/engaged', data=False),
            self.pose(5.0, '/franka_robot_state_broadcaster/current_pose', [0.30, 0.0, 0.55]),
            self.pose(6.0, '/teleop/command_tool', [0.30, 0.0, 0.60]),
        ])
        post = run_take.parse_post_stop(path, t_home_start=9.0)
        self.assertTrue(post['ok'])
        self.assertAlmostEqual(post['tracking_err_at_stop_cm'], 10.0)
        self.assertAlmostEqual(post['post_stop_drift_cm'], 5.0)
        self.assertEqual(post['commands_after_stop'], 1)

    def test_no_transition(self):
        path = self.write([
            dict(wall_t=2.0, topic='/teleop/engaged', data=True),
            self.pose(3.0, '/franka_robot_state_broadcaster/current_pose', [0.3, 0, 0.5]),
        ])
        post = run_take.parse_post_stop(path)
        self.assertFalse(post['ok'])
        self.assertIn('NO DATA', run_take.post_stop_line(post))

    def test_missing_file(self):
        from pathlib import Path
        post = run_take.parse_post_stop(Path('/nonexistent/r.jsonl'))
        self.assertFalse(post['ok'])


class CleanupCoverageTest(unittest.TestCase):
    def test_serve_helper_reaped(self):
        # RobotLink.close() kills only the host-side docker-exec client; the
        # in-container serve keeps spinning (~43% CPU each; 4 strays leaked
        # 2026-11-03) unless every cleanup path reaps it.
        serve = ('fr3-ros2', 'bagtest_robot_side.py serve')
        self.assertIn(serve, run_take.take_cleanup_procs(True))
        self.assertIn(serve, run_take.take_cleanup_procs(False))
        self.assertIn(serve, run_take.CHAIN_DOWN_PROCS)


if __name__ == '__main__':
    unittest.main()
