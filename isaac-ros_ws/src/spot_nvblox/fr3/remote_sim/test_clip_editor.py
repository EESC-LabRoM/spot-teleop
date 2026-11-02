import math
import json,tempfile,threading
from pathlib import Path
from unittest.mock import patch
import clip_editor
import unittest
from clip_editor import byte_range, validate_cut


class EditorTests(unittest.TestCase):
    def test_review_feedback_persists_without_robot_pose_annotations(self):
        with tempfile.TemporaryDirectory() as folder,patch.object(clip_editor,'OUTPUT',Path(folder)):
            editor=clip_editor.Editor.__new__(clip_editor.Editor);editor.lock=threading.RLock();editor.sources={'source':{}}
            result=editor.review_feedback(dict(source_id='source',status='wrong',note='inverter horizontal'))
            stored=json.loads((Path(folder)/'review_feedback.json').read_text())
            self.assertEqual(stored['source']['note'],'inverter horizontal');self.assertEqual(result['feedback']['status'],'wrong')
            self.assertFalse((Path(folder)/'demonstrations.json').exists())
    def test_feedback_for_old_video_is_not_applied_to_updated_run(self):
        with tempfile.TemporaryDirectory() as folder,patch.object(clip_editor,'OUTPUT',Path(folder)):
            editor=clip_editor.Editor.__new__(clip_editor.Editor);editor.lock=threading.RLock();editor.sources={'source':{}}
            editor.review=lambda dataset='realsense':dict(runs=[dict(source_id='source',job_id='new')])
            with self.assertRaises(ValueError):editor.review_feedback(dict(source_id='source',status='correct',job_id='old'))
            self.assertFalse((Path(folder)/'review_feedback.json').exists())
            editor.review_feedback(dict(source_id='source',status='wrong',job_id='new',note='menos giro'))
            self.assertEqual(json.loads((Path(folder)/'review_feedback.json').read_text())['source']['job_id'],'new')

    def test_zed_review_uses_separate_dataset_and_preserves_realsense(self):
        with tempfile.TemporaryDirectory() as folder,patch.object(clip_editor,'REPO',Path(folder)),patch.object(clip_editor,'OUTPUT',Path(folder)/'manual'):
            for dataset,path in [('realsense','realsense_analysis/cartesian_review.json'),('zed','zed_protocol_analysis/simulation_review.json')]:
                target=Path(folder)/'clips_out'/path;target.parent.mkdir(parents=True)
                target.write_text(json.dumps(dict(runs=[dict(source_id=dataset)])))
            editor=clip_editor.Editor.__new__(clip_editor.Editor)
            self.assertEqual(editor.review()['runs'][0]['source_id'],'realsense')
            self.assertEqual(editor.review('zed')['runs'][0]['source_id'],'zed')

    def test_unknown_review_feedback_is_rejected(self):
        editor=clip_editor.Editor.__new__(clip_editor.Editor);editor.sources={}
        with self.assertRaises(ValueError):editor.review_feedback(dict(source_id='unknown',status='correct'))

    def test_video_seek_ranges(self):
        self.assertEqual(byte_range('bytes=200-399', 1000), (200, 399, True))
        self.assertEqual(byte_range('bytes=900-', 1000), (900, 999, True))
        self.assertEqual(byte_range('bytes=-128', 1000), (872, 999, True))
        self.assertEqual(byte_range('bytes=0-99999', 1000), (0, 999, True))
        self.assertEqual(byte_range(None, 1000), (0, 999, False))

    def test_invalid_video_ranges(self):
        for value in ['bytes=1000-', 'bytes=80-20', 'bytes=-0', 'bytes=-', 'bytes=1-2,4-5', 'potato']:
            with self.subTest(value=value), self.assertRaises(ValueError):
                byte_range(value, 1000)

    def test_valid_trim_and_time_precision(self):
        self.assertEqual(validate_cut({'duration': 74.712}, 7.516, 12.), (7.516, 12.))
        self.assertEqual(validate_cut({'duration': 5.}, '1.23456', '4.98765'), (1.235, 4.988))

    def test_bad_cuts_do_not_reach_ffmpeg(self):
        for start, end in [(3, 1), (-1, 3), (0, 6), (1, 1.01), (True, 4), (1, math.inf), (math.nan, 3)]:
            with self.subTest(start=start, end=end), self.assertRaises(ValueError):
                validate_cut({'duration': 5.}, start, end)


if __name__ == '__main__':
    unittest.main()
