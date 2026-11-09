"""live_3d.ok must be false unless the point AND the mask are fresh AND tracking.

Regression: drill.json used to report ok:true whenever any /tracking_3d_point
had ever arrived, with age_s measuring overlay-loop liveness (the got-dict was
never drained), so a LOST tracker with an 11 s-stale mask still read ok:true.
Run (ROS-sourced): docker exec vm-viewer bash -c
  'source /opt/ros/humble/setup.bash && python3 <this file>'
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from seg_view import _live3d_ok


def check(cond, msg):
    if not cond:
        raise AssertionError(msg)
    print(f"ok: {msg}")


check(_live3d_ok(0.0, 0.2, True) is True, "fresh point + fresh mask + tracking -> ok")
check(_live3d_ok(0.0, 11.3, True) is False, "stale mask kills ok (the 11.3s incident)")
check(_live3d_ok(0.0, 0.2, False) is False, "LOST tracker kills ok despite fresh point")
check(_live3d_ok(5.0, 0.2, True) is False, "stale point kills ok")
check(_live3d_ok(None, 0.2, True) is False, "no point yet kills ok")
check(_live3d_ok(3.0, 3.0, True) is True, "boundary 3.0s still ok")
check(_live3d_ok(3.1, 0.2, True) is False, "just past boundary kills ok")
print("6/6 live_3d gate checks passed")
