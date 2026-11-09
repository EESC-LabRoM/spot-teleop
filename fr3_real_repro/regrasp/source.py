"""Poll the fused live cloud from the viewer (Phase A source).

No ROS, no viewer changes: plain HTTP GET on :8791/grasp.json, same payload
the 11-06 recipe snapshotted by hand. Only mask-paired ticks are usable;
everything else is rejected here so downstream never sees box-crop data.
"""
import json
import time
import urllib.request

from contracts import (SOURCE_MAX_AGE_S, SOURCE_MIN_PTS, VIEWER_GRASP_URL,
                       Tick)


def fetch_grasp(url=VIEWER_GRASP_URL, timeout=5):
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return json.load(r)


def tick_from_grasp(d, wall=None):
    """Parse a /grasp.json payload into a Tick. Raises ValueError when the
    payload lacks a usable cloud (never return a half tick)."""
    cloud = d.get('cloud')
    if not isinstance(cloud, list) or not cloud:
        raise ValueError('grasp payload has no cloud')
    live = d.get('live') or {}
    return Tick(wall=time.time() if wall is None else wall,
                pts=cloud,
                n_zed2=int(live.get('n_zed2', 0)),
                zed2_src=str(live.get('zed2_T') and live.get('cloud', '')),
                cloud_age_s=float(live.get('cloud_age_s', 999.0)),
                src_run=str(d.get('run', '')))


def is_paired(tick):
    """Mask-paired tick: ZED2 contributed via SAM2 (never the box fallback),
    the cloud is big enough and fresh. Relative health (70 % of last good)
    is the loop's job; this is the absolute floor."""
    return (tick.zed2_src == 'live_mask_depth+zed2_mask'
            and tick.n_zed2 > 0
            and len(tick.pts) >= SOURCE_MIN_PTS
            and tick.cloud_age_s <= SOURCE_MAX_AGE_S)


def poll_tick(url=VIEWER_GRASP_URL, timeout=5):
    """Fetch + parse one tick. Returns (Tick|None, reason). Never raises."""
    try:
        d = fetch_grasp(url, timeout)
    except Exception as e:
        return None, f'fetch: {type(e).__name__}'
    try:
        t = tick_from_grasp(d)
    except ValueError as e:
        return None, f'parse: {e}'
    if not is_paired(t):
        return None, (f'unpaired: src={t.zed2_src} n_zed2={t.n_zed2} '
                      f'n={len(t.pts)} age={t.cloud_age_s}')
    return t, 'ok'
