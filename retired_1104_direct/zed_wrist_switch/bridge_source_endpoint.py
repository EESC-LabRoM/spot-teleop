#!/usr/bin/env python3
"""Bridge-side POST /source handler logic (pure, stdlib only, tested offline).

Deploy = small diff inside lab joystick_bridge.py hardware mode:
  - route POST /source to handle_source_post() below;
  - zed activation: call ROS service /teleop/zed_start (std_srvs Trigger).
    Verified (live-stack lane, from fr3_input_teleop.py source): start_zed()
    sets input_source=zed AND sends UDP {"action":"calibrate_arm"} to
    127.0.0.1:zed_calibration_port (47815). A bare param write skips both,
    so the handler must never activate zed via param alone.
  - joystick activation: param set input_source=joystick on /fr3_joy_teleop
    (accepted at runtime; node auto-disengages + resets zed_anchor).
  - include 'source' in the /status JSON so the switch echo is verifiable.

Related teleop services (not used here, operator path unchanged):
  /teleop/zed_engage, /teleop/stop|engage|disengage|home.
"""

from source_switch import SOURCES, decide_switch


def handle_source_post(raw_body, engaged, current_source,
                       activate_zed, set_joystick):
    """Validate + apply a source-switch request.

    activate_zed() -> (ok, message): calls /teleop/zed_start (Trigger).
    set_joystick() -> (ok, message): param set input_source=joystick.
    Returns (http_code, response_dict).
    """
    try:
        import json
        body = json.loads(raw_body) if isinstance(raw_body, (str, bytes)) else raw_body
        wanted = body.get('source')
    except (ValueError, AttributeError, TypeError):
        return 400, {'ok': False, 'message': 'malformed JSON body'}
    ok, msg = decide_switch(wanted, engaged, current_source)
    if not ok:
        code = 400 if msg.startswith('unknown source') else 409
        return code, {'ok': False, 'message': msg, 'source': current_source}
    if msg.startswith('already on'):
        return 200, {'ok': True, 'message': msg, 'source': current_source}
    try:
        applied, detail = activate_zed() if wanted == 'zed' else set_joystick()
    except Exception as exc:  # fail closed: report, do not claim the switch
        return 502, {'ok': False, 'message': 'activation failed: %s' % exc,
                     'source': current_source}
    if not applied:
        return 502, {'ok': False, 'message': 'activation refused: %s' % detail,
                     'source': current_source}
    if wanted == 'zed':
        detail = ('%s; re-engage only after calibrated tracking is active, '
                  'bridge holds (ZED paused) until then' % detail)
    return 200, {'ok': True, 'message': 'source -> %s: %s' % (wanted, detail),
                 'source': wanted}
