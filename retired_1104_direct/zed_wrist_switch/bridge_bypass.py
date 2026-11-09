#!/usr/bin/env python3
"""Bypass flag for the cuRobo hardware bridge (offline, no ROS).

Question under test (Murilo, 2026-11-04): is the bridge an unnecessary
step? The direct path exists (original cuRobo MPC node publishing
straight to the Franka joint controller), so the bridge may only be
left over from an old topic-conversion job.

This file does NOT disable anything live. It defines the pure bypass
semantics + the exact live patch sketch, so the A/B test can be run
deliberately and reversibly:

  bypass=false (default) -> bridge behaves exactly as today.
  bypass=true            -> bridge goes monitor-only: it NEVER publishes
    to the joint controller, but keeps polling controllers/packets and
    reports what it WOULD have sent. The direct MPC path drives.

Rules (fail-closed, same shape as the source-switch gate):
  - Flipping the flag requires disengaged (release Start first). A flip
    request while engaged is refused.
  - Bypass never moves the robot by itself: publish is forced closed,
    engagement/auto-rearm output gates are skipped, monitoring stays on.
  - Clearing bypass (back to bridged) also requires disengaged, then a
    fresh Start press re-arms through the normal gate chain.
  - Fault monitoring stays on in both modes: ownership loss is still
    reported, it just never auto-recovers motion in bypass mode either.

Live patch sketch (lab joystick_bridge.py, needs Murilo's go + e64
handoff; bridge container restart only, MPC/teleop untouched).
Confirmed live topology (e64 lane, 2026-11-04, unverified by me):
  - MPC joint preview: /fr3_joystick_mpc/joint_preview
    (remapped from /arm/joint_command);
  - MPC -> bridge UDP: port 47927 (safe_packet_port); goals in: 47928.
  - the bridge's publish gate is the packet forwarder to the JTC, so the
    flag sits exactly there:
  - read CUROBO_BRIDGE_BYPASS at startup ("1"/"true" -> bypass);
  - add "bypass": bool to :8784/status + reason "bypass: monitor-only";
  - in output()/forwarder: if bypass: update monitors, never publish to
    the JTC, return the would-send note; else run the unchanged gate
    chain (incl. the auto-rearm deployed 2026-11-04).
"""

BYPASS_REASON = 'bypass: monitor-only, direct MPC path drives'


def parse_bypass(value):
    """Parse an env/param string into bool. Unknown -> False (fail-closed)."""
    if isinstance(value, bool):
        return value
    if value is None:
        return False
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def decide_bypass_change(requested, current, engaged):
    """Gate a bypass flip. Returns (ok, message)."""
    if not isinstance(requested, bool):
        return False, 'bypass flag must be bool, got %r' % (requested,)
    if engaged:
        return False, 'disengage first (release Start) before flipping bypass'
    if requested == current:
        return True, 'already bypass=%s (no-op)' % current
    if requested:
        return True, 'ready: bridge will go monitor-only, direct path drives'
    return True, 'ready: bridge will resume publishing (fresh Start re-arms)'


def bridge_publish_allowed(*, bypass, armed, engaged_now, controller_ready,
                           packet_valid, source_ok):
    """Pure publish gate. Returns (allowed, note)."""
    if bypass:
        return False, BYPASS_REASON
    if not armed:
        return False, 'disarmed: press Start to resume'
    if not engaged_now:
        return False, 'idle: waiting for joystick Start'
    if not controller_ready:
        return False, 'joint controller not ready'
    if not packet_valid:
        return False, 'planner packet not valid'
    if not source_ok:
        return False, 'source hold: ZED paused'
    return True, 'bridge publishing'
