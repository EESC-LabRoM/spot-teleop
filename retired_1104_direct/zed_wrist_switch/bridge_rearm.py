#!/usr/bin/env python3
"""Auto-rearm proposal for the cuRobo hardware bridge (offline, no ROS).

Problem (observed live 2026-11-04): after a joystick -> ZED -> joystick
round-trip, the teleop source lands back on `joystick` but the bridge sits
`armed=false` ("cuRobo output disarmed"), so pad Start does nothing until
someone runs a lab-shell enable call. The operator expectation is: press
Start, it just drives.

Proposal: re-arm automatically on a FRESH Start press when everything is
green, and only then. Fault disarms still require the manual enable call.

Rules encoded in maybe_auto_rearm() (pure, tested below):
  - Already armed -> unchanged.
  - Disarmed by a FAULT (joint-controller ownership lost) -> stay disarmed.
    A fresh Start after a fault must NOT silently resume motion; the manual
    enable call stays the deliberate recovery step.
  - Disarmed benignly (enable-off from the switch chain, standby) ->
    re-arm exactly on the rising edge of engagement (was idle, Start now
    held) AND controller ownership ready AND a fresh MPC packet passing
    validation. A level-held Start (stale hold predating the check) never
    arms: same guarantee as the current "disengage before enabling" rule.
  - Arming alone never moves the robot: motion still needs all existing
    output gates (ownership, fresh engaged hold, source, ZED-active, full
    packet validation incl. first-step 0.015 rad).

Deploy sketch (lab joystick_bridge.py hardware mode, needs Murilo's go +
live-stack handoff from lane e64db8e7):
  - track prev_engaged_fresh + fault_disarm (set on ownership-loss disarm,
    cleared on manual enable-off/on);
  - in output(), when not armed, call maybe_auto_rearm() with the live
    gates; on True, set armed and continue through the unchanged gate chain.
  - surface fault-vs-benign in /status reason so the dashboard can say
    "press Start" vs "needs manual enable".

Open (cannot see from here): WHO sent the enable-off during the ZED
round-trip (dashboard stop action? switch chain? a lane script?). If that
caller is found and removed, the benign disarm may disappear entirely and
this becomes a pure safety net. Lab grep: callers of /fr3_curobo/enable
with data=false, and the zed-control disengage/stop path in mesh_server.
"""

from collections import namedtuple

RearmDecision = namedtuple('RearmDecision', ['armed', 'note'])


def poll_decision(pending, pending_done, issued_at, now, service_ready,
                  timeout_s=2.0):
    """Service-poll retry gate (pure). A stuck in-flight call must not wedge
    controller discovery forever: abandon it after timeout_s so the next
    tick re-issues. Returns 'wait' | 'call' | 'reset'."""
    if pending and not pending_done:
        return 'wait' if now - issued_at < timeout_s else 'reset'
    return 'call' if service_ready else 'wait'


def maybe_auto_rearm(*, armed, fault_disarm, engaged_now, engaged_prev,
                     controller_ready, packet_valid):
    """Pure auto-rearm gate. Returns RearmDecision(armed, note)."""
    if armed:
        return RearmDecision(True, 'already armed')
    if fault_disarm:
        return RearmDecision(False, 'fault disarm: manual enable required')
    rising_edge = engaged_now and not engaged_prev
    if not rising_edge:
        return RearmDecision(False, 'disarmed: press Start to resume')
    if not controller_ready:
        return RearmDecision(False, 'disarmed: joint controller not ready')
    if not packet_valid:
        return RearmDecision(False, 'disarmed: planner packet not valid')
    return RearmDecision(True, 'auto-rearmed on fresh Start press')
