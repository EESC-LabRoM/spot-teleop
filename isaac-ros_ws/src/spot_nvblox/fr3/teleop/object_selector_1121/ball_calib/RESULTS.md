# Wrist-camera hand-eye + ball localisation — 2026-11-21 07:10–07:22Z

Two supervised passes with the user's go (operator on e-stop). Direct-JTC executor, gripper open,
no contact, robot back at Home after each pass (final joints within 0.02 rad of Home).
- Pass 1 (run_071025): 8 poses. Ball found in 6 views. Measured joints only for C3, C4, C7 and C8
  (the reader failed on the others), so most views use commanded q.
- Pass 2 (run_071727): 7 poses aimed with the pass-1 rough hand-eye. Ball found in 5 views, of which
  2 masks are partial. Joints were measured at every pose.

## Result: no rigid hand-eye at mm level
- Near-top-down views (pass 1): rms 5.2 mm, ball (0.414, -0.273, 0.061), table 0.027 (true ~0.029–0.031).
- Large wrist rotations (pass 2): rms 6–15 mm, table −0.03..+0.075 (impossible).
- Ruled out: latency (~40 ms; static arm), depth scale (free sphere radius 30–35 mm), and mounting on
  link5/6 (those fit far worse; link7 and hand fit identically).
- Remaining causes: the camera mount flexes with orientation, or RealSense depth/intrinsics bias at
  oblique views.
  The ball is 6.5 mm per side narrower than the open gripper, so 5 mm absolute error is NOT enough.

## Proposed fix (ground truth, fixed orientation)
1. The operator puts the ball between the open fingers, the gripper closes, and the width check
   confirms the grasp.
2. The robot places the ball on the table at a known FK position and opens. The ball centre is then
   known to ~1 mm.
3. Pure vertical retreat to the canonical top-down LOOK pose, then capture: ball pixel/depth against
   the known position. Repeat at 4–5 XY spots with the same orientation.
4. This gives a local hand-eye valid at the look orientation, validated by held-out placements.
   Target: ≤ 2 mm.
Live grasp then = coarse ZED position → LOOK pose (same orientation) → wrist fix → dense vertical
descent (ball_plan.py, TCP 6 mm below the centre) → close → width check 52–71 mm → lift.
