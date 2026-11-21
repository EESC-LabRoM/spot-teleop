# Tennis ball / Sprite bottle: offline grasp validation (2026-11-21)

Offline only. No binding, map, tracker, TF, controller or robot was touched.
`planner_validated` stays False for both; nothing here enables a pick.

Files: `ball_grasp.py`, `test_ball_grasp.py` (25 tests); selector integration in
`selector_backend.py` (stage gate + `ball_validation` requirement).
Run: `cd backend && python3 -m unittest test_ball_grasp test_selector_backend test_selector_http` (62 OK).

## Evidence used (from the wrist-tracker lane, verified locally)
- ZED1 cleaned ball cluster `../tennis_ball_cluster.npy`: 813 pts.
  Sphere fit r = 34.9 mm (ITF 32.7–34.3), rms 1.8 mm, inliers 99.5 %, one cluster.
  Fitted bottom is **9.6 mm below** the planner table_z 0.029 m.
- Uncleaned snapshots (`../tennis_ball_snapshot_cloud.npy`, `../zed2/...`) include
  table points: r 42.5 / 66.3 mm and are correctly refused.
- ZED2 after the same table cut (z > 0.04) + 6 cm radius filter: r 35.7 mm, rms 2.9 mm,
  bottom 14.7 mm below table. **ZED1 vs ZED2 fitted centres differ by 16.6 mm in XY**
  (13.3, −9.9 mm) and 4.3 mm in z. The captures were not simultaneous, so object motion
  can't be ruled out, but nothing says it moved.
- Peer replay of the existing drill `selection.select`: NO_FEASIBLE. Depths .03/.01 had stage
  problems, and depth 0 left the tip 1.5 cm below the top (< TIP_MIN 3 cm).
- Real GraspGen pool `../tennis_ball_snapshot_grasps.npy` (200×17, peer copy) replayed with
  `replay_graspgen` (hand-frame convention identical to `canonical/selection.py::flatness`,
  round-trip tested) over depths .03/.01/0/−.01 = 800 pose checks:
  - measured budget (16.5 / 14.7 mm): **0 / 800 pass**;
  - hypothetical 3 / 3 mm budget: **1 / 800** (idx 14, depth .03; ~1 mm side clearance, 7.8 mm off
    pad centre), so it's marginal even then;
  - only 9/200 proposals are within 15° of vertical. Most pinch above the equator, and 170 checks have
    0 points between the pads.
  This is a geometry-only replay: no IK, arm or stage collision checks. The planner still has to
  plan that candidate (the peer reports depth .03 had S2–S5 problems in the drill path).

## Why the ball needs its own rules (stricter, not relaxed)
Parallel pads hold a sphere only if they straddle its equator. If they pinch above it, the ball
squeezes out. With r ≈ 35 mm the TCP must sit ≥ r + vertical budget below the top, i.e. ≥ ~4.5 cm.
**TIP_MIN 3 cm is not enough for a ball** (test: tip 3.2 cm, TCP still above the equator → refused).
The 1.5 cm GraspGen candidate is refused by both rules. Do not lower TIP_MIN.

## Ball requirements encoded (`check_grasp` / `validate_ball_binding`)
1. Sphere fit: r 29.7–37.3 mm, rms ≤ 3 mm, ≥ 90 % inliers within 5 mm, main cluster ≥ 95 %,
   XY span ≥ 1.4 r (not just a cap), fitted bottom within 15 mm of the table.
2. Error budget is **derived, never assumed**: it needs ≥ 2 valid camera fits.
   lateral = max(XY centre disagreement, measured broker drift);
   vertical = max(z disagreement, |bottom − table|). A zero or unmeasured budget raises an error.
3. Strict approach tilt only (vert ≥ cos 15°). There is no 25° fallback rung for the ball.
4. Equator inside the pad band (TCP−45 mm … TCP) with the vertical budget on both sides.
5. Open-finger side clearance: 39.95 mm − r − |dy| − lateral budget ≥ 0.
6. Ball centre within pad half-width (11 mm) after the lateral budget.
7. Fingertips ≥ 5 mm + vertical budget above the table.
8. ≥ 50 cloud points between the pads (same bar as the planner contract); TIP_MIN kept as an extra floor.
9. Post-close `held_width_ok`: width must be in [2r − 15 mm, 2r + 4 mm]. The 0.216 mm drill-slip
   width and the 79.9 mm still-open width both fail, giving the slip detection that is missing today.
10. Held model: one sphere, r + 12 mm, at the fitted centre in fr3_hand.
11. The selector refuses a ball binding unless `plan['ball_validation']` is
    `{ok: True, run: <same run>}`, even if `planner_validated` were flipped.

## Result on the real 11-21 evidence: BLOCKED (cause not attributed)
- Gripper capacity: 79.9 mm open vs 69.8 mm ball → a perfectly centred top-down grasp absorbs
  only **5.1 mm** of lateral error.
- Measured lateral inconsistency is **16.6 mm** (ZED1↔ZED2), and 5.6 mm broker drift alone already
  exceeds the capacity. This is a geometric inconsistency, **not a proven calibration fault**. Both
  fits come from partial, cropped surfaces, and the captures were not simultaneous. The offset
  below the table and the diverging centres cannot separate calibration, depth bias, fit bias or
  object motion. The vertical budget (14.7 mm) leaves **no TCP z window** between the
  table-clearance floor and the equator.
- Exhaustive top-down sweep (z 2–9 cm × ±10 mm XY): **0 passing poses**.
- The validator is not always-false: under a hypothetical 3 mm / 3 mm budget a centred grasp passes
  (window z 37–51 mm, 180+ points between the pads).

### Near-simultaneous two-camera capture (`../twocam/`, 63.7 ms spread, peer lane)
Reproduced locally from the saved clusters:
- ZED1 r 34.9 mm, rms 1.9 mm, bottom −9.5 mm vs table .0287. ZED2 r 34.9 mm, rms 2.8 mm, bottom −12.7 mm.
  Centres are 16.6 mm apart in XY and 3.2 mm in z, so the discrepancy persists with (near-)simultaneous frames.
- **Radius fit bias ruled out as the main cause.** Fixing r at the ITF min/nominal/max (32.7/33.5/34.3 mm)
  keeps a 16.3–16.4 mm XY offset, and both free fits agree on r to 0.01 mm.
- **Per-camera depth (range) bias ruled out as the main cause.** The offset lies almost entirely *across*
  each camera's line of sight (ZED1 16.9 mm across / −1.2 mm along at 0.68 m; ZED2 16.6 mm across /
  +3.4 mm along at 1.05 m). The best per-camera range scale (1.0006 / 0.9968) leaves 16.6 mm.
  (`diagnose_two_camera`.)
- **Still open:** extrinsic TF (≈1.4° at ZED1 range or ≈0.9° at ZED2 range would do it), intrinsics
  or principal-point error, a lateral bias from the cameras seeing opposite hemispheres, or mask error.
  Two cameras alone cannot say which one is wrong. That needs an independent static reference, e.g. a
  tag or known-geometry object at a surveyed pose. The 7–12 mm below-table offset is a separate open
  question (table_z estimate vs. depth).
- Replay with this budget (16.6 / 12.7 mm) is still 0/800.

### What would have to change, with evidence, before a ball attempt
- ZED1/ZED2 ball-centre agreement ≤ ~3 mm from simultaneous captures, plus an explanation of the
  10–15 mm table/bottom offset. Disambiguating it takes, for example, a reference sphere of
  known size and position, a fit with a fixed known radius, or an independent depth check against
  the table plane. Only then can calibration, depth or fit be named as the cause.
- Broker XY drift for the ball ≤ ~3 mm.
- A ball-specific GraspGen/ranking path that emits `ball_validation`, with a replay of the real
  proposal array, then the drill-equivalent live broker preflight.
- An answer for fingertip geometry beyond the TCP (assumed 0 mm here; measure it).
- Possibly a different grasp strategy (wider fingertips / compliant pads / scoop). With 5 mm/side,
  a parallel-pad top-down grasp is inherently marginal.

## Sprite (transparent PET): no grasp path, assessment only
`assess_sprite_cloud` always returns ok = False and lists the evidence:
- ZED1: fragmented (main cluster 0.91 of 10), 71 mm vertical depth gap at z = 0.152, footprint 243 mm.
- ZED2: footprint 257 mm (a small bottle is < ~7 cm wide), so the cloud is depth smear.
  Peer measured ZED1→ZED2 p90 160 mm.

Requirements before any Sprite grasp:
- Geometry that does not come from stereo depth on clear plastic: a known bottle model fitted to
  RGB mask silhouettes from both cameras, an opaque sleeve/label band only, or a different sensor.
- The map must not carve or keep phantom voxels from the smear. Today the nvblox geometry near the
  bottle is unreliable both as an obstacle and as a carve target. Treat the area as an unknown
  keep-out, not free space (nvblox treats unknown as free today).
- The same identity/stage/held-width gates as the ball, plus a validated held model.

## Planner limits that apply to every object (also in `known_gaps` in the API)
- `plan_grasp` only raises min_obstacle problems for S1/S6, so `problems == []` is not avoidance
  evidence. The selector now re-checks every S1–S6 stage: obstacle ≥ 0.10 m, table ≥ 0.10 m,
  joint margin ≥ 0.05 rad, values present and finite. These floors were not tuned; the lowest real
  11-21 drill values are 0.118 / 0.166 m.
- Checks are per waypoint (n_waypoints = 1), not swept. The hand, fingers and carried object are
  not in the obstacle check. Y stages run on direct JTC, not MPC. Unknown space counts as free.
