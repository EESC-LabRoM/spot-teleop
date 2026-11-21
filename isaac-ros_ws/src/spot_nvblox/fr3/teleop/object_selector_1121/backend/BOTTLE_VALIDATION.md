# Sprite bottle (transparent PET): offline validation, 2026-11-21

Offline only. No ROS, live stack, map, binding or robot was touched. `planner_validated`
stays False for `sprite`.

**Verdict:**
- **Grasp: NOT supported** by the available evidence. No grasp candidate was implemented.
- **Collision model: a conservative obstacle keep-out IS supported** for the case where the bottle
  is *not* the target. It is implemented offline as a candidate and is not wired into anything.

Code: `bottle_keepout.py`; tests: `test_bottle_keepout.py` (12). Full suite:
`cd backend && python3 -m unittest test_bottle_keepout test_ball_grasp test_selector_backend test_selector_http` → 74 OK (~22 s).
Data: `../twocam/` (ZED1+ZED2 captured 63.7 ms apart, SAM2 masks, from the wrist-tracker lane).
Older single-camera snapshots (`../sprite_pair_mask.png`, `../zed2/sprite_mask.png`) give the same
mask sizes (1845/1840 px ZED1, 3372/3370 px ZED2).

## 1. What the data shows

| | ZED1 | ZED2 |
|---|---|---|
| Mask | 1840 px, 1 component, bbox 38×78 px | 3370 px, 1 component, 56×101 px |
| Visual check (overlays) | whole upright bottle incl. cap | whole bottle incl. cap |
| Valid depth inside mask | 90 % | 95 % |
| In-mask depth q10/50/90 | 0.669 / 0.731 / 0.879 m | 0.781 / 0.862 / 0.957 m |
| Ring around mask q50 | 0.855 m | 1.003 m |
| Depth cloud verdict (`assess_sprite_cloud`) | fragmented, 71 mm z gap, 243 mm footprint | 257 mm footprint |

Depth "exists" (90–95 % valid), but it spreads 18–21 cm in range across an object only a few cm
deep. That is refraction/see-through smear, not a surface. **RGB silhouettes are clean and
repeatable; depth is not usable as bottle geometry.**

## 2. Control experiment on the co-captured tennis ball (known geometry)
- Each camera's own ball cloud projects 100 % into its own mask (projection convention verified).
  Projected into the *other* camera, only 66–71 % lands in the mask, offset ~13–14 px. This is the
  same 16.6 mm ZED1↔ZED2 inconsistency seen in 3D (cause not attributed; see BALL_SPRITE_VALIDATION.md).
- **Diameter from silhouette width is too coarse.** The 65.4–68.6 mm ball reads 83.5 mm (ZED1, p90 row
  width) and 70.8 mm (ZED2), or 73/60 mm using median widths. That is ±15 mm, at 2.6 mm/px for ZED1
  (fx 262 at 0.68 m).
- **A raw two-view visual hull is NOT conservative.** At 0 px dilation the ball hull contains only
  59 % / 55 % of the ZED1 / ZED2 sphere fits. Containment by dilation (px → ZED1, ZED2 fraction):
  0→.59/.55, 3→.76/.75, 6→.91/.92, 7→.98/.99, **8→1.0/1.0**. An earlier exploratory sampling needed
  9 px, so the applied dilation is **8 + 1 = 9 px**.

## 3. Bottle geometry, depth-free
- Hull axis (0 px) at **x 0.734, y −0.236** (fr3_link0).
- Top height from the top-most silhouette pixel ray: **ZED1 0.253 m, ZED2 0.276 m (23 mm apart)**.
  Moving the axis by the 16.6 mm lateral inconsistency shifts each by 10–15 mm. The bottle is about
  22–25 cm tall above the table (≈ 0.0287), consistent with the overlays.
- Diameter: width estimates of 71–91 mm come from a method shown to be ±15 mm off on the ball. Not usable.

## 4. Implemented offline candidate: obstacle keep-out (`bottle_keepout()`)
1. Calibrate the mask dilation on the ball: the smallest k at which the two-view hull contains both
   cameras' sphere fits (k = 8), then add 1 px for the observed sampling sensitivity (k = 9).
2. Carve the bottle hull from both silhouettes dilated by k (3 mm voxels).
3. Require the hull to re-project over ≥ 99 % of each silhouette (got 1.0 / 1.0).
4. Emit an axis-aligned cuboid: **centre (0.7285, −0.2345, 0.1577), dims (0.120, 0.204, 0.261) m**,
   base at the table, frame fr3_link0.

Tests check:
- the keep-out contains the undilated hull;
- it covers both masks;
- it is bounded (< 25 cm footprint, < 30 cm tall) and sits on the table;
- it is ≥ 25 cm from the ball;
- it does not chase the 243 mm depth smear;
- one camera or an empty mask raises an error.

**Why this is evidence-supported, and its limits:**
- Conservative *relative to the measured ZED1↔ZED2 inconsistency at the ball*. A common-mode error
  (both cameras off the same way, e.g. a shared TF-chain error) cannot be seen with two cameras and
  is not covered.
- Calibrated at the ball (~0.33 m from the bottle). A rotational extrinsic error grows with
  distance, so validity at the bottle's position is extrapolated, not measured.
- Single static snapshot. Masks are manual-box SAM2 snapshots, not the live tracker.
- The top of a two-view hull overshoots (cone intersection). That is acceptable for an obstacle and
  wrong for a grasp.
- The cuboid is loose along y (204 mm) because two views carve a diamond. A third view or a
  bottle-shape prior would tighten it.

**Not done, by scope:** feeding it to nvblox or cuRobo. The map still holds whatever the smear
produced near the bottle, and nvblox treats unknown space as free. Integration means adding this
cuboid as a static obstacle while the bottle is not the target. That is the map/coordination lane's
call, and it needs a replay check that it doesn't block valid drill paths.

## 5. Why no grasp candidate (`assess_bottle_grasp` → refused, 3 reasons on real data)
1. **Body grasp:** the diameter is unknown to ±15 mm. Even an optimistic 60 mm body leaves
   (79.9 − 60)/2 ≈ 10 mm clearance per side, below the 16.6 mm measured lateral inconsistency
   (same blocker as the ball). A 90 mm upper bound may not fit the 79.9 mm opening at all.
2. **Cap/neck grasp** (a ~30 mm cap would leave ~25 mm lateral clearance): the cap height disagrees
   by 23 mm between cameras, more than the ~20 mm neck window. The pads could land on the shoulder
   or on air.
3. **Planner contract:** `pts_between_pads >= 50` cannot be honestly met from depth on clear plastic.
   Any bottle path would need a silhouette or model-based substitute, and that needs its own validation.

Even with ideal geometry, `assess_bottle_grasp` refuses until a non-depth substitute for points
between the pads exists (tested).

## 6. What would unblock a bottle grasp (none of it is available offline)
- **Physical measurement** of the bottle: body diameter, cap diameter, neck length, height, mass.
  One ruler measurement removes the ±15 mm diameter uncertainty.
- Resolve or bound the 16.6 mm two-camera inconsistency against an independent static reference
  (tag or known object at a surveyed pose), plus the 7–12 mm below-table offset.
- A model-based pose: fit a known bottle model (from the measurements) to both silhouettes, giving
  axis xy, base and cap z with stated residuals, instead of stereo depth.
- A bottle-specific grasp generator and checks. For a cap grasp: pads on the cap or below it, not on
  the shoulder, with vertical uncertainty ≤ ~5 mm. Also a held-object model from the bottle model,
  then the normal stage gate, broker preflight and operator-supervised trial.
- For the map: a keep-out like §4 while the bottle is an obstacle, and explicit handling of the
  smear voxels when it becomes the target. Carving smear would leave phantom geometry or holes.

## 7. Exploratory cap-color triangulation (not a grasp pose)

A separate offline check used the near-synchronous `twocam/zed1.npz` and
`zed2.npz` frames (63.7 ms apart) with each camera transform stored by the
capture script. Within the bottle masks, a hand-selected HSV threshold
(H=35–90, S>50, V>35) isolated the top green component as the cap. The
component centroids were (403.3, 231.9) in ZED1 (14×13 px) and (168.5, 128.8)
in ZED2 (24×22 px). Treating these image centroids as the same 3D point gives
a least-squares ray intersection at (0.7318, -0.2423, 0.2434) m in
`fr3_link0`, with reprojection errors 2.26 px and 3.52 px. That residual is
large for such a small target, and centroids from two views need not correspond
to the same physical point on a beveled cap.

The projected optical depths at this intersection are 0.663 m and 0.790 m;
the component depth medians are about 0.643 m and 0.764 m. These are
20–26 mm differences, but the image centroids do not identify a common
surface point, so this comparison is only diagnostic. Apparent cap widths
(about 36 and 40 mm) are not physical measurements. This does not resolve the
23 mm full-silhouette top-height disagreement, does not supply a pose error
bound, and does not change `planner_validated=False` or the Y lock. A cap
grasp still needs measured dimensions, a validated pose source and full-path/
payload clearance checks.

A separate offline replay exercised the new `sprite_cap()` estimator on the
stored real masked clouds (`twocam/zed1_sprite_cloud.npy`,
`zed2_sprite_cloud.npy`; not synthetic tests). With table_z=0.029 m it
returned ZED1 cap (0.72138, -0.24082, 0.26009) m from 1,662 points; ZED2 cap
(0.73753, -0.23497, 0.27209) m from 3,212 points; merged cap
(0.72881, -0.23806, 0.26987) m. All three returned an empty problems list,
yet the per-camera estimates differ by 17.2 mm XY and 12.0 mm Z. The live
Sprite plan reported cap_top=(0.7212, -0.2399, 0.2504) m, close to ZED1 and
about 10 mm below that camera's replayed estimate. The current 20 mm circle-fit
RMS gate accepts these inconsistent poses. Keep Sprite Y locked; synthetic
unit tests and a swept map check do not resolve this pose disagreement.

## 8. Real-cloud replay of the live cap estimator (2026-11-21 07:42Z)

I independently replayed the live lane's `sprite_cap()` estimator against its
stored masked point clouds, rather than its synthetic test fixture:

- ZED1 (`twocam/zed1_sprite_cloud.npy`, 1,662 pts): cap estimate
  (0.72138, -0.24082, 0.26009) m; reported spread 12.1 mm.
- ZED2 (`twocam/zed2_sprite_cloud.npy`, 3,212 pts):
  (0.73753, -0.23497, 0.27209) m; spread 6.3 mm.
- Merged cloud: (0.72881, -0.23806, 0.26987) m; spread 9.0 mm.

The estimator returns no problems for all three inputs, but the per-camera
centres differ by 17.2 mm in XY and 12.0 mm in Z. The live plan's
`cap_top=(0.7212,-0.2399,0.2504)` m follows the ZED1 XY estimate and is about
10 mm below its Z estimate. This is incompatible with the documented ≤5 mm
cap-pose objective; the existing 20 mm circle-fit RMS gate does not catch the
cross-camera disagreement. Keep Sprite `planner_validated=false` and Y locked
until the pose source is reconciled against ground truth and the complete
payload path is validated. This replay changed no live state and issued no
robot command.

## 9. Live supervised side-grasp cycle (2026-11-21 08:03Z)

The selector lane reports a successful complete pick/place cycle on run
`20261121_075900_sprite_selector`, using a horizontal/pitched label-band grasp
after the operator moved the bottle closer (reported axis near
`(0.483,-0.074)`). Approach, descent, close/width verification, lift, held-model
load and return/place all reportedly passed. The lane reports the robot at
Home, gripper open, idle, with no regrasp lock. A read-only selector response
at 08:02:53Z reports Sprite Y enabled as a supervised trial and fresh mask/map
with 100% carve; it reports camera error ~10mm against 7.5mm per-side slack.

This demonstrates one supervised success at that moved bottle pose. It does
not remove the prior cross-camera inconsistency or prove robust performance at
other poses. The lane reports fixed 60N compressed the PET bottle to 24mm;
force and bottle deformation therefore need evaluation before treating the
setting as generally suitable. Keep the feature labelled as supervised trial;
the ball remains Y-locked pending calibration and a validated grasp path.
