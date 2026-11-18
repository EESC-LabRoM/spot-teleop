# Recorded-data validation, 2026-11-18

Selected: constrained `rgbd_body` with the explicit take04 operator profile. **No real robot, live camera, TF or controller changes were performed.**

## Results

| Metric | Fixed-camera baseline | Selected operator reference |
|---|---:|---:|
| Processed frames, all8 takes | 6385 | 6388 |
| Position-valid frames | 6142 | 6031 |
| Orientation-valid frames | 6085 | 5977 |
| Processing median / p95 | 13.85 / 15.40ms | 14.24 / 35.17ms |
| Capture-to-diagnostic age median / p95 | 70.9 / 212.5ms | 71.8 / 250.5ms |
| Hand-equivalent steps >5cm, continuous frames | 1 | 1 |
| Up/down take06: forward excursion | 15.50cm | 15.70cm |
| Up/down take06: vertical excursion | 75.77cm | 75.02cm |

Estimated arm length **0.500056m**, corresponding gain **1.709810** for0.855m configured robot reach. Arm lengths were fitted from raw RGB-D/Pose in take04 only. The saved profile contains no camera axes or position. Fresh body reference readiness: **1.033s**. Scale variation across the session:0.0; body-axis variation:0.00000121degrees (numerical precision).

Seven takes are held out from arm-length fitting (01,05,06,07,08,09,12). On 5536 matching processed held-out frames after the first2s, baseline has5329 valid positions; the candidate retains5208 of them (**97.73%**) and additionally tracks some baseline-invalid frames. All acceptance checks in `validation.json` pass.

Hand-equivalent ranges/steps divide out each tracker's gain so amplification is not mistaken for jitter. These are observed motion excursions, **not** ground-truth position or orientation errors. Shoulder-roll correction changes axes slightly. Recorded delivery latency is about50.5ms median before inference; body inference raises processing p95 by about20ms. This does not establish physical robot response latency.

## Problems found and resolved

1. Existing `rgbd_fixed` disables operator calibration and uses constant1.315 gain. The candidate uses measured segment-sum scale and an explicit body reference without reverting the newer wrist-depth/palm estimator.
2. Cold calibration took37.57s when the first take lacked enough reliable arm landmarks. The selected explicit profile is learned from take04, requires a fresh current shoulder, and initializes new axes/origin in1.03s. Other people still need their own measurements; a new-person reset clears the old saved profile.
3. The first saved-profile trial incorrectly inferred body pitch from torso depth (forward row had a vertical component~-0.222). Up/down forward excursion grew from15.5cm to28.8cm. Rejected. Final axes keep camera-facing forward and calibrate image-plane shoulder roll; torso depth does not supply gravity. Up/down forward excursion returns to15.7cm.
4. One occluded shoulder could shift the origin. Final translation follows a consensus of common shoulder/hip displacements; insufficient or conflicting landmarks hold the reference. Unit tests cover single-shoulder contamination and disagreeing shoulder pairs.
5. Opposing upper-arm/forearm depth errors could hide behind a constant total. Both segments now need a dominant consensus. Repeated/nonfinite timestamps and corrupt saved profiles are rejected.
6. Start must preserve calibration while `reset_person` must discard it, including the pending saved profile. Both paths are tested. The consumer uses divider1.0 so operator normalization is applied once.

## Remaining limits

- Fewer valid frames are largely warmup and existing operator-identity gates. In take05,80frames after a long loss are deliberately refused until current arm evidence confirms the operator. A high-confidence Right-hand label alone does not bypass this gate; a regression test preserves it.
- The reference assumes an upright fixed operator camera and does not estimate arbitrary camera/body pitch or yaw from noisy landmark depths. It retains the original forward/up axis convention with shoulder-roll correction.
- Camera-space wrist orientation remains model-based; no independent orientation ground truth was recorded. Joint winding limits remain the separate bounded-yaw consumer policy from `wrist_reach_fix_1117`.
- No gripper/contact/loaded-lift or autonomous-grasp validation is claimed. Robot-view camera extrinsics are a separate calibration problem. Live ZED3/RealSense cable faults are handled by the camera lane.
- Historical rejected/cold trials remain in `comparison.json` for audit. Their logs and all dense traces stay under the checks directory. The selected source exactly matches `safe_run_manifest.json`.

Pure verification:10 arm-calibration tests,14 reference tests, exact-consumer gain test and post-loss identity guard test. Replaycontainers used network none,CPU only, no physical outputs. Models and bags are not committed.
