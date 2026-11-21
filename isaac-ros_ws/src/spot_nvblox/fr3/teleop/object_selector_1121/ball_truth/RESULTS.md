# Tennis-ball ground truth by gripper centring (2026-11-21 08:25–08:35Z, supervised)

The operator placed the ball between the open fingers at spot A. The robot centred it with two
15 N squeezes at 90° (widths 60.0 / 60.4 mm), so the ball centre equals the TCP XY at
(0.420, −0.270), with z = table 0.029 + 0.0335. The robot then picked it at A (60.3 mm held
through the lift), carried it and placed it at B (0.380, −0.150). This was the first real ball
grasp. Motions: `plan_truth.py` → `centre_A.json`, `carry_AB.json`; direct-JTC executor.

Per-camera error = image-ray through the ball's colour-blob centroid ∩ the plane z = ball centre,
compared with the truth (live TF):

| spot | ZED2 (mm) | ZED1 (mm) |
|---|---|---|
| A | (+0.6, −3.4) | (−14.2, +1.1) |
| B | (−3.5, −4.0) | (−11.3, 0.0) |

- ZED2 ray-on-plane is good to ~4–5 mm raw, and ~2 mm after the mean offset (−1.5, −3.7) mm.
  ZED1 has an ~11–14 mm x bias in this area, and its stereo-depth ball centre is worse (~20 mm).
- Candidate ball estimator: ZED2 mask centroid ray ∩ (table + r), with a bias correction. It needs a
  third held-out spot (C, `carry_BC.json`) before Y is enabled. It must stay within the 6.5 mm/side
  gripper slack.
