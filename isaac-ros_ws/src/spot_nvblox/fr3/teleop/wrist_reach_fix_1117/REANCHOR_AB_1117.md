# Re-anchor A/B — 2026-11-17 (Claude lane 1ca94120)

Offline only. The `wrist-posture-sim-1117` container (`--network none`, GPU1) replays the 23:30 live take `live2330` through the staged consumer with native MPC and MuJoCo. No robot.

## Changes made
1. **Kernel switch.** `candidate/consumer_kernel.py` now reads `zed_preserve_workspace_anchor` from the `PRESERVE_ANCHOR` env var, defaulting to `1` (old behaviour). Backup: `candidate/consumer_kernel.py.pre_reanchor`.
2. **New results:** `results/reanchor_{g2,g15_v15,g2_v15}_live2330/`, plus `replay_reanchor_*.log`.
3. Nothing else was edited.

## Results

| run | gain | vertical | clamp % | rejections | TCP err p95 | orientation p95 | tilt p95 | j7 span | TCP z min |
|---|---|---|---|---|---|---|---|---|---|
| live_old (live profile) | 1 | 1 | 15 | 0 | .066 | 29.6° | 7.2° | 246° | **.249** |
| live_gain2 (preserve anchor) | 2 | 1 | **64** | **321** | .110 | 3.8° | 2.9° | 136° | .091 |
| reanchor_g2 | 2 | 1 | 8 | 15 | .076 | 6.1° | 3.3° | 134° | .170 |
| reanchor_g15_v15 | 1.5 | 1.5 | 6 | 96 | .079 | 5.8° | 3.1° | 135° | .239 |
| **reanchor_g2_v15** | 2 | 1.5 | 11 | 38 | .078 | 5.5° | 3.2° | 134° | **.118** |

## Reading
- **Old profile.** The TCP never went below z .249, which explains "can't lower it". The 30° rotation error and 246° joint-7 span are the twisting.
- **Gain 2 with a preserved anchor.** It reaches the floor, but the target sits clamped 64% of the time, giving 321 "arm not following" holds and 11 cm TCP lag.
- **Recommended:** gain 2, vertical gain 1.5, `zed_preserve_workspace_anchor:=false` (re-anchor on clamp). Keep the candidate's yaw clamp (±60° at 15°/s), v_z .20 and floor .10.
- **Effect of the recommendation:**
  - it reaches z .118, about the drill grasp TCP z .1206;
  - clamping falls to 11% and holds fall to 38;
  - rotation stays fixed.
- **Caveat:** this is open-loop replay of a hand that was reacting to the old robot. Live, closed-loop behaviour will differ, and contact height is still not validated.
