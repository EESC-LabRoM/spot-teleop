# FR3 bag-as-camera teleop test

Runs Hipólito's recorded ZED protocol bags
(`/home/hipolito/spot-teleop/bags/zed_protocol_20261030_005244`, 13 takes /
12 movements, 05 repeated) as a real-time camera on the **real robot**,
through the **live movement-recognition pipeline**, and CHECKs each movement
against what the MuJoCo simulation validated
(`clips_out/zed_protocol_analysis/simulation_review.json`).

Chain per take (all test topics/ports; nothing live is restarted or remapped):

```
stage d8 bag ──bag_camera.py──> /fr3_bagtest/{rgb,depth,camera_info} (DDS 8, 1.0x)
        │                              │
        │ recorded samples             │ test operator_hand_tracker
        └─────────────> /fr3_bagtest/recorded_sample    │
                                        ▼
                   test zed_input_relay ──UDP 47817──> test fr3_input_teleop
                     (compare: new vs recorded)         (profile per stage)
                                                              │ cartesian impedance
                                                              ▼ real FR3
```

Visualization while it runs: bag camera page `:8793`, test tracker overlay
`:8790/snapshot`, robot `:8787/snapshot?topic=/zed1/zed_node/left/image_rect_color`.

## Files

- `bagtime.py` — ROS-free helpers: stage/profile map, stamp rebase, window
  math, TCP metrics, `compare_take` CHECKs, re-inference drift. Unit-tested.
- `test_bagtime.py` — stdlib unittest suite (`python3 -m unittest test_bagtime`).
- `bag_camera.py` — bag image publisher + visualization page (runs in
  fr3-curobo, DDS 8). Imports `decode_depth` from the lab `remote_sim`.
- `record_jsonl.py` — topic-to-jsonl recorder (runs in both containers).
- `run_take.py` — one-stage orchestrator: preflight → test chain → play →
  engage/record → stop/home → compare → `summary.json` + CHECK line.
- `run_session.sh` — all 13 takes in order on one persistent session chain
  (tracker + relay + camera; `run_take.py --chain up/down`). The chain stays
  up across takes: per-take publisher restarts wedge the tracker's
  subscriptions under lab CPU load, and stage 09 (6.6 s) never locks a cold
  tracker. Takes only restart their recorders and `load` the next bag into
  the camera (same publishers, no rediscovery).

## Step 0 — put in Hipólito's shared teleop part (lab, needs go)

The lab repo already sits at his `franka-teleop` tip (`bbea50b`); the running
copy is older in two files (`zed_input_relay.py` drops v3 samples,
`deploy.py` lacks the review states). Install it:

```bash
ssh murilo@100.88.134.127
cd /home/hipolito/spot-teleop/isaac-ros_ws/src/spot_nvblox/fr3/teleop
python3 deploy.py            # dry run: shows the restart plan
python3 deploy.py --apply    # copies runtime + restarts teleop/dashboard (refuses when engaged)
```

## Step 1 — deploy this test (lab)

```bash
scp -r fr3_teleop_bagtest murilo@100.88.134.127:/home/murilo/fr3_bagtest
```

## Step 2 — dry runs: recognition pipeline (no robot motion, safe anytime)

```bash
cd /home/murilo/fr3_bagtest
python3 run_take.py --stage 00_neutro --out takes/dry_00
bash run_session.sh takes/dry_session        # all 12, dry
```

Each dry take must print `RECOGNITION PASS`: new-inference samples flow at a
healthy rate with no gap over 3 s, and the stage's recognition metric matches
the recorded samples: windowed de-meaned wrist drift ≤ 12 cm for translation
stages, relative-rotation drift ≤ 15° for roll/pitch/yaw (their robot motion
comes only from palm orientation), both gestures seen for the gripper stage.
Each take starts with a warm-up pass so the tracker locks before the scored
pass, mirroring a continuous operator session (stage 09 is only 6.6 s and
never locks cold).
Absolute hand coordinates are NOT compared: the p/q reference changed
between the October recording code and the current tracker (~2 m mean
offset on identical images), and teleop only uses motion relative to its
Start anchor. Fix tracker/camera issues here, re-run, then go to step 3.
`rescore.py <take>` re-applies the current metric to a finished take's
saved `samples.jsonl` without re-running the lab chain.

## Step 3 — robot takes (needs go: e-stop person + franka-robot ownership)

Coordinate with the grasp lane first (it drives the same arm). Then:

```bash
cd /home/murilo/fr3_bagtest
python3 run_take.py --stage 04_frente_tras --out takes/04_take1 --execute
```

Preflight refuses unless: no teleop process runs, nobody publishes
`equilibrium_pose`, the cartesian impedance controller is active, live hand
topics are idle, and the arm homes cleanly. Between takes the arm returns
home. Per validated movement: if the CHECK fails, fix, re-run that stage.

Expected motion per stage (fr3_link0, from home; see `STAGE_SPEC`):

| stage | profile | expected |
|---|---|---|
| 00_neutro | neutral | still (<3 cm, <8°) |
| 01_giro_plano_imagem | roll | tool roll >15°, translation <3 cm |
| 02_inclinar_dedos | pitch | orientation >10°, translation <3 cm |
| 03_virar_palma | yaw | orientation >10°, translation <3 cm |
| 04_frente_tras | height | z span >5 cm, x/y <4 cm |
| 05_direita_esquerda[_2] | horizontal | y span >5 cm, x/z <4 cm |
| 06_cima_baixo | reach | x span >5 cm, y/z <4 cm |
| 07_abrir_fechar | gripper | still + gripper width span >5 mm |
| 08_cruz | cross | x and y spans >5 cm, z <4 cm |
| 09_gesto_em_movimento | translation | partial (sim: small); samples must flow |
| 10_perda_retorno | translation | multi-axis translation |
| 11_distancias | translation | multi-axis translation (never engaged live yet) |

## Step 4 — operator handoff

After the last take, leave the stack so a human can press Start:

1. Test chain stopped (run_take cleans its own processes; verify no
   `fr3_bagtest`/`bag_camera.py`/`zed_port:=47817` processes remain).
2. Arm at home, teleop disengaged.
3. Start the **operator** teleop (live UDP 47813) disengaged; dashboards
   `:8088`/`:8089` green; live tracker + relay running (untouched).
4. **ZED3 must stream**: the wrapper runs but the camera currently yields no
   images (physical replug needed at the lab). Verify
   `ros2 topic hz /zed3/zed_node/rgb/image_rect_color` and `:8786/snapshot`
   before handing over — without it the operator's hand is invisible.

## Verify

- Unit: `python3 -m unittest test_bagtime` (19 tests, no ROS needed).
- Lab dry: step 2 prints `RECOGNITION PASS` per stage; `:8793` shows the bag
  frames; `:8790/snapshot` shows the tracker overlay.
- Lab robot: `summary.json` per take with `check.ok=true`; CHECK line on
  stdout; `takes/<stage>/robot.jsonl` + `samples.jsonl` kept for review.
