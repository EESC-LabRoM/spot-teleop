#!/usr/bin/env bash
# Sim rehearsal of the avoidance run from the arm's LIVE joints on the CURRENT map capture, at a given extra buffer; on PASS writes rehearsal_q0.txt / rehearsal_buf.txt (read by avoid_run.sh).
#   [CFG=fr3_vcap1.2_acc10_post.yml MORE="-p mpc_override:=mpc_override_ns200.yml"] NPZ=esdf_X.npz (in calib_1031/avoidance_sim) BUF=0.12 RUN=real_rehearsal_X [BAR=5] ./rehearse.sh
H=$(cd "$(dirname "$0")" && pwd); cd /home/nexus/spot-teleop/isaac-sim_ws/franka_mpc/deploy_sim; source ds_env.sh
NPZ=${NPZ:?set NPZ}; BUF=${BUF:-0.12}; RUN=${RUN:?set RUN}; BAR=${BAR:-5}   # BAR = min real gap (cm) to bottle/drill/box that counts as a pass
AS=/home/nexus/spot-teleop/fr3_real_repro/calib_1031/avoidance_sim
L="ssh -i $HOME/.ssh/id_ed25519_lab_claude -o BatchMode=yes -o ConnectTimeout=8 murilo@100.88.134.127"
Q=$($L "docker exec fr3-ros2 bash -c 'source /opt/ros/jazzy/setup.bash; source /ros2_ws/install/setup.bash; export ROS_DOMAIN_ID=77; timeout 8 ros2 topic echo --once --field position /joint_states'" 2>/dev/null | python3 -c "
import sys,re; print(','.join(re.findall(r'-?\d+\.\d+(?:e-?\d+)?', sys.stdin.read())[:7]))"); echo "Q=$Q"
FK=$("${DSRUN[@]}" --gpus device=1 $IMG /workspace/franka_mpc/deploy_sim/fk_q.py "$Q" 2>&1 | grep -E "^(P0|QUAT)"); echo "$FK"
P0=$(echo "$FK" | sed -n 's/^P0 //p'); QU=$(echo "$FK" | sed -n 's/^QUAT_XYZW //p')
Q0=$Q P0=$P0 QUAT=$QU WPS="0.166,0.311,-0.268,12,2;0.166,-0.289,-0.368,16,3;0.106,0.151,-0.368,14,3;0,0,0,14,3" HOLD=10 TAIL=3 \
MAP=$AS NPZ=$NPZ VOXEL=${VOXEL:-0.05} GOAL=goal_obstacle.py NODE_EXTRA="-p extra_collision_sphere_buffer:=$BUF ${MORE:-}" BRIDGE_ARGS="--max-accel 10" SECS=84 RUN=$RUN \
timeout 420 ./run_deploy_sim_obstacle.sh > /tmp/rehearse_$RUN.log 2>&1; echo "sim exit $?"
grep -E "hold  0-" /tmp/rehearse_$RUN.log | cut -c1-200
CL=$("${DSRUN[@]}" --gpus device=1 -v $AS:/o:ro $IMG /workspace/franka_mpc/deploy_sim/clearance.py /workspace/franka_mpc/runs/$RUN /o/${OBJ:-objects_raw.npz} 0.042 2>&1 | grep -E "min clearance|hand target"); echo "$CL" | cut -c1-200
"${DSRUN[@]}" --gpus device=1 -v $AS:/o:ro -v $H:/v:ro $IMG /v/map_clearance.py /o/$NPZ /workspace/franka_mpc/runs/$RUN/trace.npz /o/${OBJ:-objects_raw.npz} 2>&1 | grep -E "^MAP|floaters|all occupied"
"${DSRUN[@]}" --gpus device=1 -v $H:/v:ro $IMG /v/joint_excursion.py /workspace/franka_mpc/runs/$RUN/trace.npz 2>&1 | grep -E "^POSTURE"
python3 - "$CL" "$BAR" "$Q" "$BUF" "$H" "/tmp/rehearse_$RUN.log" <<'PY'
import sys, re, os
cl, bar, q, buf, h, log = sys.argv[1:7]; bar = float(bar)
mins = {k: float(v) for k, v in re.findall(r"(bottle|drill|box)\s+min clearance\s+(-?[\d.]+) cm", cl)}
cont = sum(int(x) for x in re.findall(r"contact steps \(<0\): (\d+)", cl))
hm = re.search(r"hold  0-.*max\s+([\d.]+) mm", open(log).read())
hold = float(hm.group(1)) if hm else 99
ok = len(mins) == 3 and cont == 0 and min(mins.values()) >= bar and hold < 5
print(f"REHEARSAL {'PASS' if ok else 'FAIL'}: min gap {min(mins.values()) if mins else '?'} cm (bar {bar}), contacts {cont}, hold max {hold} mm (bar 5)")
if ok and not os.environ.get("NOWRITE"): open(f"{h}/rehearsal_q0.txt", "w").write(q + "\n"); open(f"{h}/rehearsal_buf.txt", "w").write(buf + "\n"); open(f"{h}/rehearsal_cfg.txt", "w").write(os.environ.get("CFG", "fr3_vcap1.2_acc10.yml") + "|" + os.environ.get("MORE", "") + "\n"); print("wrote rehearsal_q0.txt, rehearsal_buf.txt, rehearsal_cfg.txt")
PY
echo REHEARSE_DONE
