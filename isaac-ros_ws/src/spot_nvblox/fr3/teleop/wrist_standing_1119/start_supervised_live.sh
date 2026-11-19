#!/usr/bin/env bash
# CALLER MUST hold franka-robot and have current e-stop/empty-gripper/clear-space confirmation.
# Starts a finite 180s physical test; starts disengaged, then uses normal fresh/stable-hand Start gate.
set -euo pipefail
docker exec fr3-ros2 bash -lc 'source /opt/ros/jazzy/setup.bash; source /ros2_ws/install/setup.bash; export FASTRTPS_DEFAULT_PROFILES_FILE=/lab/fr3_zed_runtime/live_transport.xml; python3 /tmp/wrist_standing_1119/preflight_standing.py'
python3 - <<'CANCEL_EXPIRY'
import pathlib,signal,os
r=pathlib.Path('/home/hipolito/franka_teleop_checks/standing_ready_1119');p=r/'warm_expiry.pid'
if p.exists():
 pid=int(p.read_text());proc=pathlib.Path('/proc')/str(pid)/'cmdline'
 try:args=proc.read_bytes().decode().split(chr(0))
 except FileNotFoundError:args=[]
 if str(r/'expire_warmup.py') in args:os.kill(pid,signal.SIGTERM)
 p.unlink(missing_ok=True)
CANCEL_EXPIRY
docker exec fr3-ros2 bash -lc 'source /opt/ros/jazzy/setup.bash; source /ros2_ws/install/setup.bash; export FASTRTPS_DEFAULT_PROFILES_FILE=/lab/fr3_zed_runtime/live_transport.xml; export PYTHONPATH="$PYTHONPATH:/lab/fr3_zed_runtime"; exec python3 -u /lab/wrist_standing_1119/live_supervisor.py'
