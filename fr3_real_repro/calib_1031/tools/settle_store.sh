# inside isaac-build, right after a restart of nvblox_human: sample the long-lived dashboard store against a fresh server's store while the map builds
cd /tmp/nvblox_real; source /opt/ros/humble/setup.bash; source /workspaces/isaac_ros-dev/install/setup.bash; export ROS_DOMAIN_ID=8 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
setsid nohup python3 -u mesh_server.py --node nvblox_human --port 8799 --bind 127.0.0.1 > /tmp/claude_mesh_clean.log 2>&1 < /dev/null &
SPID=$!; T0=$(date +%s)
for i in 1 2 3 4 5 6 7 8; do sleep 18
python3 - $(( $(date +%s) - T0 )) <<'PY'
import struct, sys, urllib.request
out = []
for nm, port in (("long-lived", 8788), ("fresh", 8799)):
    try: d = urllib.request.urlopen(f"http://127.0.0.1:{port}/mesh.bin", timeout=8).read(); out.append(f"{nm} {struct.unpack_from('<3I', d, 0)[1]:6d} verts")
    except Exception as e: out.append(f"{nm} n/a")
print(f"+{int(sys.argv[1]):3d}s  " + "   ".join(out))
PY
done
kill $SPID 2>/dev/null; sleep 1; pkill -f "[p]ort 8799" 2>/dev/null; echo "clean server left: $(pgrep -f '[p]ort 8799' | wc -l)"
