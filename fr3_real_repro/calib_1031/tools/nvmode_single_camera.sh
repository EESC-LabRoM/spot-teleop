# runs INSIDE isaac-build as root. usage: bash -s MODE   (MODE = both | zed1 | zed2)
# restarts nvblox_human (3 cm, 12 cm truncation) integrating only the chosen camera(s); then waits and saves a PLY
set -e
MODE=$1; WAIT=${WAIT:-60}
P=""; for p in $(pgrep -x nvblox_node); do tr '\0' ' ' < /proc/$p/cmdline | grep -q "__node:=nvblox_human" && P=$p; done
[ -n "$P" ] || { echo "nvblox_human not found: abort"; exit 1; }
BASE=/tmp/launch_params_human_3cm; grep -q "^    voxel_size: 0.03$" $BASE || { echo "base params not as expected: abort"; exit 1; }
NEWP=/tmp/launch_params_human_3cm_$MODE
if [ "$MODE" = both ]; then cp $BASE $NEWP; else sed -e 's/^    num_cameras: 2$/    num_cameras: 1/' $BASE > $NEWP; fi
python3 - "$MODE" "$BASE" "$NEWP" <<'PY'
import sys
mode, base, newp = sys.argv[1:4]
a = open('/tmp/nvh3_cmdline.bin','rb').read().split(b'\0')[:-1]
assert base.encode() in a
a = [newp.encode() if x == base.encode() else x for x in a]
if mode == 'zed2':      # camera_0 slot reads zed2's topics
    a = [x.replace(b':=/franka_seg_zed1/', b':=/franka_seg_zed2/').replace(b':=/zed1/', b':=/zed2/') if x.startswith(b'camera_0/') else x for x in a]
open('/tmp/nvm_cmdline.bin','wb').write(b'\0'.join(a) + b'\0')
print([x.decode() for x in a if x.startswith(b'camera_0/') or x.startswith(b'camera_1/')][:4])
PY
grep -n "num_cameras" $NEWP
cat > /tmp/nvm_launch.py <<'PY'
import os
args = open('/tmp/nvm_cmdline.bin','rb').read().split(b'\0')[:-1]
env = dict(e.split(b'=',1) for e in open('/tmp/nvh_environ.bin','rb').read().split(b'\0') if b'=' in e)
os.chdir(open('/tmp/nvh_cwd.txt').read().strip()); os.execve(args[0], args, env)
PY
kill -TERM $P
for i in $(seq 1 30); do kill -0 $P 2>/dev/null || break; sleep 0.5; done
if kill -0 $P 2>/dev/null && ! ps -o stat= -p $P | grep -q Z; then echo "still alive after 15 s: SIGKILL"; kill -KILL $P; sleep 1; fi
setsid nohup python3 /tmp/nvm_launch.py > /tmp/nvblox_human_$MODE.log 2>&1 < /dev/null &
sleep 4
NP=""; for p in $(pgrep -x nvblox_node); do tr '\0' ' ' < /proc/$p/cmdline | grep -q "__node:=nvblox_human" && NP=$p; done
echo "mode $MODE: new nvblox_human pid $NP (old $P); waiting ${WAIT}s for the map to build"
sleep $WAIT
kill -0 $NP || { echo "nvblox_human died"; tail -5 /tmp/nvblox_human_$MODE.log; exit 1; }
grep -E "ros/(color|depth)|layer_streamer/mesh" /tmp/nvblox_human_$MODE.log | tail -4
for i in 1 2; do curl -s -m 5 -D - -o /tmp/mb.bin http://127.0.0.1:8788/mesh.bin | grep -i "x-map-age" | tr -d "\r\n"; python3 -c "import struct;print('  store (version,nverts,ntris) =', struct.unpack('<3I', open('/tmp/mb.bin','rb').read(12)))"; sleep 3; done
