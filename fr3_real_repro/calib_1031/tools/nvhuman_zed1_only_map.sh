# inside isaac-build: bash -s TAG   (env WAIT). nvblox_human = Spot params (2 cm), ONLY zed1 (num_cameras 1, camera_0 = zed1); settle; save dashboard mesh to /tmp/mesh_z1_TAG.b64
set -e
TAG=$1; WAIT=${WAIT:-110}; OLD=/tmp/launch_params_bysqbya9; NEW=/tmp/launch_params_spot_z1
P=""; for p in $(pgrep -x nvblox_node); do tr '\0' ' ' < /proc/$p/cmdline | grep -q "__node:=nvblox_human" && P=$p; done
[ -n "$P" ] || { echo "nvblox_human not found: abort"; exit 1; }
grep -q "^    voxel_size: 0.02$" $OLD || { echo "orig params not as expected"; exit 1; }
sed -e 's/^    num_cameras: 2$/    num_cameras: 1/' $OLD > $NEW
python3 - <<PY
a = open('/tmp/nvh_cmdline.bin','rb').read().split(b'\0')[:-1]
assert b'$OLD' in a; a = [b'$NEW' if x == b'$OLD' else x for x in a]      # camera_0 stays zed1
open('/tmp/nvz1_cmdline.bin','wb').write(b'\0'.join(a) + b'\0')
PY
cat > /tmp/nvz1_launch.py <<'PY'
import os
args = open('/tmp/nvz1_cmdline.bin','rb').read().split(b'\0')[:-1]
env = dict(e.split(b'=',1) for e in open('/tmp/nvh_environ.bin','rb').read().split(b'\0') if b'=' in e)
os.chdir(open('/tmp/nvh_cwd.txt').read().strip()); os.execve(args[0], args, env)
PY
kill -TERM $P; for i in $(seq 1 30); do kill -0 $P 2>/dev/null || break; sleep 0.5; done
if kill -0 $P 2>/dev/null && ! ps -o stat= -p $P | grep -q Z; then kill -KILL $P; sleep 1; fi
setsid nohup python3 /tmp/nvz1_launch.py > /tmp/nvblox_human_z1_$TAG.log 2>&1 < /dev/null &
sleep $WAIT
if ! grep -q "^ros/depth_image_callback" /tmp/nvblox_human_z1_$TAG.log; then echo "NO depth callbacks (discovery?): see log"; fi
grep -E "^ros/(depth|depth_image_callback)" /tmp/nvblox_human_z1_$TAG.log | tail -2
curl -s -m 10 http://127.0.0.1:8788/mesh.bin | base64 -w0 > /tmp/mesh_z1_$TAG.b64; echo "mesh $TAG: $(python3 -c "import base64,struct;d=base64.b64decode(open('/tmp/mesh_z1_$TAG.b64').read());print(struct.unpack_from('<3I',d,0)[1],'vertices')")"
