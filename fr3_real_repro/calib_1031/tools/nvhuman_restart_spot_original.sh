set -e
P=""; for p in $(pgrep -x nvblox_node); do tr '\0' ' ' < /proc/$p/cmdline | grep -q "__node:=nvblox_human" && P=$p; done
[ -n "$P" ] || { echo "nvblox_human not found: abort"; exit 1; }
grep -q "launch_params_bysqbya9" /tmp/nvh_cmdline.bin && grep -q "^    voxel_size: 0.02$" /tmp/launch_params_bysqbya9 || { echo "original files not as expected: abort"; exit 1; }
cat > /tmp/nvorig_launch.py <<'PY'
import os
args = open('/tmp/nvh_cmdline.bin','rb').read().split(b'\0')[:-1]
env = dict(e.split(b'=',1) for e in open('/tmp/nvh_environ.bin','rb').read().split(b'\0') if b'=' in e)
os.chdir(open('/tmp/nvh_cwd.txt').read().strip()); os.execve(args[0], args, env)
PY
kill -TERM $P
for i in $(seq 1 30); do kill -0 $P 2>/dev/null || break; sleep 0.5; done
if kill -0 $P 2>/dev/null && ! ps -o stat= -p $P | grep -q Z; then echo "SIGKILL"; kill -KILL $P; sleep 1; fi
setsid nohup python3 /tmp/nvorig_launch.py > /tmp/nvblox_human_spot2cm.log 2>&1 < /dev/null &
sleep 5
NP=""; for p in $(pgrep -x nvblox_node); do tr '\0' ' ' < /proc/$p/cmdline | grep -q "__node:=nvblox_human" && NP=$p; done
echo "nvblox_human restarted with the original Spot-equivalent params: pid $NP (old $P)"
sleep 150
grep -a -E "voxel_size: 0.0" /tmp/nvblox_human_spot2cm.log | head -1 | cut -c1-120
grep -E "ros/(color|depth)|layer_streamer/mesh" /tmp/nvblox_human_spot2cm.log | tail -4
curl -s -m 10 http://127.0.0.1:8788/mesh.bin | base64 -w0 > /tmp/mesh_spot2cm.b64; wc -c /tmp/mesh_spot2cm.b64
