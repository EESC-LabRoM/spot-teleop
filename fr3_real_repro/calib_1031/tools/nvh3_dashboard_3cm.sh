# runs INSIDE isaac-build as root: restart nvblox_human (dashboard map) at voxel 0.03 m, same metric truncation (12 cm), via re-exec of the captured cmdline
set -e
P=2888563; OLD=/tmp/launch_params_bysqbya9; NEW=/tmp/launch_params_human_3cm
tr '\0' ' ' < /proc/$P/cmdline | grep -q "__node:=nvblox_human" || { echo "pid $P is not nvblox_human: abort"; exit 1; }
grep -q "^    voxel_size: 0.02$" $OLD || { echo "old params not as expected: abort"; exit 1; }
cp /proc/$P/cmdline /tmp/nvh_cmdline.bin; cp /proc/$P/environ /tmp/nvh_environ.bin; readlink /proc/$P/cwd > /tmp/nvh_cwd.txt
[ -s /tmp/nvh_cmdline.bin ] && [ -s /tmp/nvh_environ.bin ] || { echo "capture empty: abort"; exit 1; }
sed -e 's/^    voxel_size: 0.02$/    voxel_size: 0.03/' \
    -e 's/^    dynamic_mapper.projective_integrator_truncation_distance_vox: 6.0$/    dynamic_mapper.projective_integrator_truncation_distance_vox: 4.0/' \
    -e 's/^    static_mapper.projective_integrator_truncation_distance_vox: 6.0$/    static_mapper.projective_integrator_truncation_distance_vox: 4.0/' \
    -e 's/^    foreground_mapper.projective_integrator_truncation_distance_vox: 10.0$/    foreground_mapper.projective_integrator_truncation_distance_vox: 7.0/' $OLD > $NEW
diff $OLD $NEW || true
python3 - <<PY
a = open('/tmp/nvh_cmdline.bin','rb').read().split(b'\0')[:-1]
assert b'$OLD' in a
a = [b'$NEW' if x == b'$OLD' else x for x in a]
open('/tmp/nvh3_cmdline.bin','wb').write(b'\0'.join(a) + b'\0')
PY
cat > /tmp/nvh3_launch.py <<'PY'
import os
args = open('/tmp/nvh3_cmdline.bin','rb').read().split(b'\0')[:-1]
env = dict(e.split(b'=',1) for e in open('/tmp/nvh_environ.bin','rb').read().split(b'\0') if b'=' in e)
os.chdir(open('/tmp/nvh_cwd.txt').read().strip()); os.execve(args[0], args, env)
PY
echo "captured $(wc -c < /tmp/nvh_cmdline.bin) B cmdline, cwd $(cat /tmp/nvh_cwd.txt)"
kill -TERM $P
for i in $(seq 1 30); do kill -0 $P 2>/dev/null || break; sleep 0.5; done
if kill -0 $P 2>/dev/null && ! ps -o stat= -p $P | grep -q Z; then echo "still alive after 15 s: SIGKILL"; kill -KILL $P; sleep 1; fi
echo "old gone: $(ps -o stat= -p $P 2>/dev/null || echo yes)"
setsid nohup python3 /tmp/nvh3_launch.py > /tmp/nvblox_human_3cm.log 2>&1 < /dev/null &
sleep 4; echo "new nvblox_human pids: $(pgrep -f '__node:=nvblox_human' | tr '\n' ' ')"
