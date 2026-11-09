# inside isaac-build: side nvblox instances with candidate map settings (live nvblox_curobo untouched), capture each ESDF, then stop ONLY them
set -e
P=""; for p in $(pgrep -x nvblox_node); do tr '\0' ' ' < /proc/$p/cmdline | grep -q "__node:=nvblox_curobo" && P=$p; done; [ -n "$P" ] || { echo "nvblox_curobo not found"; exit 1; }
PF=$(tr '\0' '\n' < /proc/$P/cmdline | grep -o '/tmp/launch_params_[A-Za-z0-9_]*' | tail -1); echo "live params file: $PF (pid $P)"; grep -E "voxel_size|esdf_integrator_min_weight|mesh_integrator_min_weight|max_weight|truncation_distance_vox" $PF | tr '\n' ' '; echo
cp /proc/$P/environ /tmp/cand_environ.bin
mk() {  # name voxel esdfmin meshmin
  sed -e "s/^    voxel_size: .*/    voxel_size: $2/" -e "s/^    static_mapper.esdf_integrator_min_weight: .*/    static_mapper.esdf_integrator_min_weight: $3/" -e "s/^    static_mapper.mesh_integrator_min_weight: .*/    static_mapper.mesh_integrator_min_weight: $4/" $PF > /tmp/launch_params_$1
  grep -q "esdf_integrator_min_weight" /tmp/launch_params_$1 || echo "static_mapper.esdf_integrator_min_weight: $3" >> /tmp/launch_params_$1
  python3 - "$1" "$PF" <<'PY'
import sys
n, pf = sys.argv[1:3]; a = open(f'/proc/{sys.argv[0] if False else 0}/cmdline','rb').read() if False else None
PY
  python3 - "$1" "$PF" "$P" <<'PY'
import sys
n, pf, pid = sys.argv[1:4]
a = open(f'/proc/{pid}/cmdline','rb').read().replace(b"__node:=nvblox_curobo", f"__node:={n}".encode()).replace(pf.encode(), f"/tmp/launch_params_{n}".encode())
open(f'/tmp/{n}_cmdline.bin','wb').write(a)
open(f'/tmp/{n}_launch.py','w').write(f"import os\nargs=open('/tmp/{n}_cmdline.bin','rb').read().split(b'\\0')[:-1]\nenv=dict(e.split(b'=',1) for e in open('/tmp/cand_environ.bin','rb').read().split(b'\\0') if b'=' in e)\nos.chdir('/workspaces/isaac_ros-dev'); os.execve(args[0], args, env)\n")
PY
  setsid nohup python3 /tmp/$1_launch.py > /tmp/nvblox_$1.log 2>&1 < /dev/null &
}
mk cand_a 0.05 0.1 0.1; mk cand_b 0.03 0.5 0.5; mk cand_c 0.03 0.1 0.1
sleep ${WAIT:-150}
source /opt/ros/humble/setup.bash; source /workspaces/isaac_ros-dev/install/setup.bash; export ROS_DOMAIN_ID=8 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
for n in cand_a cand_b cand_c; do
  echo "== $n: $(grep -E '^ros/depth ' /tmp/nvblox_$n.log | tail -1 | tr -s '\t ' ' ')"
  timeout 90 python3 /tmp/esdf_capture.py --out /tmp/claude_esdf_$n.npz --service /$n/get_esdf_and_gradient --center 0.35 0 0.55 --size 1.6 1.8 1.3 2>&1 | grep -E "captured|failed|no service" | cut -c1-200
done
for n in cand_a cand_b cand_c; do for p in $(pgrep -x nvblox_node); do tr '\0' ' ' < /proc/$p/cmdline | grep -q "__node:=$n " && kill -TERM $p; done; done; sleep 3
echo "live nvblox nodes after cleanup: $(for p in $(pgrep -x nvblox_node); do tr '\0' ' ' < /proc/$p/cmdline | grep -o '__node:=[a-z_]*'; done | tr '\n' ' ')"
