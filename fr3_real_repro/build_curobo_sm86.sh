#!/usr/bin/env bash
# Build cuRobo 0.7.7 CUDA extensions for the A2000 (sm_86 only) inside the Isaac Sim 6.0 container (Isaac's python 3.12 + torch 2.11+cu128).
# g++ and setuptools/ninja are installed in the throwaway container; the .so files land in the mounted source tree.
set -uo pipefail
D=/home/nexus/spot-teleop/fr3_real_repro
docker run --rm --gpus all -e ACCEPT_EULA=Y -v /home/nexus/spot-teleop/isaac-sim_ws:/workspace:rw --entrypoint bash spot-isaac-sim:6.0.0 -c '
set -e
echo "[build] $(date +%T) apt g++"; apt-get update -qq >/dev/null && apt-get install -y -qq --no-install-recommends g++ make >/dev/null; g++ --version | head -1
echo "[build] $(date +%T) pip setuptools wheel ninja"; /isaac-sim/python.sh -m pip install -q --no-cache-dir --target /tmp/bdeps setuptools wheel ninja 2>&1 | tail -2
export CUDA_HOME=/workspace/curobo-isaac6/cuda-12.8 TORCH_CUDA_ARCH_LIST=8.6 MAX_JOBS=4 SETUPTOOLS_SCM_PRETEND_VERSION=0.7.7.post1.dev5
export PATH=$CUDA_HOME/bin:/tmp/bdeps/bin:$PATH
export PYTHONPATH=/tmp/bdeps:/isaac-sim/extsDeprecated/omni.isaac.ml_archive/pip_prebundle:/isaac-sim/exts/omni.pip.compute/pip_prebundle:/workspace/curobo-isaac6/.venv/lib/python3.12/site-packages
cd /workspace/curobo-isaac6
echo "[build] $(date +%T) build_ext (sm_86, MAX_JOBS=4)"
nice -n 10 /isaac-sim/python.sh setup.py build_ext --inplace 2>&1 | grep -v -E "^\s*$" | tail -400
echo "[build] $(date +%T) .so files:"; ls -la src/curobo/curobolib/*.so
' > $D/build_curobo.log 2>&1
echo "EXIT $?" > $D/build_curobo.done
