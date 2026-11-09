# Shared env for the deployment-path sim (sourced). Same container env as ../run_mpc_node.sh.
WS=/home/nexus/spot-teleop/isaac-sim_ws; FM=$WS/franka_mpc; DS=$FM/deploy_sim; VR=/home/nexus/spot-teleop/fr3_real_repro/vm_real
NVM=/home/nexus/spot-teleop/zed_lite/nvblox_msgs_jazzy/install; IMG=spot-isaac-sim:6.0.0; J=/isaac-sim/exts/isaacsim.ros2.core/jazzy
PY=/isaac-sim/extscache/omni.warp.core-1.13.0+lx64:/isaac-sim/extsDeprecated/omni.isaac.ml_archive/pip_prebundle
PY=$PY:/isaac-sim/exts/omni.pip.compute/pip_prebundle:/workspace/curobo-isaac6/src
PY=$PY:/workspace/curobo-isaac6/.venv/lib/python3.12/site-packages:/workspace/franka_mpc/pydeps:$J/rclpy:/nvm/lib/python3.12/site-packages
DSRUN=(docker run --rm --network host --ipc host -e ROS_DISTRO=jazzy -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e ROS_DOMAIN_ID=${DOMAIN:-8}
  -e LD_LIBRARY_PATH=$J/lib:/nvm/lib -e PYTHONPATH="$PY" -e PYTHONUNBUFFERED=1 -e PYTHONDONTWRITEBYTECODE=1
  -e SETUPTOOLS_SCM_PRETEND_VERSION=0.7.7.post1.dev5 -e ACCEPT_EULA=Y -v "$WS":/workspace:ro -v "$VR":/vr:ro -v "$NVM":/nvm:ro
  -v "$WS/cache/computecache":/root/.nv/ComputeCache --entrypoint /isaac-sim/python.sh)
