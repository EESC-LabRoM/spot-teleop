# VM103 offload (grasp lane, cut over 2026-11-02 on Murilo's go)

Compute moved off the lab PC (`hipolito-Precision-5860-Tower`) onto VM103
(`ubuntu-gpu`, `10.0.0.3`) over the direct 10G cable (`10.0.0.0/24`, no Tailscale).
ROS domain 8, FastDDS pinned to the cable + SHM (see `cable.xml`: UDP whitelist
`10.0.0.3`, initial peers `10.0.0.2` (lab) and `10.0.0.3` (self; VM-local nodes
need it for mutual discovery)). Lab GPU1 util went 87% -> 0%.

## Live input contract (what the VM nodes consume)

| VM node (`__node`) | container | subscribes (lab-hosted) | serves |
|---|---|---|---|
| `nvblox_curobo` | `vm-nvblox-prod` (GPU1) | depth `/franka_seg_zed1/depth` + `/zed1/zed_node/depth/camera_info`; color `/zed1/zed_node/left/image_rect_color` + `/zed1/zed_node/left/camera_info`; TF `/tf`+`/tf_static` | `/nvblox_curobo/get_esdf_and_gradient`, `/nvblox_curobo/mesh`, ... |
| `sam2_tracker_node` | `vm-tracker-prod` (GPU1) | rgb `/zed1/zed_node/left/image_rect_color`, depth `/franka_seg_zed1/depth`, info `/zed1/zed_node/depth/camera_info`, `/perception/seed_command`, `/tracking/seed_pixel` | `/zed1/segmentation_mask` (mono8), `/tracking/seed_3d`, `/tracking_state`, `/tracking_3d_point`, `/target_object/clear_box` |
| `vm_seed_pixel_bridge` | `vm-px-bridge` (CPU) | `/tracking/seed_3d`, depth `camera_info` | `/tracking/seed_pixel` |

Key points for freshness/health readers:

- **Depth IS still robot-self-filtered.** `use_segmentation:=false` means only that
  the SAM2 *target mask* (`/zed1/target_mask`) is not consumed. The depth input is
  `/franka_seg_zed1/depth` from `franka-seg-zed1` on the lab (arm filtered out).
- **ZED1 only.** Nothing on the VM subscribes any `zed2` topic (ZED2 is down at the
  source: mount fell, wrapper silent). A stale `/franka_seg_zed2/depth` is expected
  and irrelevant to the VM map. Watch `/franka_seg_zed1/depth` only.
- **Color is `left/image_rect_color`**, not the `/zed1/zed_node/rgb/*` alias (that
  alias doesn't cross to the VM; same for `/zed1/target_mask`, see below).
- Mapping: 2 cm voxels, `static_tsdf`, integrate 5 Hz, mesh/ESDF 2 Hz.
  Measured ESDF: 8-90 ms, 0.07 MB replies for the 0.5x0.6x0.45 m work volume —
  but the lab MPC node pulls the FULL 1.4x1.6x1.2 m grid (336k voxels, ~1.4 MB,
  0.7-4 s over the cable). Size every consumer by ITS OWN aabb; the 0.07 MB
  number does not apply to the planner feed.

## ESDF service bridge 142 -> 8 (lab-local `claude-esdf-proxy`)

The lab MPC node (`fr3-zed-live-mpc`, domain 142) cannot call the VM map
(domain 8) directly; `fr3-zed-relay` bridges topics only. `esdf_proxy_142to8.py`
forwards the service (dual-context rclpy, 142 server / 8 client). v2 (2026-11-03)
sends ONE upstream call per node call (8 s budget) + 1.5 s cache for identical
AABBs; v1's 0.4 s retry spam self-DDoS'd the 1.4 MB responses into 31k timeouts
and a stale planner map (see 2026-11-03 grasp post-mortem: the arm retreated on
a 98-minute-old grid). Deployed at `/home/murilo/esdf_proxy_v2/esdf_proxy.py` on
the lab, shadow-mounted over `/p/esdf_proxy.py` in the container (the
`live_zed/` original is Hipolito-owned, read-only mount). Verify: proxy log
shows `fail=0` + rising `cache`, and the node log shows `ESDF voxel grid
updated` with no `ESDF request timed out`.

## Known gaps (non-blocking while the drill is off the table)

- `/zed1/target_mask` and `/zed1/zed_node/rgb/*` don't arrive on the VM (lab-local
  publishers; VM probes never match them). Hence segmentation-free mapping.
- Lab `mask_sync` was restarted fresh but receives nothing (not even lab-local
  depth); 0 subscribers, harmless. Must be fixed before drill carving returns.
- Tracker: seed ONCE (rapid reseeds crash the SAM video predictor).

## Files here

- `run_vm_nvblox.sh`, `run_vm_tracker.sh`, `run_vm_seed_bridge.sh`: exact launches.
- `cable.xml`: FastDDS profile shared by all three.
- `seed_pixel_bridge.py`: topic-parameterized bridge (`dur seed3_topic seedpx_topic`).
- `nvblox_base.yaml`, `specializations/nvblox_zed.yaml`: lab's Isaac ROS params,
  copied read-only 2026-11-02 (originals in `isaac-build` at
  `/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/...`).
- Tracker image `vm-tracker:sam2`: `FROM nvblox-lite:3.2.5-humble` + pinned
  `numpy==1.26.4 opencv-python==4.11.0.86 pillow==12.0.0 torch==2.9.0
  torchvision==0.24.0 ultralytics==8.4.24` + `/tmp/trackpkg` (spotop sources +
  `sam2.1_t.pt`, fetched from lab `isaac-build:/tmp/trackpkg` 2026-11-02).

## Rollback (restart on lab, stop VM twins first to avoid duplicate pubs)

Tracker (in `isaac-build`):
`bash -c "source /opt/ros/humble/setup.bash && source /workspaces/isaac_ros-dev/install/setup.bash && export CUDA_VISIBLE_DEVICES=1 ROS_DOMAIN_ID=8 && cd /tmp/trackpkg && python3 -m spotop.sam2_tracker_node --ros-args -p rgb_topic:=/zed1/zed_node/left/image_rect_color -p depth_topic:=/franka_seg_zed1/depth -p depth_info_topic:=/zed1/zed_node/depth/camera_info -p segmentation_mask_topic:=/zed1/segmentation_mask -p visualize:=false"`

nvblox (in `isaac-build`, GPU1): same flags as `run_vm_nvblox.sh` plus the lab's
2-camera remaps and `mapping_type:=human_with_static_tsdf use_segmentation:=true`
(see lane history for the verbatim 2026-11-02 21:50 command line).

## Viewers (container `vm-viewer`, port 8791)

`seg_view.py` + `grasp_render.py` (this dir), served from VM103 over the cable —
open from the lab PC browser (verified 200 from the lab):

- index: `http://10.0.0.3:8791/`
- live segmentation: `http://10.0.0.3:8791/seg` (MJPEG + coverage/age/fps pills;
  also publishes `/zed1/segmentation_overlay` for recording)
- grasp vs cloud: `http://10.0.0.3:8791/grasp` (`?run=<id>` overrides latest;
  renders pick frame + TCP + approach + finger span over the masked cloud)
- interactive grasp 3D: `http://10.0.0.3:8791/grasp3d` (same selector; orbit /
  zoom / pan the cloud + gripper meshes + TCP + frame; data `/grasp.json`).
  Also embedded in the main dashboard below "Robot segmentation" (proxied as
  `/graspjson`, same three.js r160 as the map panel).

(Not inside Hipolito's `:8788` dashboard: that file is his and read-only to us.
These are standalone pages next to it.) Start command is the container's own
command line (`docker inspect vm-viewer`); restart: `docker restart vm-viewer`.
