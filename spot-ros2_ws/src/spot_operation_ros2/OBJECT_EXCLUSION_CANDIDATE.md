# Four-object mask candidate and target-only map filter

Status: **target-only map policy implemented and live static-scene map rebuilt; moving-object selection still needs tracker integration**.

`multi_object_exclusion_node.py` runs Ultralytics SAM2 on `/vm/cable/rgb` and
publishes source-stamped per-object PNG masks plus a union mask. The union is
published only when every required mask passes ID, size, prompt-region, and
overlap checks. Murilo selected target-only carving for the live map: only the
selected grasp target may be removed from the ESDF; every other object stays
as an obstacle. With no target selected, the depth filter passes full raw
depth. An active selection requires a fresh matching per-object mask. A lost
mask, stale selection heartbeat, unknown target, or empty remaining depth
pauses integration. The filter pairs target masks to depth by source timestamp
and verifies unmasked pixels remain unchanged.

Run the installed node with `ros2 run spot_operation_ros2 multi_object_exclusion`
only after a live-camera preflight. The candidate defaults assume the fixed
640x360 tabletop view and all four objects remain stationary in their current
positions. The prompts are not a persistent tracker: after any object or
camera movement, stop map integration and reseed/validate the prompts. The map
filter uses an explicit heartbeat on `/wrist_map/target_state`: `NO_TARGET`
passes full depth; `ACTIVE:drill` requires the drill mask topic; `LOST:drill`,
unknown, or stale state pauses integration. Do not use a one-shot state message.

The screwdriver prompt was tightened after inspecting the cached 640x360
frame: the mask shrank from 1,101 pixels to 546 pixels and its observed bounds
changed from `(220,215)-(305,250)` to `(250,216)-(306,249)`, matching the visible
tool more closely. This is an offline image result only, not a live accuracy or
map-leak measurement. Four-object inference on that cached frame took about
0.81 s after model load; it does not establish a usable live frame rate.

## Live validation still required

cuRobo currently consumes only the `/nvblox_curobo` ESDF. Therefore target
state matters: `NO_TARGET` retains all scene objects, `ACTIVE:drill` removes
only the drill, and `LOST:drill` withholds depth until a fresh mask returns.
The current static map is complete. Continuous target-mask use after the drill
moves must be wired to the tracker rather than the temporary test selection.
Offline state-gate tests live under
`isaac-ros_ws/src/spot_nvblox/fr3/teleop/wrist_drill_1117/map_filter/test`.

## 2026-11-19 lab validation

No robot motion was issued. Murilo selected `drill`. The source RGB was
`/zed1/zed_node/rgb/image_rect_color` (`bgra8`); the segmenter depth stream was
about 30 Hz. The live SAM2 masks were source-stamp matched to the registered
depth at 0 ms delta. Warm inference on GPU1 was about 1.1 s; the live mask/state
stream was about 1.3 Hz. A representative drill mask covered about 1.86% of the
frame after 8 px dilation, left 174k+ valid depth pixels, and changed zero
pixels outside the mask. An explicit `NO_TARGET` state passed raw depth with
zero removal and 178k+ valid pixels. Missing/stale target state and a lost drill
mask withheld depth.

After saving the old 11,673-vertex PLY, the single `/nvblox_curobo` node was
restarted under `ACTIVE:drill`. The rebuilt PLY contained 14,584 vertices. In a
read-only sample of the rebuilt mesh, drill upper-surface points were a median
19.6 cm (p10 6.2 cm) from mapped vertices, while tennis ball, Sprite bottle,
and screwdriver points were each within about 8-9 mm median (p90 at most 1.4
cm). A read-only ESDF query succeeded at 2 cm voxel size; sampled ESDF values
were unknown (`-1000`) at the carved drill point and approximately zero at the
three retained objects. The current MPC conversion maps unknown ESDF to free
space. The screwdriver therefore remains in the collision map by design; its
tightened segmentation prompt was separately verified on the live camera
frame. This validates a static scene, not tracking while objects move.

Runtime files/evidence are under `/tmp/wrist_map_filter/` inside `isaac-build`:
`pre_rebuild_mesh.ply`, `rebuild_drill_only_mesh_latest.ply`,
`object_map_validation.json`, `status.json`, and `filter_targetgate.log`.

Before any future integration restart, connect the target selection to the
tracker, verify moving-object behavior, and repeat the source-stamp and mask
checks. The current map was rebuilt with the drill selected and all non-target
objects retained. No controller was enabled and no robot motion was issued.

Unit checks:

```sh
PYTHONPATH=spot-ros2_ws/src/spot_operation_ros2 \
  python3 -m unittest discover \
  -s spot-ros2_ws/src/spot_operation_ros2/test \
  -p 'test_object_exclusion_union.py' -v
python3 -m unittest discover \
  -s isaac-ros_ws/src/spot_nvblox/fr3/teleop/wrist_drill_1117/map_filter/test \
  -p 'test_target_depth_gate.py' -v
```
