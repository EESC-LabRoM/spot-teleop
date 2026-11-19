# Four-object exclusion candidate (offline only)

Status: **not deployed and not ready for teleop or autonomous grasp**.

`multi_object_exclusion_node.py` runs Ultralytics SAM2 on `/vm/cable/rgb` and
publishes source-stamped per-object PNG masks plus a union mask. The union is
published only when every required mask passes ID, size, prompt-region, and
overlap checks. The existing depth filter can select the union topic through
its `mask_topic` parameter. It still pairs masks to depth by source timestamp
and verifies the filtered output does not alter unmasked depth pixels.

Run the installed node with `ros2 run spot_operation_ros2 multi_object_exclusion`
only after a live-camera preflight. The candidate defaults assume the fixed
640x360 tabletop view and all four objects remain stationary in their current
positions. The prompts are not a persistent tracker: after any object or
camera movement, stop map integration and reseed/validate the prompts. Missing,
stale, malformed, or rejected masks withhold the union; the filter then pauses
instead of passing unmasked depth.

The screwdriver prompt was tightened after inspecting the cached 640x360
frame: the mask shrank from 1,101 pixels to 546 pixels and its observed bounds
changed from `(220,215)-(305,250)` to `(250,216)-(306,249)`, matching the visible
tool more closely. This is an offline image result only, not a live accuracy or
map-leak measurement. Four-object inference on that cached frame took about
0.81 s after model load; it does not establish a usable live frame rate.

## Collision-map decision required

cuRobo currently consumes only the `/nvblox_curobo` ESDF. No independent
per-object collision-cloud feed for the tennis ball, Sprite bottle, screwdriver,
or drill has been verified. Removing all four masks from nvblox would also
remove those objects from cuRobo's obstacle world. The current depth filter
also rejects an empty mask; a map rebuild with no valid selected-object mask
can therefore leave an empty map. Do not clear/rebuild the live map until the
operator has chosen a policy and verified that the planner retains obstacles.

Before enabling this candidate, validate on live frames that (1) each mask
tracks its named object and does not include background, (2) RGB/depth source
stamps and optical frames match within the filter slop, (3) the union is fresh
at the point of use, and (4) a map replay shows excluded pixels removed while
all unmasked pixels and required collision obstacles remain. Then validate the
single live map/ESDF before enabling any controller. No TF, map, or controller
is changed by this candidate.

Unit checks:

```sh
PYTHONPATH=spot-ros2_ws/src/spot_operation_ros2 \
  python3 -m unittest discover \
  -s spot-ros2_ws/src/spot_operation_ros2/test \
  -p 'test_object_exclusion_union.py' -v
```
