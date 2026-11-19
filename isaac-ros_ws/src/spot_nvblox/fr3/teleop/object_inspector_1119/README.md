# Object masks and collision-map inspector, Nov19

User wants to see drill, tennis ball, small Sprite bottle and Phillips screwdriver segmentation and inspect map leakage. This is an ADDITIVE READ-ONLY diagnostic: no ROS imports in the server/inference scripts; no target seed publication, TF changes, map reset, depth-filter changes, controller activation or robot motion.

Current server: loopback ONLY, http://127.0.0.1:47814, timed25min lifetime. Network exposure at100.88.134.127:47814 was rejected by automatic approval review; the explicit address-specific user approval question remains pending. Do NOT bind that address until approved. Local alternative was built and verified. Resource claims cover tower/port/VM stats polling and outlast server TTL; release after it exits or before yielding.

The page displays:
- Actual four-object SAM2 snapshot masks, individually toggleable, RGB comparison, prompt boxes and per-object mask thumbnails.
- Existing canonical live selected-target overlay and cached tracker state.
- Current mask freshness, filter heartbeat, timestamp of the last SUCCESSFUL source frame, input exclusion counters only when current, and refusal reason.
- Embedded existing accumulated nvblox map at the existing map endpoint. Old mesh occupancy is NOT automatically certified clean by depth exclusion.

Four-object preview uses a cached /segsnap JPEG and manually placed boxes on the actual four visible objects. snapshot_segment.py runs existing Ultralytics SAM2Tiny weights /tmp/trackpkg/sam2.1_t.pt in a separate short-lived process inside vm-tracker-prod, GPU1. The canonical tracker process/state/topics are untouched. It returned4 masks in0.952s inference, model load0.627s, peak CUDA allocation483MB. This proves snapshot segmentation, not4-object video FPS, object-class recognition or valid grasp depth. Each mask's survival under the existing drill5x5 erosion is displayed.

Actual observed live state: LOST/IDLE,0%coverage; filter heartbeat current but last accepted source stamp1795041797 (~19h old). New depth integration is paused on empty masks; retained map remains. Top-level drill.json ok/live was therefore misleading; the inspector does not trust it. Last-success zeros masked_nonzero/outside_changed are NOT shown as current validation. Calibration lane c66ecc79 informed; its arm session finished Home/passive without incident, offline fit still running and TF/map changes require coordination.

Sources: existing cached HTTP /segsnap, VM :8791/stats.json and local isaac-build:/tmp/wrist_map_filter/status.json. No new camera subscriptions. HTTP polling explicitly coordinated with calibration lane. Missing/stale/empty sources produce NOT VERIFIED. Server rejects POST. The whitelist serves only inspector assets, not arbitrary filesystem paths.

Validation:7 audit tests (old accepted frame with live heartbeat, empty mask, stale heartbeat, future frame, successful exclusion, nonzero inside, changed outside); all passed. JavaScript syntax checked with Node. All local HTTP page/JSON/images tested200. Headless browser screenshot visually checked (all4 masks, live data, status and embedded map). Images and inference logs stay only in local /home/hipolito/franka_teleop_checks/object_inspector_1119; no camera pixels uploaded to Git.

Run server.py with default loopback binding after claiming port/tower/VM resources. Needs local source.jpg, the4 *_mask.png files and preview.json from run_preview.py. run_preview.py does one offline cached-image inference over existing SSH; it is NOT a live target selector. It requires the existing VM container/model and current box prompts. Re-seed boxes when objects move; do not pretend stale fixed prompts track objects. The server itself never launches inference.
