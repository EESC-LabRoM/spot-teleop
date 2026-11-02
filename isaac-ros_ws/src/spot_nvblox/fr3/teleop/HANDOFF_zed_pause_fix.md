# ZED Start/pause fix — completed and deployed, 2026-10-31 00:04 BRT

Canonical implementation and two regression files were implemented by Codex and reviewed by the supervisor. Incremental patch: /tmp/fr3-zed-fix.patch; worker report: /tmp/fr3-zed-fix-report.md.

Start uses shared read-only preflight (start_reason) rather than requiring armed mode_ready; the existing guarded prepare still activates/arms only on operator Start. Paused/stale ZED sends fresh measured TF hold goals to the planner, without physical joint output while paused. Readiness uses the same command predecessor as output, retains hard guards, exposes the actual cause and clears obsolete fault notes.

Supervisor verification: 46 focused tests including 5 independent checks passed; 15 selected existing input tests passed; 10 checks of the actual served Start/Home expressions passed; changed-file correctness lint passed. Existing unrelated trailing whitespace at lab_dashboard.html:487 was preserved. Tests used mock bridge/output and isolated DDS191; no real motion test was performed during repair.

Deployed canonical main, review wrapper, web backend, top-level policy loader, curobo/curobo_start.py and curobo_live/joystick_bridge.py under /home/murilo/franka_ws/lab/fr3_zed_runtime. The distinct existing fr3_dashboard.html was preserved. Updated port8788 HTML is /tmp/nvblox_real/lab_dashboard.html in isaac-build. Restarted teleop and bridge disarmed, speed_scale=rot_scale=0.1, reviewed full_6d mapping. Remote dashboard8089 is automatically restarted by its existing supervisor and was also refreshed.

Last live evidence: engaged=false, armed=false, JTC inactive, command_ready=true, start_reason=null, new bridge sent=0. Actual Stop through8788 succeeded and Start eligibility remained available after Stop. Hand tracker currently requests visibility of the right arm; operator should hard-refresh8788, click Start, show right shoulder/elbow/wrist/fingers and hold still for calibration.

Rollback backups: fr3-ros2:/tmp/zed-runtime-before-fix; isaac-build:/tmp/zed-dashboard-before-fix.html. Supervisor check source /tmp/test_nexus_zed_review.py, live snapshot /tmp/zed-live-state-after-fix.json. Temporary container test staging was removed.

Provider configuration: user authorized setup; existing ChatGPT subscription was validated. ~/.omnigent/config.yaml now aliases legacy openai-budget to the existing Codex subscription. It uses the subscription allowance. Previous config backup /tmp/omnigent-config-before-zed-provider.yaml. No password was used or stored.
