# One masked nvblox for Murilo's cuRobo on the real FR3 (2026-10-16, robot static, nothing commanded)
Files: curobo_map_fresh_masked_vs_raw.{json,png} (fresh raw ref node vs the masked /nvblox_node, same scene), curobo_map_before_swap.* (old
raw /nvblox_node vs old /fr3_nvblox, the latter had the drill integrated before the mask tracked it), curobo_map_final_*.json (single node
after the swap / after the restart test), plan_only.json (cuRobo MotionGen, 8 plans).
Scripts: ../curobo_map_compare.py, ../fr3_curobo_plan_only.py, ../start_nvblox_masked.sh, ../run_nvblox_real_masked.sh, ../../launch/fr3_nvblox.launch.py
