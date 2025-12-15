import omni.kit.commands
anims = ["/World/stand_idle_loop_skelanim"]
omni.kit.commands.execute("CreateRetargetAnimationsCommand",
						  source_skeleton_path="/World/biped_demo_meters/Root",  # source skel
						  target_skeleton_path="/World/male_adult_construction_05_new/male_adult_construction_05/ManRoot/male_adult_construction_05/male_adult_construction_05/male_adult_construction_05",  # target skel
						  source_animation_paths=anims,  # typing.List[str] Of Animations 
						  target_animation_parent_path="/World/male_adult_construction_05_new",  # Prim To Save The anims under
						  set_root_identity=False)