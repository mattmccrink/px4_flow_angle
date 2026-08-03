# Top-level CMake for the out-of-tree flow_angle module.
# PX4 reads this when built with EXTERNAL_MODULES_LOCATION pointing at this repo.
# The path is relative to <this repo>/src/
set(config_module_list_external
	modules/flow_angle
	PARENT_SCOPE
)
