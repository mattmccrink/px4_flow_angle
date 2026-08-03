# PX4 builds external modules by doing:
#   add_subdirectory("${EXTERNAL_MODULES_LOCATION}/src" external_modules)
# so THIS file (src/CMakeLists.txt) is where config_module_list_external must be
# set. PARENT_SCOPE pushes it up to the root CMake scope, which then iterates the
# list and add_subdirectory()s each entry, relative to this src/ directory.
set(config_module_list_external
	modules/flow_angle
	PARENT_SCOPE
)
