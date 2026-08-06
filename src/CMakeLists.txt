# External module list for this out-of-tree repo.
#
# The root CMakeLists.txt (v1.17.0) add_subdirectory()'s this file, then for each
# entry below does add_subdirectory(${EXTERNAL_MODULES_LOCATION}/src/<entry>), so
# list module paths relative to this src/ directory. PARENT_SCOPE is required.
#
# NOTE: on v1.17.0 this must live in src/CMakeLists.txt (not a repo-root
# CMakeLists.txt); the root build reads config_module_list_external from here.

set(config_module_list_external
	modules/flow_angle
	PARENT_SCOPE
	)
