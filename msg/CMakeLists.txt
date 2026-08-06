# External uORB message list for this out-of-tree module.
#
# The in-tree msg/CMakeLists.txt (v1.17.0) add_subdirectory()'s this file and then
# prepends ${EXTERNAL_MODULES_LOCATION}/msg/ to each entry below, so list bare
# filenames only. PARENT_SCOPE hands the list back to the in-tree msg build.

set(config_msg_list_external
	SensorFlowAngle.msg
	PARENT_SCOPE
	)
