# ---------------------------------------------------------------------------
# Out-of-tree compile-environment fix (PX4 v1.17.0)
#
# An external add_subdirectory() does NOT inherit the root CMakeLists' directory-
# scoped platform flags set by px4_add_common_flags() / px4_os_add_flags() (root
# CMakeLists lines ~368-369). The reorder patch fixes target *existence* so
# DEPENDS resolve, but not this compile environment.
#
# Symptom: __PX4_NUTTX ends up undefined in this module's translation units, so
# <lib/drivers/device/i2c.h> takes its posix branch and device::I2C (guarded by
# #if defined(CONFIG_I2C)) is compiled out -> "expected class-name before ','"
# on the I2CSPIDriver base class.
#
# We can't just re-call px4_os_add_flags() -- it creates the 'copy_header' custom
# target and a second call errors. So re-apply the specific pieces directly: the
# platform defines, plus the root directory's include dirs (which carry the NuttX
# config header where CONFIG_I2C is defined). Gated to NuttX so SITL/posix builds
# are untouched.
# ---------------------------------------------------------------------------
if(${PX4_PLATFORM} STREQUAL "nuttx")
	add_definitions(-D__PX4_NUTTX -D_SYS_CDEFS_H_ -D_SYS_REENT_H_)

	get_directory_property(_fa_root_incs DIRECTORY ${PX4_SOURCE_DIR} INCLUDE_DIRECTORIES)
	if(_fa_root_incs)
		include_directories(${_fa_root_incs})
	endif()
endif()

px4_add_module(
	MODULE modules__flow_angle
	MAIN flow_angle
	SRCS
		flow_angle_main.cpp
		FlowAngle.cpp
		FlowAngle.hpp
	DEPENDS
		px4_work_queue
		drivers__device
	EXTERNAL
	)
