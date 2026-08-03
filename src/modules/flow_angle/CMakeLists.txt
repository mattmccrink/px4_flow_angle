px4_add_module(
	MODULE modules__flow_angle
	MAIN flow_angle
	SRCS
		FlowAngle.cpp
		flow_angle_params.c
	# No DEPENDS: external modules are configured before src/lib, so depending on
	# a src/lib target (e.g. px4_work_queue) fails with "non-existent target".
	# A thread-based ModuleBase needs no such dependency. The I2CSPIDriver refactor
	# in milestone 2 will need drivers__device / px4_work_queue, which forces a
	# small, documented reorder patch to the PX4 root CMakeLists (see README).
	EXTERNAL
)
