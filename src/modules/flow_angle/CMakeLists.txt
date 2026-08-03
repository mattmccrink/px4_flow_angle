px4_add_module(
	MODULE modules__flow_angle
	MAIN flow_angle
	SRCS
		FlowAngle.cpp
		flow_angle_params.c
	DEPENDS
		px4_work_queue
	# EXTERNAL marks this as an out-of-tree module so it is built via
	# EXTERNAL_MODULES_LOCATION and never conflicts with the in-tree config.
	EXTERNAL
)
