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
