px4_add_module(
	MODULE modules__flow_angle
	MAIN flow_angle
	SRCS
		FlowAngle.cpp
	# No DEPENDS: thread-based ModuleBase needs no src/lib target.
	# Params are deferred to milestone 2 (module.yaml): in-tree *_params.c relies on
	# a build-provided force-include for PARAM_DEFINE_* that isn't applied to
	# external-module sources, so milestone 1 runs on compiled-in defaults instead.
	EXTERNAL
)
