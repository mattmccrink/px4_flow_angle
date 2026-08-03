# Registers out-of-tree uORB message definitions.
# The generated headers land in <build_dir>/uORB/topics/ exactly like in-tree msgs.
# NOTE: the message name must not collide with any in-tree PX4 message.
set(config_msg_list_external
	SensorFlowAngle.msg
	PARENT_SCOPE
)
