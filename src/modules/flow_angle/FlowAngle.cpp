#include "FlowAngle.hpp"

#include <math.h>

FlowAngle::FlowAngle() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

FlowAngle::~FlowAngle()
{
	perf_free(_loop_perf);
}

bool FlowAngle::init()
{
	float rate = _param_rate_hz.get();

	if (rate < 1.f) {
		rate = 50.f;
	}

	ScheduleOnInterval((uint32_t)(1e6f / rate));   // ScheduleOnInterval takes microseconds
	return true;
}

void FlowAngle::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	sensor_flow_angle_s out{};
	out.timestamp_sample = hrt_absolute_time();

	if (_param_sim_en.get() != 0) {
		// Synthetic sweep so the topic has something recognisable in the log:
		// alpha ~ +/-6 deg, beta ~ +/-3 deg, at a nominal 20 m/s.
		_sim_phase += 0.02f;
		out.true_airspeed_m_s = 20.f;
		out.dynamic_pressure_pa = 245.f;          // ~ 0.5 * 1.225 * 20^2
		out.alpha_rad = 0.10f * sinf(_sim_phase);
		out.beta_rad  = 0.05f * sinf(0.5f * _sim_phase);
		out.dp_alpha_pa = out.dynamic_pressure_pa * out.alpha_rad;   // placeholder mapping
		out.dp_beta_pa  = out.dynamic_pressure_pa * out.beta_rad;
		out.device_id = 0;
		out.valid = out.dynamic_pressure_pa > _param_q_min.get();

	} else {
		// Hardware path (mux select -> read 3x MS4525DO -> 5-hole reduction)
		// is added in milestone 2. Until then, report invalid.
		out.valid = false;
	}

	out.timestamp = hrt_absolute_time();
	_sensor_flow_angle_pub.publish(out);

	perf_end(_loop_perf);
}

int FlowAngle::task_spawn(int argc, char *argv[])
{
	FlowAngle *instance = new FlowAngle();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	if (!instance->init()) {
		PX4_ERR("init failed");
		delete instance;
		_object.store(nullptr);
		_task_id = -1;
		return PX4_ERROR;
	}

	return PX4_OK;
}

int FlowAngle::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FlowAngle::print_status()
{
	perf_print_counter(_loop_perf);
	PX4_INFO("sim mode: %s", _param_sim_en.get() != 0 ? "ON" : "OFF");
	return 0;
}

int FlowAngle::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Five-hole probe flow-angle driver (alpha/beta) for three MS4525DO sensors behind
a TCA9548A I2C mux.

This milestone-1 scaffold runs in simulation mode (FA_SIM_EN=1) and publishes
synthetic data to the sensor_flow_angle topic, so the out-of-tree build, the
custom uORB message, logging and MAVLink telemetry can be validated in SITL with
no hardware attached.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("flow_angle", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int flow_angle_main(int argc, char *argv[])
{
	return FlowAngle::main(argc, argv);
}
