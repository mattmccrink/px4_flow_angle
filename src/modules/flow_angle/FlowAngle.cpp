#include "FlowAngle.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <errno.h>
#include <math.h>

void FlowAngle::load_parameters()
{
	param_t h;

	h = param_find("FA_SIM_EN");
	if (h != PARAM_INVALID) { param_get(h, &_sim_en); }

	h = param_find("FA_RATE");
	if (h != PARAM_INVALID) { param_get(h, &_rate_hz); }

	h = param_find("FA_Q_MIN");
	if (h != PARAM_INVALID) { param_get(h, &_q_min); }

	if (_rate_hz < 1.f) { _rate_hz = 50.f; }
}

int FlowAngle::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("flow_angle",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      2048,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

FlowAngle *FlowAngle::instantiate(int argc, char *argv[])
{
	FlowAngle *instance = new FlowAngle();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

int FlowAngle::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

void FlowAngle::run()
{
	load_parameters();

	const uint64_t interval_us = (uint64_t)(1e6f / _rate_hz);

	while (!should_exit()) {
		sensor_flow_angle_s out{};
		out.timestamp_sample = hrt_absolute_time();

		if (_sim_en != 0) {
			// synthetic sweep: alpha ~ +/-6 deg, beta ~ +/-3 deg at a nominal 20 m/s
			_sim_phase += 0.02f;
			out.true_airspeed_m_s   = 20.f;
			out.dynamic_pressure_pa = 245.f;                 // ~ 0.5 * 1.225 * 20^2
			out.alpha_rad = 0.10f * sinf(_sim_phase);
			out.beta_rad  = 0.05f * sinf(0.5f * _sim_phase);
			out.dp_alpha_pa = out.dynamic_pressure_pa * out.alpha_rad;   // placeholder
			out.dp_beta_pa  = out.dynamic_pressure_pa * out.beta_rad;
			out.device_id = 0;
			out.valid = out.dynamic_pressure_pa > _q_min;

		} else {
			// hardware path (mux select -> 3x MS4525DO -> 5-hole reduction) is milestone 2
			out.valid = false;
		}

		out.timestamp = hrt_absolute_time();

		if (_flow_angle_pub == nullptr) {
			_flow_angle_pub = orb_advertise(ORB_ID(sensor_flow_angle), &out);

		} else {
			orb_publish(ORB_ID(sensor_flow_angle), _flow_angle_pub, &out);
		}

		px4_usleep(interval_us);
	}

	if (_flow_angle_pub != nullptr) {
		orb_unadvertise(_flow_angle_pub);
	}
}

int FlowAngle::print_status()
{
	PX4_INFO("sim mode: %s, rate: %.1f Hz", _sim_en != 0 ? "ON" : "OFF", (double)_rate_hz);
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

Milestone-1 scaffold: runs as a thread-based module in simulation mode
(FA_SIM_EN=1) and publishes synthetic data to the sensor_flow_angle topic, so the
out-of-tree build, custom uORB message, logging and telemetry can be validated in
SITL with no hardware attached.
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
