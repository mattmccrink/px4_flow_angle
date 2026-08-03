#pragma once

#define MODULE_NAME "flow_angle"

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/sensor_flow_angle.h>

using namespace time_literals;

/**
 * Milestone 1 scaffold.
 *
 * A plain work-queue module (no I2C yet) whose only job is to publish the
 * sensor_flow_angle topic so the out-of-tree build, uORB message, logging and
 * telemetry path can be validated end-to-end in SITL.
 *
 * Milestone 2 swaps the base class to I2CSPIDriver, owns the TCA9548A + three
 * MS4525DO, and moves the synthetic branch behind FA_SIM_EN so it survives as a
 * hardware-less regression path.
 */
class FlowAngle : public ModuleBase<FlowAngle>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	FlowAngle();
	~FlowAngle() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	void Run() override;

	uORB::Publication<sensor_flow_angle_s> _sensor_flow_angle_pub{ORB_ID(sensor_flow_angle)};

	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

	float _sim_phase{0.f};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::FA_SIM_EN>)  _param_sim_en,
		(ParamFloat<px4::params::FA_RATE>)  _param_rate_hz,
		(ParamFloat<px4::params::FA_Q_MIN>) _param_q_min
	)
};
