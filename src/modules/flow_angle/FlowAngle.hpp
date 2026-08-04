#pragma once

#define MODULE_NAME "flow_angle"

#include <px4_platform_common/module.h>
#include <drivers/drv_hrt.h>
#include <parameters/param.h>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_flow_angle.h>
#include <uORB/topics/debug_array.h>

/**
 * Milestone 1 scaffold (thread-based).
 *
 * A plain ModuleBase task whose only job is to publish sensor_flow_angle so the
 * out-of-tree build, custom uORB message, logging and telemetry path can be
 * validated in SITL with no hardware. Deliberately has zero src/lib dependencies
 * so it configures cleanly ahead of src/lib in the external-module build order.
 *
 * Milestone 2 swaps this for an I2CSPIDriver that owns the TCA9548A mux + three
 * MS4525DO, moving the synthetic branch behind FA_SIM_EN as a regression path.
 */
class FlowAngle : public ModuleBase<FlowAngle>
{
public:
	FlowAngle() = default;
	~FlowAngle() override = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);
	static FlowAngle *instantiate(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	void run() override;
	int print_status() override;

private:
	void load_parameters();

	orb_advert_t _flow_angle_pub{nullptr};
	orb_advert_t _debug_array_pub{nullptr};   // mirror to QGC via DEBUG_FLOAT_ARRAY

	int32_t _sim_en{1};
	float   _rate_hz{50.f};
	float   _q_min{20.f};

	float   _sim_phase{0.f};
};
