#include "FlowAngle.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <parameters/param.h>
#include <drivers/drv_sensor.h>
#include <stdlib.h>

// Any unused DRV_*_DEVTYPE works here; it only seeds the device_id.
#ifndef DRV_DIFF_PRESS_DEVTYPE_MS4525DO
#define DRV_DIFF_PRESS_DEVTYPE_MS4525DO 0x39
#endif

static constexpr uint32_t I2C_SPEED = 400 * 1000;   // 400 kHz

void FlowAngle::print_usage()
{
	PRINT_MODULE_DESCRIPTION(
		"flow_angle driver v" FLOW_ANGLE_VERSION "\n"
		R"DESCR_STR(
### Description
Five-hole probe flow-angle driver. It reads three MS45x differential pressure
sensors behind a PCA9545A I2C switch on an external I2C bus. It publishes
differential_pressure for the pitot channel and sensor_flow_angle for the
alpha/beta reduction.

The start command takes no bus arguments. The I2C bus, the switch address, and
each sensor's address, range and type come from parameters and the SD config
file (/fs/microsd/etc/flow_angle/config.txt). Set FA_BUS to change the bus
(default 4). Set FA_SIM_EN=1 for a hardware-less synthetic path.

### Examples
Start the driver:
$ flow_angle start
Read the raw frames, with temperature, from each channel:
$ flow_angle scan
Capture the zero offsets with the probe capped:
$ flow_angle null
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("flow_angle", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("airspeed_sensor");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("scan",
					 "Read all mux channels. Print raw bytes, status, counts and temperature. "
					 "'scan N' streams N frames per channel to show counts under pressure.");
	PRINT_MODULE_USAGE_COMMAND_DESCR("null",
					 "Capture the per-channel zero offset from no-flow samples. Cap the probe first. "
					 "'null N' averages N samples.");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" __EXPORT int flow_angle_main(int argc, char *argv[])
{
	using ThisDriver = FlowAngle;

	// Bus and switch address come from parameters, not CLI flags. This is always
	// an external I2C sensor.
	int32_t fa_bus = 4;
	int32_t fa_mux = 0x70;
	param_t h = param_find("FA_BUS");       if (h != PARAM_INVALID) { param_get(h, &fa_bus); }
	h = param_find("FA_MUX_ADDR");          if (h != PARAM_INVALID) { param_get(h, &fa_mux); }

	BusCLIArguments cli{true, false};       // I2C only
	cli.i2c_address = (uint8_t)fa_mux;
	cli.default_i2c_frequency = I2C_SPEED;

	const char *verb = cli.parseDefaultArguments(argc, argv);

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	// Force the external bus from FA_BUS, ignoring any leftover flags.
	cli.bus_option = I2CSPIBusOption::I2CExternal;
	cli.requested_bus = fa_bus;
	cli.i2c_address = (uint8_t)fa_mux;

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_DIFF_PRESS_DEVTYPE_MS4525DO);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);

	} else if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);

	} else if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);

	} else if (!strcmp(verb, "scan")) {
		if (argc >= 3) { int n = atoi(argv[argc - 1]); if (n > 0) { cli.custom1 = n; } }

		return ThisDriver::module_custom_method(cli, iterator, false);

	} else if (!strcmp(verb, "null")) {
		if (argc >= 3) { int n = atoi(argv[argc - 1]); if (n > 0) { cli.custom1 = n; } }

		cli.custom2 = 1;
		return ThisDriver::module_custom_method(cli, iterator, false);
	}

	ThisDriver::print_usage();
	return -1;
}
