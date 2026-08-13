#include "FlowAngle.hpp"

#include <px4_platform_common/getopt.h>
#include <stdlib.h>
#include <px4_platform_common/module.h>
#include <drivers/drv_sensor.h>

// Any unused DRV_*_DEVTYPE works here; it only seeds the device_id. Reusing the
// MS4525DO diff-pressure devtype keeps the pitot's differential_pressure looking
// like the sensor family it is. Swap for a dedicated devtype if you prefer.
#ifndef DRV_DIFF_PRESS_DEVTYPE_MS4525DO
#define DRV_DIFF_PRESS_DEVTYPE_MS4525DO 0x39
#endif

// PCA9545A default address (A1=A0=GND on the schematic) and bus speed.
static constexpr uint8_t  MUX_ADDRESS_DEFAULT = 0x70;
static constexpr uint32_t I2C_SPEED           = 400 * 1000; // 400 kHz

void FlowAngle::print_usage()
{
	PRINT_MODULE_DESCRIPTION(
		"flow_angle driver v" FLOW_ANGLE_VERSION "\n"
		R"DESCR_STR(
### Description
Five-hole probe flow-angle driver: one PCA9545A I2C switch + three MS4515DO
sensors (alpha/beta + airspeed) behind it. Publishes differential_pressure for
the pitot channel and sensor_flow_angle for the reduction. FA_SIM_EN=1 runs a
hardware-less synthetic path for SITL.

### Examples
Start on the bus the PCA9545A enumerated on (Pixhawk 4: mux seen at 0x70 on I2C4):
$ flow_angle start -b 4 -a 0x70 -f 400
(-X selects the board's external bus instead of a fixed bus number, if preferred)
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("flow_angle", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("airspeed_sensor");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(MUX_ADDRESS_DEFAULT);
	PRINT_MODULE_USAGE_COMMAND_DESCR("scan",
					 "Sweep channels x {0x28,0x36,0x46,0x48}, printing full frame bytes. "
					 "'scan N' streams N frames/channel so you can watch counts under applied pressure.");
	PRINT_MODULE_USAGE_COMMAND_DESCR("null",
					 "Capture per-channel zero offset from no-flow samples (probe capped). 'null N' averages N samples.");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" __EXPORT int flow_angle_main(int argc, char *argv[])
{
	using ThisDriver = FlowAngle;
	BusCLIArguments cli{true, false};          // I2C only
	cli.i2c_address = MUX_ADDRESS_DEFAULT;
	cli.default_i2c_frequency = I2C_SPEED;

	const char *verb = cli.parseDefaultArguments(argc, argv);

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_DIFF_PRESS_DEVTYPE_MS4525DO);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);

	} else if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);

	} else if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);

	} else if (!strcmp(verb, "scan")) {
		// `flow_angle scan [N]`: optional N streams N frames/channel (watch counts).
		if (argc >= 3) { int n = atoi(argv[argc - 1]); if (n > 0) { cli.custom1 = n; } }

		// run on the COMMAND thread (false) so output reaches this console; the
		// driver pauses its sample loop internally for the duration.
		return ThisDriver::module_custom_method(cli, iterator, false);

	} else if (!strcmp(verb, "null")) {
		// `flow_angle null [N]`: capture per-channel zero from N no-flow samples.
		if (argc >= 3) { int n = atoi(argv[argc - 1]); if (n > 0) { cli.custom1 = n; } }

		cli.custom2 = 1;   // select the null sub-command
		return ThisDriver::module_custom_method(cli, iterator, false);
	}

	ThisDriver::print_usage();
	return -1;
}
