#pragma once

#define MODULE_NAME "flow_angle"

// Bump this on every released tarball. Printed on start, in `flow_angle status`,
// and in the usage text; grep-able in source to confirm which tree is in play.
#define FLOW_ANGLE_VERSION "0.2.1"

#include <drivers/drv_hrt.h>
#include <lib/drivers/device/i2c.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <parameters/param.h>

#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/sensor_flow_angle.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/debug_array.h>

/*
 * Milestone-2 hardware driver for a 5-hole probe.
 *
 * One driver owns a PCA9545A I2C switch (default addr 0x70) and three MS4515DO
 * digital pressure sensors sitting behind it, one per switch channel:
 *
 *   ch0 -> alpha    MS4515DO, I2C 0x46, +/-4 inH2O  differential, output type B
 *   ch1 -> airspeed MS4515DO, I2C 0x46, +/-20 inH2O differential, output type B
 *   ch2 -> beta     MS4515DO, I2C 0x46, +/-4 inH2O  differential, output type B
 *
 * All three parts are marked "3BK" = 3.3V / type B / interface K (I2C 0x46), so
 * every channel answers at the SAME address 0x46 -- the mux is what makes three
 * identical-address parts individually addressable. (Confirmed off the physical
 * package markings, which override the Digikey ordering strings.)
 *
 * Per cycle, for each channel: write the channel-select byte to the PCA9545A,
 * issue a Read-MR (measurement request) to the sensor, wait for the conversion,
 * then Read-DF4 (4-byte data fetch). The MR/DF handshake is what the Low-Power
 * airspeed part needs; the free-running alpha/beta parts tolerate it too, so all
 * three channels use the same path.
 *
 * Publications:
 *   - differential_pressure  (pitot/airspeed channel only) -> stock airspeed selector -> EKF2
 *   - sensor_flow_angle       (alpha/beta + q + TAS + raw diffs)
 *   - debug_array "flow"      (alpha deg, beta deg, TAS) -> QGC DEBUG_FLOAT_ARRAY (live view)
 *
 * FA_SIM_EN=1 keeps the milestone-1 synthetic path as a hardware-less SITL
 * regression: no I2C traffic, just synthesized sensor_flow_angle at FA_RATE.
 *
 * The channel map, sensor addresses, ranges, output type and reduction gains are
 * all params (module.yaml), so a wiring change or a re-cal is a param reload, not
 * a recompile. The angle reduction here is a linear placeholder; milestone 3
 * replaces it with the Calspan tunnel map.
 */
class FlowAngle : public device::I2C, public I2CSPIDriver<FlowAngle>
{
public:
	FlowAngle(const I2CSPIDriverConfig &config);
	~FlowAngle() override;

	static void print_usage();

	int  init() override;
	void print_status() override;

	void RunImpl();

private:
	int probe() override;          // confirm the PCA9545A answers at its address
	void custom_method(const BusCLIArguments &cli) override; // 'scan' verb (runs on the wq)

	static constexpr int N_CH = 3; // channels actually populated behind the mux

	// MS4515DO Read-MR command (begin measurement); Read-DF is a plain read.
	static constexpr uint8_t ADDR_READ_MR = 0x00;

	// inches-H2O -> Pa (1 inH2O = 248.84 Pa)
	static constexpr float INH2O_TO_PA = 248.84f;

	// 14-bit full scale
	static constexpr float FULL_SCALE = 16383.f;

	// Conversion wait between Read-MR and Read-DF. Reference ms4525do uses 2 ms;
	// nudged up for the Low-Power airspeed part's wake+convert. Tune on the bench.
	static constexpr hrt_abstime CONVERSION_INTERVAL{2500};

	enum class Role : uint8_t { ALPHA = 0, PITOT = 1, BETA = 2 };

	enum class Phase : uint8_t { MEASURE, READ };

	enum class Status : uint8_t {   // top 2 bits of the first frame byte
		Normal   = 0b00,
		Reserved = 0b01,
		Stale    = 0b10,
		Fault    = 0b11,
	};

	struct ChannelCfg {
		uint8_t mux_bit;   // PCA9545A control byte: 1 << channel (0x01/0x02/0x04)
		uint8_t addr;      // sensor 7-bit I2C address behind the mux
		float   p_min_pa;  // pressure at 5% output (type B)
		float   p_max_pa;  // pressure at 95% output (type B)
		Role    role;
	};

	struct ChannelSample {
		float press_pa{0.f};
		float temp_c{0.f};
		bool  ok{false};
	};

	void  load_parameters();
	void  run_sim();                                   // FA_SIM_EN synthetic path
	int   mux_select(uint8_t mux_bit);                 // write control byte to PCA9545A
	bool  read_frame(const ChannelCfg &c, ChannelSample &out); // decode one DF4 frame
	float transfer_fn(int16_t bridge, float p_min_pa, float p_max_pa) const;
	void  publish_cycle();
	void  schedule_next_cycle();
	const ChannelSample *sample_for(Role r) const;

	uint8_t _mux_addr{0x70};

	ChannelCfg    _cfg[N_CH] {
		{0x01, 0x46, -4.f  * INH2O_TO_PA, +4.f  * INH2O_TO_PA, Role::ALPHA},
		{0x02, 0x46, -20.f * INH2O_TO_PA, +20.f * INH2O_TO_PA, Role::PITOT},
		{0x04, 0x46, -4.f  * INH2O_TO_PA, +4.f  * INH2O_TO_PA, Role::BETA},
	};
	ChannelSample _samp[N_CH];

	bool        _bus_ready{false};   // I2C::init() succeeded (false in sim mode)
	Phase       _phase{Phase::MEASURE};
	uint8_t     _ch_idx{0};
	hrt_abstime _cycle_start{0};
	hrt_abstime _timestamp_sample{0};

	// output-type offset/span fractions: A(10-90%) = 0.1/0.8, B(5-95%) = 0.05/0.9
	float _out_offset{0.05f};
	float _out_span{0.9f};

	// tunables (params)
	int32_t _sim_en{1};
	float   _rate_hz{50.f};
	float   _q_min{20.f};
	float   _rho{1.225f};
	float   _cal_a{1.f};   // placeholder alpha gain; real map is milestone 3
	float   _cal_b{1.f};   // placeholder beta gain

	float   _sim_phase{0.f};

	uORB::Publication<sensor_flow_angle_s>          _flow_angle_pub{ORB_ID(sensor_flow_angle)};
	uORB::PublicationMulti<differential_pressure_s> _diff_press_pub{ORB_ID(differential_pressure)};
	uORB::Publication<debug_array_s>                _debug_array_pub{ORB_ID(debug_array)};

	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": comms errors")};
	perf_counter_t _fault_perf{perf_alloc(PC_COUNT, MODULE_NAME": sensor faults")};
};
