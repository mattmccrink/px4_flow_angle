#pragma once

#define MODULE_NAME "flow_angle"

// Bump this on every released tarball. Printed on start, in `flow_angle status`,
// and in the usage text; grep-able in source to confirm which tree is in play.
#define FLOW_ANGLE_VERSION "0.5.0"

// Human-readable channel config on the SD card (see README for the format).
#define FA_CONFIG_PATH    "/fs/microsd/etc/flow_angle/config.txt"
#define FA_CONFIG_VERSION 1

#include <drivers/drv_hrt.h>
#include <lib/drivers/device/i2c.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/atomic.h>
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

	// Post-MR conversion wait and MR style are runtime params now (FA_CONV_US,
	// FA_MR_MODE) -- a low-power part can need a longer wake and/or an address-only
	// measurement request. See _conv_us / _mr_mode below.

	enum class Role : uint8_t { ALPHA = 0, PITOT = 1, BETA = 2 };

	enum class Phase : uint8_t { MEASURE, READ };

	enum class Status : uint8_t {   // top 2 bits of the first frame byte
		Normal   = 0b00,
		Reserved = 0b01,
		Stale    = 0b10,
		Fault    = 0b11,
	};

	enum class FrameResult : uint8_t {  // outcome of one decode+validate
		Ok = 0,
		Comms,      // I2C transfer failed
		Fault,      // status bits = Fault
		Stale,      // status bits not Normal (Stale/Reserved) -> reject, do not trust
		OutOfRange, // decoded pressure outside the sensor's physical range
	};

	// On a rejected frame, re-issue MR->convert->DF up to this many extra times
	// before giving up on the channel this cycle. Targets low-power parts that can
	// hand back a stale/max register on a first fetch.
	static constexpr int MAX_REREADS = 2;

	struct ChannelCfg {
		uint8_t mux_bit;    // PCA9545A control byte: 1 << channel (0x01/0x02/0x04)
		uint8_t addr;       // sensor 7-bit I2C address behind the mux
		float   p_min_pa;   // pressure at the low output code
		float   p_max_pa;   // pressure at the high output code
		float   out_offset; // output-type offset fraction: A=0.10, B=0.05
		float   out_span;   // output-type span fraction:   A=0.80, B=0.90
		Role    role;
		bool    verified;   // passed the boot-time temperature-sanity gate
	};

	struct ChannelSample {
		float press_pa{0.f};
		float temp_c{0.f};
		bool  ok{false};
	};

	void  load_parameters();
	void  run_sim();                                   // FA_SIM_EN synthetic path
	int   mux_select(uint8_t mux_bit);                 // write control byte to PCA9545A
	int   send_mr(uint8_t addr);                       // issue a measurement request (mode-dependent)
	FrameResult read_frame(const ChannelCfg &c, ChannelSample &out, uint8_t raw[4]); // decode+validate one DF4 frame
	void  read_channel(int idx);                       // full read w/ stale-reject + bounded re-read
	void  log_raw(int idx, const uint8_t raw[4], FrameResult r, int tries); // rate-limited raw-byte dump
	static const char *result_str(FrameResult r);      // frame-result name for logs
	static const char *role_str(Role r);
	bool  load_config_file();                          // read/parse the SD config; false -> defaults
	void  verify_channels();                           // boot-time presence + temperature-sanity gate
	void  update_health();                             // per-cycle health monitor -> QGC alerts
	void  enforce_dpres_off();                          // clear (or warn on) stock SENS_DPRES_OFF
	void  do_scan(int stream);                         // `scan` sub-command (command thread)
	void  do_null(int n);                              // `null` sub-command: capture per-channel zero
	float apply_offset(Role r, float raw_pa, float temp_c) const; // subtract the channel zero
	float transfer_fn(int16_t bridge, const ChannelCfg &c) const;
	void  publish_cycle();
	void  schedule_next_cycle();
	const ChannelSample *sample_for(Role r) const;

	uint8_t _mux_addr{0x70};

	// type-B defaults (5..95%); a config file or FA_OUT_TYP may change per channel.
	ChannelCfg    _cfg[N_CH] {
		{0x01, 0x46, -4.f  * INH2O_TO_PA, +4.f  * INH2O_TO_PA, 0.05f, 0.9f, Role::ALPHA, false},
		{0x02, 0x46, -20.f * INH2O_TO_PA, +20.f * INH2O_TO_PA, 0.05f, 0.9f, Role::PITOT, false},
		{0x04, 0x46, -4.f  * INH2O_TO_PA, +4.f  * INH2O_TO_PA, 0.05f, 0.9f, Role::BETA,  false},
	};
	ChannelSample _samp[N_CH];

	bool        _bus_ready{false};   // I2C::init() succeeded (false in sim mode)
	px4::atomic<bool> _pause{false}; // set by scan (command thread) to park RunImpl
	uint8_t     _last_raw[N_CH][4] {};   // last raw 4-byte frame per channel (for status)
	FrameResult _last_result[N_CH] {FrameResult::Comms, FrameResult::Comms, FrameResult::Comms};
	Phase       _phase{Phase::MEASURE};
	uint8_t     _ch_idx{0};
	hrt_abstime _cycle_start{0};
	hrt_abstime _timestamp_sample{0};

	// (output-type fractions are per-channel in ChannelCfg now)

	// tunables (params)
	int32_t _sim_en{0};   // default HW; FA_SIM_EN=1 (param) re-enables the synthetic path
	int32_t _dbg_raw{0};  // FA_DBG_RAW=1 -> dump raw 4-byte frames (rate-limited)
	int32_t _cfg_sd{1};   // FA_CFG_SD=1 -> load channel config from the SD card at boot
	int32_t _zero_dpres{1};// FA_ZERO_DPRES=1 -> force SENS_DPRES_OFF=0 at start
	uint32_t _conv_us{5000};  // FA_CONV_US: post-MR conversion/wake wait [us]
	int32_t _mr_mode{0};      // FA_MR_MODE: 0 = 1-byte 0x00 write, 1 = address-only write
	uint8_t _last_tries[N_CH] {};   // re-reads needed on the last cycle, per channel
	hrt_abstime _last_dbg{0};       // raw-log rate limiter

	// mid-flight health monitor -> QGC alerts
	orb_advert_t _mavlink_log_pub{nullptr};
	uint32_t    _fail_count[N_CH] {};   // consecutive failed cycles, per channel
	bool        _ch_failed[N_CH] {};    // latched "failed" state (debounced)
	hrt_abstime _last_alert[N_CH] {};   // last QGC alert time, per channel
	int32_t     _fail_ms{1000};         // FA_FAIL_MS: dropout time before alerting
	float   _rate_hz{50.f};
	float   _q_min{20.f};
	float   _rho{1.225f};
	float   _cal_a{1.f};   // placeholder alpha gain; real map is milestone 3
	float   _cal_b{1.f};   // placeholder beta gain
	float   _off[3] {0.f, 0.f, 0.f};   // per-role zero offset [alpha, airspeed, beta], Pa (FA_OFF_*)

	float   _sim_phase{0.f};

	uORB::Publication<sensor_flow_angle_s>          _flow_angle_pub{ORB_ID(sensor_flow_angle)};
	uORB::PublicationMulti<differential_pressure_s> _diff_press_pub{ORB_ID(differential_pressure)};
	uORB::Publication<debug_array_s>                _debug_array_pub{ORB_ID(debug_array)};

	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": comms errors")};
	perf_counter_t _fault_perf{perf_alloc(PC_COUNT, MODULE_NAME": sensor faults")};
	perf_counter_t _reject_perf{perf_alloc(PC_COUNT, MODULE_NAME": frames rejected")};
	perf_counter_t _reread_perf{perf_alloc(PC_COUNT, MODULE_NAME": cycles needing re-read")};
	perf_counter_t _stale_perf{perf_alloc(PC_COUNT, MODULE_NAME": channels dropped (stale)")};
};
