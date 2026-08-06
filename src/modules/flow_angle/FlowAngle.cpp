#include "FlowAngle.hpp"

#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <math.h>
#include <string.h>

using namespace time_literals;

static constexpr float RAD2DEG = 57.29578f;

FlowAngle::FlowAngle(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config)
{
	// The driver is registered at the PCA9545A address; remember it so we can
	// flip back to it for channel selects between sensor reads.
	_mux_addr = get_device_address();
}

FlowAngle::~FlowAngle()
{
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_fault_perf);
}

void FlowAngle::load_parameters()
{
	auto get_i = [](const char *name, int32_t &v) {
		param_t h = param_find(name);
		if (h != PARAM_INVALID) { param_get(h, &v); }
	};
	auto get_f = [](const char *name, float &v) {
		param_t h = param_find(name);
		if (h != PARAM_INVALID) { param_get(h, &v); }
	};

	get_i("FA_SIM_EN", _sim_en);
	get_f("FA_RATE",   _rate_hz);
	get_f("FA_Q_MIN",  _q_min);
	get_f("FA_RHO",    _rho);
	get_f("FA_CAL_A",  _cal_a);
	get_f("FA_CAL_B",  _cal_b);

	if (_rate_hz < 1.f)  { _rate_hz = 50.f; }
	if (_rho     < 0.1f) { _rho = 1.225f; }

	// output type: 0 = A (10-90%), 1 = B (5-95%). Our parts are B.
	int32_t out_typ = 1;
	get_i("FA_OUT_TYP", out_typ);

	if (out_typ == 0) { _out_offset = 0.1f;  _out_span = 0.8f; }
	else              { _out_offset = 0.05f; _out_span = 0.9f; }

	// per-channel map / addresses / ranges (defaults match the populated PCB)
	struct { const char *ch; const char *ad; const char *rng; Role role; int idx; } p[] = {
		{"FA_A_CH",  "FA_A_ADDR",  "FA_A_RNG",  Role::ALPHA, 0},
		{"FA_AS_CH", "FA_AS_ADDR", "FA_AS_RNG", Role::PITOT, 1},
		{"FA_B_CH",  "FA_B_ADDR",  "FA_B_RNG",  Role::BETA,  2},
	};

	for (auto &e : p) {
		int32_t ch = __builtin_ctz(_cfg[e.idx].mux_bit); // current bit -> channel index
		int32_t ad = _cfg[e.idx].addr;
		float   rng = _cfg[e.idx].p_max_pa / INH2O_TO_PA; // current +FS in inH2O

		get_i(e.ch, ch);
		get_i(e.ad, ad);
		get_f(e.rng, rng);

		if (ch < 0) { ch = 0; }
		if (ch > 3) { ch = 3; }

		_cfg[e.idx].mux_bit  = (uint8_t)(1u << ch);
		_cfg[e.idx].addr     = (uint8_t)ad;
		_cfg[e.idx].p_max_pa = +rng * INH2O_TO_PA; // differential parts are bidirectional
		_cfg[e.idx].p_min_pa = -rng * INH2O_TO_PA; // (gage part -> set p_min_pa = 0)
		_cfg[e.idx].role     = e.role;
	}
}

int FlowAngle::init()
{
	load_parameters(); // read FA_SIM_EN (and the rest) before deciding to touch the bus

	if (_sim_en != 0) {
		// Hardware-less regression path: never open the bus or probe the mux, just
		// synthesize. Runs even on an FC with nothing wired to this bus.
		PX4_INFO("FA_SIM_EN=1: synthetic mode (no I2C bus init)");
		ScheduleNow();
		return PX4_OK;
	}

	int ret = I2C::init(); // opens the bus and runs probe() against the PCA9545A

	if (ret != PX4_OK) {
		DEVICE_DEBUG("I2C::init failed (%i)", ret);
		return ret;
	}

	_bus_ready = true;

	// bench aid: report which configured channels answer at their address.
	for (int i = 0; i < N_CH; i++) {
		const ChannelCfg &c = _cfg[i];
		bool ok = false;

		if (mux_select(c.mux_bit) == PX4_OK) {
			set_device_address(c.addr);
			uint8_t mr = ADDR_READ_MR;
			ok = (transfer(&mr, 1, nullptr, 0) == PX4_OK);
		}

		PX4_INFO("ch%u (0x%02x) role=%u addr=0x%02x range=+/-%.0f Pa : %s",
			 (unsigned)__builtin_ctz(c.mux_bit), (unsigned)c.mux_bit, (unsigned)c.role,
			 (unsigned)c.addr, (double)c.p_max_pa, ok ? "ACK" : "no reply");
	}

	set_device_address(_mux_addr);
	mux_select(0x00); // leave all channels deselected until the first cycle

	ScheduleNow();
	return PX4_OK;
}

int FlowAngle::probe()
{
	_retries = 1;

	// The addressable device on the bus is the PCA9545A. Deselecting all
	// channels (control byte 0x00) is a harmless write that must ACK.
	set_device_address(_mux_addr);
	uint8_t clear = 0x00;

	for (int i = 0; i < 5; i++) {
		if (transfer(&clear, 1, nullptr, 0) == PX4_OK) {
			return PX4_OK;
		}

		px4_usleep(1000);
	}

	return PX4_ERROR;
}

int FlowAngle::mux_select(uint8_t mux_bit)
{
	set_device_address(_mux_addr);
	return transfer(&mux_bit, 1, nullptr, 0);
}

float FlowAngle::transfer_fn(int16_t bridge, float p_min_pa, float p_max_pa) const
{
	// Inversion of the MS4515DO pressure transfer function.
	//   type A: 10%..90% -> offset 0.1, span 0.8
	//   type B:  5%..95% -> offset 0.05, span 0.9  (our parts)
	return (bridge - _out_offset * FULL_SCALE) * (p_max_pa - p_min_pa)
	       / (_out_span * FULL_SCALE) + p_min_pa;
}

bool FlowAngle::read_frame(const ChannelCfg &c, ChannelSample &out)
{
	// mux already selected for this channel; retarget to the sensor and fetch 4 bytes.
	set_device_address(c.addr);

	uint8_t d[4] {};

	if (transfer(nullptr, 0, d, sizeof(d)) != PX4_OK) {
		perf_count(_comms_errors);
		out.ok = false;
		return false;
	}

	const uint8_t status = (d[0] & 0b1100'0000) >> 6;

	if (status == (uint8_t)Status::Fault) {
		perf_count(_fault_perf);
		out.ok = false;
		return false;
	}

	// accept Normal (fresh) and Stale (valid but re-fetched) frames
	const int16_t bridge = ((d[0] & 0b0011'1111) << 8) | d[1];
	const int16_t temp11 = ((d[2] << 8) + (0b1110'0000 & d[3])) / (1 << 5);

	out.press_pa = transfer_fn(bridge, c.p_min_pa, c.p_max_pa);
	out.temp_c   = (200.f * temp11) / 2047.f - 50.f;
	out.ok       = true;
	return true;
}

const FlowAngle::ChannelSample *FlowAngle::sample_for(Role r) const
{
	for (int i = 0; i < N_CH; i++) {
		if (_cfg[i].role == r) { return &_samp[i]; }
	}

	return nullptr;
}

void FlowAngle::schedule_next_cycle()
{
	const hrt_abstime now = hrt_absolute_time();
	const uint32_t period_us = (uint32_t)(1e6f / _rate_hz);
	const hrt_abstime next = _cycle_start + period_us;

	int32_t delay = (int32_t)(next - now);

	if (delay < 1000) { delay = 1000; }

	ScheduleDelayed(delay);
}

void FlowAngle::RunImpl()
{
	if (_sim_en != 0) {
		run_sim();
		return;
	}

	const ChannelCfg &c = _cfg[_ch_idx];

	switch (_phase) {
	case Phase::MEASURE: {
			if (_ch_idx == 0) {
				_cycle_start = hrt_absolute_time();
				perf_begin(_sample_perf);
			}

			if (mux_select(c.mux_bit) != PX4_OK) {
				perf_count(_comms_errors);
				_samp[_ch_idx].ok = false;
				// skip this channel's read this cycle
				_phase = Phase::MEASURE;

				if (_ch_idx + 1 < N_CH) { _ch_idx++; ScheduleNow(); }
				else { perf_end(_sample_perf); _ch_idx = 0; publish_cycle(); schedule_next_cycle(); }

				return;
			}

			set_device_address(c.addr);
			uint8_t mr = ADDR_READ_MR;

			if (transfer(&mr, 1, nullptr, 0) != PX4_OK) {
				perf_count(_comms_errors);
				_samp[_ch_idx].ok = false;

				if (_ch_idx + 1 < N_CH) { _ch_idx++; ScheduleNow(); }
				else { perf_end(_sample_perf); _ch_idx = 0; publish_cycle(); schedule_next_cycle(); }

				return;
			}

			_timestamp_sample = hrt_absolute_time();
			_phase = Phase::READ;
			ScheduleDelayed(CONVERSION_INTERVAL);
			break;
		}

	case Phase::READ: {
			// re-select the channel every transaction (belt-and-suspenders: we own
			// the mux, but this survives a future second mux user or a stale latch)
			mux_select(c.mux_bit);
			read_frame(c, _samp[_ch_idx]);

			_phase = Phase::MEASURE;

			if (_ch_idx + 1 < N_CH) {
				_ch_idx++;
				ScheduleNow();

			} else {
				perf_end(_sample_perf);
				_ch_idx = 0;
				publish_cycle();
				schedule_next_cycle();
			}

			break;
		}
	}
}

void FlowAngle::publish_cycle()
{
	const hrt_abstime now = hrt_absolute_time();

	const ChannelSample *pitot = sample_for(Role::PITOT);
	const ChannelSample *a     = sample_for(Role::ALPHA);
	const ChannelSample *b     = sample_for(Role::BETA);

	// --- pitot -> differential_pressure (feeds stock airspeed selector / EKF2) ---
	if (pitot && pitot->ok) {
		differential_pressure_s dp{};
		dp.timestamp_sample = _timestamp_sample;
		dp.device_id = get_device_id();
		dp.differential_pressure_pa = pitot->press_pa;
		dp.temperature = pitot->temp_c;
		dp.error_count = perf_event_count(_comms_errors);
		dp.timestamp = now;
		_diff_press_pub.publish(dp);
	}

	// --- 5-hole reduction (PLACEHOLDER linear model; milestone 3 = Calspan map) ---
	const float q  = (pitot && pitot->ok) ? fmaxf(pitot->press_pa, 0.f) : 0.f;
	const float dpa = (a && a->ok) ? a->press_pa : 0.f;
	const float dpb = (b && b->ok) ? b->press_pa : 0.f;

	const bool valid = pitot && pitot->ok && a && a->ok && b && b->ok && (q > _q_min);

	float alpha_rad = 0.f;
	float beta_rad  = 0.f;
	float tas       = 0.f;

	if (valid) {
		// normalized-by-q linear stand-in: good enough to see the probe respond,
		// NOT a calibrated angle. Real reduction is coefficients f(pressure ratios).
		alpha_rad = _cal_a * (dpa / q);
		beta_rad  = _cal_b * (dpb / q);
		tas       = sqrtf(2.f * q / _rho); // density-corrected TAS needs real rho; refine later
	}

	sensor_flow_angle_s out{};
	out.timestamp_sample   = _timestamp_sample;
	out.true_airspeed_m_s  = tas;
	out.dynamic_pressure_pa = q;
	out.alpha_rad          = alpha_rad;
	out.beta_rad           = beta_rad;
	out.dp_alpha_pa        = dpa;
	out.dp_beta_pa         = dpb;
	out.device_id          = get_device_id();
	out.valid              = valid;
	out.timestamp          = now;
	_flow_angle_pub.publish(out);

	// --- live QGC mirror (DEBUG_FLOAT_ARRAY) ---
	debug_array_s dbg{};
	dbg.timestamp = now;
	dbg.id = 0;
	strncpy(dbg.name, "flow", sizeof(dbg.name));
	dbg.data[0] = alpha_rad * RAD2DEG;
	dbg.data[1] = beta_rad  * RAD2DEG;
	dbg.data[2] = tas;
	_debug_array_pub.publish(dbg);
}

void FlowAngle::run_sim()
{
	// Milestone-1 synthetic sweep, retained as a hardware-less regression path.
	const hrt_abstime now = hrt_absolute_time();

	_sim_phase += 0.02f;

	const float q     = 245.f;             // ~0.5 * 1.225 * 20^2
	const float alpha = 0.10f * sinf(_sim_phase);
	const float beta  = 0.05f * sinf(0.5f * _sim_phase);

	sensor_flow_angle_s out{};
	out.timestamp_sample    = now;
	out.true_airspeed_m_s   = 20.f;
	out.dynamic_pressure_pa = q;
	out.alpha_rad           = alpha;
	out.beta_rad            = beta;
	out.dp_alpha_pa         = q * alpha;
	out.dp_beta_pa          = q * beta;
	out.device_id           = 0;
	out.valid               = q > _q_min;
	out.timestamp           = now;
	_flow_angle_pub.publish(out);

	debug_array_s dbg{};
	dbg.timestamp = now;
	dbg.id = 0;
	strncpy(dbg.name, "flow", sizeof(dbg.name));
	dbg.data[0] = alpha * RAD2DEG;
	dbg.data[1] = beta  * RAD2DEG;
	dbg.data[2] = 20.f;
	_debug_array_pub.publish(dbg);

	ScheduleDelayed((hrt_abstime)(1e6f / _rate_hz));
}

void FlowAngle::custom_method(const BusCLIArguments &cli)
{
	// Invoked by `flow_angle scan`. module_custom_method schedules this as a
	// single-shot on the driver's own work queue (wq:I2Cx), so it is serialized
	// against RunImpl -- the bus is ours for the duration, no mux-latch race. It
	// does briefly stall sampling (a few ms per responding sensor).
	if (!_bus_ready) {
		PX4_WARN("scan needs the bus initialized -- start with FA_SIM_EN 0 first");
		return;
	}

	// MS4515DO interface-letter addresses: I=0x28, J=0x36, K=0x46, 0=0x48.
	static constexpr uint8_t kAddrs[] = {0x28, 0x36, 0x46, 0x48};

	PX4_INFO("PCA9545A channel x MS4515DO address sweep:");
	PX4_INFO(" ch  addr  MR   status   counts");

	for (uint8_t ch = 0; ch < 4; ch++) {
		if (mux_select((uint8_t)(1u << ch)) != PX4_OK) {
			PX4_INFO("  %u   ----  mux select failed", ch);
			continue;
		}

		for (uint8_t addr : kAddrs) {
			set_device_address(addr);
			uint8_t mr = ADDR_READ_MR;

			if (transfer(&mr, 1, nullptr, 0) != PX4_OK) {
				PX4_INFO("  %u   0x%02x  --", ch, addr);
				continue;
			}

			px4_usleep(CONVERSION_INTERVAL);

			uint8_t d[4] {};
			const char *stat = "no-data";
			int counts = -1;

			if (transfer(nullptr, 0, d, sizeof(d)) == PX4_OK) {
				const uint8_t s = (d[0] & 0b1100'0000) >> 6;
				counts = ((d[0] & 0b0011'1111) << 8) | d[1];
				stat = (s == 0) ? "Normal" : (s == 2) ? "Stale" : (s == 3) ? "Fault" : "Rsvd";
			}

			PX4_INFO("  %u   0x%02x  ACK  %-7s  %d", ch, addr, stat, counts);
		}
	}

	set_device_address(_mux_addr);
	mux_select(0x00); // deselect; RunImpl re-selects on its next cycle
	PX4_INFO("scan done");
}

void FlowAngle::print_status()
{
	I2CSPIDriverBase::print_status();

	PX4_INFO("mode: %s, rate: %.1f Hz, q_min: %.1f Pa, mux: 0x%02x",
		 _sim_en ? "SIM" : "HW", (double)_rate_hz, (double)_q_min, (unsigned)_mux_addr);

	for (int i = 0; i < N_CH; i++) {
		const ChannelCfg &c = _cfg[i];
		PX4_INFO("  ch%u role=%u addr=0x%02x P[%.0f..%.0f]Pa last=%.2fPa %s",
			 (unsigned)__builtin_ctz(c.mux_bit), (unsigned)c.role, (unsigned)c.addr,
			 (double)c.p_min_pa, (double)c.p_max_pa,
			 (double)_samp[i].press_pa, _samp[i].ok ? "ok" : "--");
	}

	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_fault_perf);
}
