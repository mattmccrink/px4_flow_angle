#include "FlowAngle.hpp"

#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <systemlib/mavlink_log.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
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
	perf_free(_reject_perf);
	perf_free(_reread_perf);
	perf_free(_stale_perf);
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
	get_i("FA_DBG_RAW", _dbg_raw);
	get_i("FA_CFG_SD", _cfg_sd);
	get_i("FA_ZERO_DPRES", _zero_dpres);
	get_i("FA_FAIL_MS", _fail_ms);
	get_i("FA_MR_MODE", _mr_mode);

	int32_t conv = (int32_t)_conv_us;
	get_i("FA_CONV_US", conv);
	if (conv < 500)    { conv = 500; }
	if (conv > 100000) { conv = 100000; }
	_conv_us = (uint32_t)conv;

	get_f("FA_RATE",   _rate_hz);
	get_f("FA_Q_MIN",  _q_min);
	get_f("FA_RHO",    _rho);
	get_f("FA_CAL_A",  _cal_a);
	get_f("FA_CAL_B",  _cal_b);
	get_f("FA_OFF_A",  _off[(int)Role::ALPHA]);
	get_f("FA_OFF_AS", _off[(int)Role::PITOT]);
	get_f("FA_OFF_B",  _off[(int)Role::BETA]);

	if (_rate_hz < 1.f)  { _rate_hz = 50.f; }
	if (_rho     < 0.1f) { _rho = 1.225f; }

	// output type: 0 = A (10-90%), 1 = B (5-95%). Applied to all channels here as a
	// default; a per-channel SD config 'type' overrides it in load_config_file().
	int32_t out_typ = 1;
	get_i("FA_OUT_TYP", out_typ);

	const float off = (out_typ == 0) ? 0.1f  : 0.05f;
	const float spn = (out_typ == 0) ? 0.8f  : 0.9f;

	for (int i = 0; i < N_CH; i++) { _cfg[i].out_offset = off; _cfg[i].out_span = spn; }

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

const char *FlowAngle::role_str(Role r)
{
	switch (r) {
	case Role::ALPHA: return "alpha";
	case Role::PITOT: return "airspeed";
	case Role::BETA:  return "beta";
	}

	return "?";
}

static char *fa_trim(char *s)
{
	while (*s == ' ' || *s == '\t' || *s == '\r' || *s == '\n') { s++; }

	char *end = s + strlen(s);

	while (end > s && (end[-1] == ' ' || end[-1] == '\t' || end[-1] == '\r' || end[-1] == '\n')) {
		*--end = 0;
	}

	return s;
}

static float fa_unit_to_pa(const char *u)
{
	if (strcmp(u, "inH2O") == 0 || strcmp(u, "inh2o") == 0) { return 248.84f; }
	if (strcmp(u, "psi") == 0)                              { return 6894.757f; }
	if (strcmp(u, "kPa") == 0 || strcmp(u, "kpa") == 0)     { return 1000.f; }
	if (strcmp(u, "hPa") == 0 || strcmp(u, "mbar") == 0)    { return 100.f; }
	if (strcmp(u, "Pa") == 0 || strcmp(u, "pa") == 0)       { return 1.f; }

	return -1.f;   // unknown
}

static float fa_temp_c(const uint8_t d[4])
{
	// 11-bit temperature field -> degrees C
	const int16_t t11 = ((d[2] << 8) + (0b1110'0000 & d[3])) / (1 << 5);
	return (200.f * t11) / 2047.f - 50.f;
}

bool FlowAngle::load_config_file()
{
	FILE *f = fopen(FA_CONFIG_PATH, "r");

	if (f == nullptr) {
		PX4_WARN("config: %s not found", FA_CONFIG_PATH);
		return false;
	}

	// work on a copy so a malformed file never half-applies
	ChannelCfg tmp[N_CH];
	float range[N_CH]; float unit[N_CH]; bool bidir[N_CH];
	bool s_role[N_CH], s_addr[N_CH], s_range[N_CH], s_units[N_CH], s_type[N_CH], touched[N_CH];

	for (int i = 0; i < N_CH; i++) {
		tmp[i] = _cfg[i];
		range[i] = 0.f; unit[i] = 0.f; bidir[i] = true;
		s_role[i] = s_addr[i] = s_range[i] = s_units[i] = s_type[i] = touched[i] = false;
	}

	int file_ver = -1;
	int errors = 0;
	int lineno = 0;
	char line[128];

	while (fgets(line, sizeof(line), f) != nullptr) {
		lineno++;

		char *hash = strchr(line, '#'); if (hash) { *hash = 0; }
		char *eq = strchr(line, '=');
		if (eq == nullptr) { continue; }   // blank or comment-only

		*eq = 0;
		char *key = fa_trim(line);
		char *val = fa_trim(eq + 1);
		if (*key == 0) { continue; }

		if (strcmp(key, "version") == 0) { file_ver = atoi(val); continue; }

		if (key[0] == 'c' && key[1] == 'h' && key[2] >= '0' && key[2] <= '9' && key[3] == '.') {
			const int ch = key[2] - '0';

			if (ch >= N_CH) { PX4_WARN("config line %d: channel %d out of range", lineno, ch); errors++; continue; }

			const char *fld = key + 4;
			touched[ch] = true;

			if (strcmp(fld, "role") == 0) {
				if (strcmp(val, "alpha") == 0)                                  { tmp[ch].role = Role::ALPHA; }
				else if (strcmp(val, "airspeed") == 0 || strcmp(val, "pitot") == 0) { tmp[ch].role = Role::PITOT; }
				else if (strcmp(val, "beta") == 0)                             { tmp[ch].role = Role::BETA; }
				else { PX4_WARN("config line %d: bad role '%s'", lineno, val); errors++; continue; }

				s_role[ch] = true;

			} else if (strcmp(fld, "addr") == 0) {
				tmp[ch].addr = (uint8_t)strtol(val, nullptr, 0); s_addr[ch] = true;

			} else if (strcmp(fld, "range") == 0) {
				range[ch] = strtof(val, nullptr); s_range[ch] = true;

			} else if (strcmp(fld, "units") == 0) {
				unit[ch] = fa_unit_to_pa(val);
				if (unit[ch] < 0.f) { PX4_WARN("config line %d: unknown units '%s'", lineno, val); errors++; continue; }
				s_units[ch] = true;

			} else if (strcmp(fld, "type") == 0) {
				if (strcmp(val, "A") == 0 || strcmp(val, "a") == 0)      { tmp[ch].out_offset = 0.1f;  tmp[ch].out_span = 0.8f; }
				else if (strcmp(val, "B") == 0 || strcmp(val, "b") == 0) { tmp[ch].out_offset = 0.05f; tmp[ch].out_span = 0.9f; }
				else { PX4_WARN("config line %d: bad type '%s' (A or B)", lineno, val); errors++; continue; }

				s_type[ch] = true;

			} else if (strcmp(fld, "bidir") == 0) {
				bidir[ch] = (strcmp(val, "yes") == 0 || strcmp(val, "true") == 0 || strcmp(val, "1") == 0);

			} else if (strcmp(fld, "family") == 0) {
				// informational only (4515 / 4525 share the DF frame format)

			} else {
				PX4_WARN("config line %d: unknown field 'ch%d.%s'", lineno, ch, fld); errors++;
			}

			continue;
		}

		PX4_WARN("config line %d: unknown key '%s'", lineno, key); errors++;
	}

	fclose(f);

	if (file_ver != FA_CONFIG_VERSION) {
		PX4_ERR("config: version %d != expected %d -- ignoring file", file_ver, FA_CONFIG_VERSION);
		return false;
	}

	int ready = 0;

	for (int ch = 0; ch < N_CH; ch++) {
		if (!touched[ch]) { continue; }

		if (!(s_role[ch] && s_addr[ch] && s_range[ch] && s_units[ch] && s_type[ch])) {
			PX4_ERR("config ch%d: incomplete (need role, addr, range, units, type) -- ignoring file", ch);
			return false;
		}

		if (range[ch] <= 0.f) { PX4_ERR("config ch%d: range must be > 0 -- ignoring file", ch); return false; }

		const float fs = range[ch] * unit[ch];
		tmp[ch].p_max_pa = fs;
		tmp[ch].p_min_pa = bidir[ch] ? -fs : 0.f;
		ready++;
	}

	if (ready == 0) { PX4_WARN("config: no complete channel entries -- using defaults"); return false; }

	for (int i = 0; i < N_CH; i++) { _cfg[i] = tmp[i]; }

	PX4_INFO("config: loaded %s (v%d, %d channel(s), %d parse warning(s))",
		 FA_CONFIG_PATH, file_ver, ready, errors);

	for (int i = 0; i < N_CH; i++) {
		const ChannelCfg &c = _cfg[i];
		PX4_INFO("  ch%d %-8s addr=0x%02x P[%.0f..%.0f]Pa type%s",
			 i, role_str(c.role), (unsigned)c.addr,
			 (double)c.p_min_pa, (double)c.p_max_pa, (c.out_span > 0.85f) ? "B" : "A");
	}

	return true;
}

void FlowAngle::verify_channels()
{
	for (int i = 0; i < N_CH; i++) {
		ChannelCfg &c = _cfg[i];
		c.verified = false;

		bool got = false;
		float temp = -99.f;

		if (mux_select(c.mux_bit) == PX4_OK && send_mr(c.addr) == PX4_OK) {
			px4_usleep(_conv_us);
			uint8_t d[4] {};

			if (transfer(nullptr, 0, d, 4) == PX4_OK) {
				temp = fa_temp_c(d);
				got = true;
			}
		}

		// A converting MS45x reports roughly ambient; a dead/absent/unpowered part
		// rails temperature to the -50C reset floor. This is the acceptance gate.
		if (got && temp > -20.f && temp < 60.f) {
			c.verified = true;
			PX4_INFO("verify ch%d %-8s 0x%02x: temp=%.1fC  OK",
				 i, role_str(c.role), (unsigned)c.addr, (double)temp);

		} else if (got) {
			PX4_WARN("verify ch%d %-8s 0x%02x: temp=%.1fC  NOT CONVERTING (dead/unpowered)",
				 i, role_str(c.role), (unsigned)c.addr, (double)temp);

		} else {
			PX4_WARN("verify ch%d %-8s 0x%02x: no reply (absent / mux / wiring)",
				 i, role_str(c.role), (unsigned)c.addr);
		}
	}

	set_device_address(_mux_addr);
	mux_select(0x00);
}

void FlowAngle::update_health()
{
	// Runs once per cycle. Declares a channel failed after ~_fail_ms of continuous
	// dropouts (debounced against transient I2C hiccups), and alerts QGC on the
	// healthy->failed and failed->healthy edges. Re-alerts every 30 s while failed,
	// so the message stays visible on a long flight.
	const hrt_abstime now = hrt_absolute_time();
	const float fc = (float)_fail_ms * _rate_hz / 1000.f;
	const uint32_t fail_cycles = (fc < 1.f) ? 1u : (uint32_t)fc;

	for (int i = 0; i < N_CH; i++) {
		const bool ok = _samp[i].ok;
		const Role role = _cfg[i].role;

		if (ok) {
			_fail_count[i] = 0;

			if (_ch_failed[i]) {
				_ch_failed[i] = false;
				mavlink_log_info(&_mavlink_log_pub, "flow_angle: %s sensor recovered", role_str(role));
			}

			continue;
		}

		if (_fail_count[i] < 0xFFFFFFFF) { _fail_count[i]++; }

		const bool cross = (!_ch_failed[i] && _fail_count[i] >= fail_cycles);
		const bool renotify = (_ch_failed[i] && (now - _last_alert[i]) > 30_s);

		if (cross || renotify) {
			_ch_failed[i] = true;
			_last_alert[i] = now;

			if (role == Role::PITOT) {
				mavlink_log_critical(&_mavlink_log_pub,
						     "flow_angle: AIRSPEED sensor lost -- airspeed/EKF2 affected");

			} else {
				mavlink_log_critical(&_mavlink_log_pub,
						     "flow_angle: %s sensor lost -- flow-angle data affected", role_str(role));
			}
		}
	}
}

void FlowAngle::enforce_dpres_off()
{
	param_t h = param_find("SENS_DPRES_OFF");

	if (h == PARAM_INVALID) { return; }

	float off = 0.f;
	param_get(h, &off);

	if (fabsf(off) <= 0.001f) { return; }   // already zero

	if (_zero_dpres != 0) {
		// flow_angle owns the airspeed zero (via FA_OFF_AS + flow_angle null). Clear
		// the stock offset so it cannot stack. param_set notifies the airspeed selector
		// to re-read; we do not param_save, so it is re-cleared every boot without a
		// flash write. Disable with FA_ZERO_DPRES=0.
		const float zero = 0.f;
		param_set(h, &zero);
		PX4_WARN("SENS_DPRES_OFF was %.2f; forced to 0 so flow_angle owns the airspeed zero (FA_ZERO_DPRES=1).",
			 (double)off);

	} else {
		PX4_WARN("SENS_DPRES_OFF=%.2f is nonzero: the stock airspeed offset STACKS on the flow_angle null.",
			 (double)off);
		PX4_WARN("Run 'param set SENS_DPRES_OFF 0', or set FA_ZERO_DPRES=1 to clear it automatically.");
	}
}

int FlowAngle::init()
{
	load_parameters(); // read FA_SIM_EN (and the rest) before deciding to touch the bus

	PX4_INFO("flow_angle v" FLOW_ANGLE_VERSION " starting (%s)", _sim_en ? "SIM" : "HW");

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

	// Channel config: SD file (if enabled and present) overrides the compiled
	// defaults / params. Falls back cleanly on any problem.
	if (_cfg_sd != 0) {
		if (!load_config_file()) {
			PX4_INFO("using compiled/param channel config (no valid SD config)");
		}
	}

	// Boot-time acceptance gate: confirm each channel's sensor is present AND
	// actually converting (temperature reads ambient, not the -50C reset rail).
	verify_channels();

	enforce_dpres_off();   // warn if the stock airspeed offset would stack on our null

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

int FlowAngle::send_mr(uint8_t addr)
{
	// Measurement request to trigger a conversion. Mode 0 writes a 0x00 data byte
	// (works for free-running parts, matches the in-tree ms4525do). Mode 1 issues
	// an address-only write (no data), which some low-power parts require to wake.
	set_device_address(addr);

	if (_mr_mode != 0) {
		return transfer(nullptr, 0, nullptr, 0);
	}

	uint8_t mr = ADDR_READ_MR;
	return transfer(&mr, 1, nullptr, 0);
}

float FlowAngle::transfer_fn(int16_t bridge, const ChannelCfg &c) const
{
	// Inversion of the MS45x pressure transfer function, per-channel output type:
	//   type A: 10%..90% -> offset 0.10, span 0.80
	//   type B:  5%..95% -> offset 0.05, span 0.90
	return (bridge - c.out_offset * FULL_SCALE) * (c.p_max_pa - c.p_min_pa)
	       / (c.out_span * FULL_SCALE) + c.p_min_pa;
}

float FlowAngle::apply_offset(Role r, float raw_pa, float temp_c) const
{
	// Scalar per-channel zero captured by `flow_angle null`. temp_c is the seam for
	// a future temperature-dependent offset (M3 oven cal); unused for now.
	(void)temp_c;
	return raw_pa - _off[(int)r];
}

FlowAngle::FrameResult FlowAngle::read_frame(const ChannelCfg &c, ChannelSample &out, uint8_t raw[4])
{
	// mux already selected for this channel; retarget to the sensor and fetch 4 bytes.
	set_device_address(c.addr);

	raw[0] = raw[1] = raw[2] = raw[3] = 0;

	if (transfer(nullptr, 0, raw, 4) != PX4_OK) {
		perf_count(_comms_errors);
		return FrameResult::Comms;
	}

	const uint8_t status = (raw[0] & 0b1100'0000) >> 6;

	if (status == (uint8_t)Status::Fault) {
		perf_count(_fault_perf);
		return FrameResult::Fault;
	}

	// Only a Normal frame carries fresh, trustworthy data. A Stale/Reserved frame
	// means we fetched before the (low-power) part finished -- its pressure field
	// is old/uninitialized (this is the "frozen 5529 Pa" bug). Reject, don't latch.
	if (status != (uint8_t)Status::Normal) {
		return FrameResult::Stale;
	}

	const int16_t bridge = ((raw[0] & 0b0011'1111) << 8) | raw[1];
	const int16_t temp11 = ((raw[2] << 8) + (0b1110'0000 & raw[3])) / (1 << 5);
	const float press = transfer_fn(bridge, c);

	// Physical-range backstop: a stale/garbage register (e.g. a near-0xFFFF bridge)
	// decodes to a pressure beyond the sensor's full scale -- impossible, so reject
	// even though the status bits claimed Normal. Small tolerance for rail noise.
	const float tol = 0.02f * (c.p_max_pa - c.p_min_pa);

	if (press < c.p_min_pa - tol || press > c.p_max_pa + tol) {
		return FrameResult::OutOfRange;
	}

	out.press_pa = press;
	out.temp_c   = (200.f * temp11) / 2047.f - 50.f;
	return FrameResult::Ok;
}

const char *FlowAngle::result_str(FlowAngle::FrameResult r)
{
	switch (r) {
	case FlowAngle::FrameResult::Ok:         return "ok";
	case FlowAngle::FrameResult::Comms:      return "comms";
	case FlowAngle::FrameResult::Fault:      return "fault";
	case FlowAngle::FrameResult::Stale:      return "stale";
	case FlowAngle::FrameResult::OutOfRange: return "out-of-range";
	}

	return "?";
}

void FlowAngle::log_raw(int idx, const uint8_t raw[4], FrameResult r, int tries)
{
	const hrt_abstime now = hrt_absolute_time();

	if (now - _last_dbg < 250_ms) { return; }   // ~4 Hz cap so the shell stays readable

	_last_dbg = now;

	const uint16_t bridge = ((raw[0] & 0x3F) << 8) | raw[1];
	PX4_INFO("ch%d raw=%02x %02x %02x %02x st=%u bridge=%u -> %s (tries=%d)",
		 idx, raw[0], raw[1], raw[2], raw[3], (unsigned)((raw[0] >> 6) & 0x3),
		 (unsigned)bridge, result_str(r), tries);
}

void FlowAngle::read_channel(int idx)
{
	const ChannelCfg &c = _cfg[idx];
	uint8_t raw[4] {};

	// First attempt uses the measurement request already issued in MEASURE.
	mux_select(c.mux_bit);
	FrameResult r = read_frame(c, _samp[idx], raw);

	int tries = 0;

	// On a rejected frame, take a fresh measurement (MR -> convert -> DF) and retry.
	// A bare DF re-fetch would return the same stale register on a low-power part,
	// so each retry re-issues the measurement request.
	while (r != FrameResult::Ok && tries < MAX_REREADS) {
		perf_count(_reject_perf);
		tries++;

		if (mux_select(c.mux_bit) != PX4_OK) { r = FrameResult::Comms; continue; }

		if (send_mr(c.addr) != PX4_OK) { r = FrameResult::Comms; continue; }

		px4_usleep(_conv_us);
		mux_select(c.mux_bit);
		r = read_frame(c, _samp[idx], raw);
	}

	_last_tries[idx] = (uint8_t)tries;
	_last_result[idx] = r;
	_last_raw[idx][0] = raw[0]; _last_raw[idx][1] = raw[1];
	_last_raw[idx][2] = raw[2]; _last_raw[idx][3] = raw[3];

	if (r == FrameResult::Ok) {
		_samp[idx].ok = true;

		if (tries > 0) { perf_count(_reread_perf); }

		if (_dbg_raw) { log_raw(idx, raw, r, tries); }

	} else {
		// exhausted: never latch a bad frame -- drop the channel this cycle
		_samp[idx].ok = false;
		perf_count(_stale_perf);
		log_raw(idx, raw, r, tries);   // always surface a fully-failed channel
	}
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

	// A `flow_angle scan` on the command thread parks us here so it owns the bus.
	if (_pause.load()) {
		ScheduleDelayed(20_ms);
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

			if (send_mr(c.addr) != PX4_OK) {
				perf_count(_comms_errors);
				_samp[_ch_idx].ok = false;

				if (_ch_idx + 1 < N_CH) { _ch_idx++; ScheduleNow(); }
				else { perf_end(_sample_perf); _ch_idx = 0; publish_cycle(); schedule_next_cycle(); }

				return;
			}

			_timestamp_sample = hrt_absolute_time();
			_phase = Phase::READ;
			ScheduleDelayed(_conv_us);
			break;
		}

	case Phase::READ: {
			// stale-reject + bounded re-read; read_channel owns its mux selects
			read_channel(_ch_idx);

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

	update_health();   // per-channel dropout monitor -> QGC alerts

	const ChannelSample *pitot = sample_for(Role::PITOT);
	const ChannelSample *a     = sample_for(Role::ALPHA);
	const ChannelSample *b     = sample_for(Role::BETA);

	// --- pitot -> differential_pressure (feeds stock airspeed selector / EKF2) ---
	if (pitot && pitot->ok) {
		differential_pressure_s dp{};
		dp.timestamp_sample = _timestamp_sample;
		dp.device_id = get_device_id();
		// Apply the flow_angle zero here too, so the stock airspeed selector / EKF2 /
		// VFR_HUD path uses the same offset as our reduction. Keep SENS_DPRES_OFF at 0
		// so there is exactly one correction (see check_dpres_off).
		dp.differential_pressure_pa = apply_offset(Role::PITOT, pitot->press_pa, pitot->temp_c);
		dp.temperature = pitot->temp_c;
		dp.error_count = perf_event_count(_comms_errors);
		dp.timestamp = now;
		_diff_press_pub.publish(dp);
	}

	// --- 5-hole reduction (PLACEHOLDER linear model; milestone 3 = Calspan map) ---
	// 5-hole reduction inputs, offset-corrected (nulled) via apply_offset(). Note the
	// published differential_pressure above stays RAW -- the stock airspeed selector
	// applies its own SENS_DPRES_OFF, so double-correcting there would be wrong. Our
	// internal q / dp_alpha / dp_beta use the flow_angle zero.
	const float q  = (pitot && pitot->ok) ? fmaxf(apply_offset(Role::PITOT, pitot->press_pa, pitot->temp_c), 0.f) : 0.f;
	const float dpa = (a && a->ok) ? apply_offset(Role::ALPHA, a->press_pa, a->temp_c) : 0.f;
	const float dpb = (b && b->ok) ? apply_offset(Role::BETA,  b->press_pa, b->temp_c) : 0.f;

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
	// Runs on the COMMAND thread (module_custom_method(..., run_on_work_queue=false)),
	// so PX4_INFO reaches the same console as `start`/`status`. Since we are no longer
	// serialized by the work queue, park RunImpl via _pause and let it quiesce before
	// touching the bus. cli.custom2 selects the sub-command (0=scan, 1=null).
	if (!_bus_ready) {
		PX4_WARN("needs the bus initialized -- start with FA_SIM_EN 0 first");
		return;
	}

	_pause.store(true);
	px4_usleep(3 * (uint32_t)(1e6f / _rate_hz) + 20000);   // wait for RunImpl to park

	if (cli.custom2 == 1) {
		do_null(cli.custom1);

	} else {
		do_scan(cli.custom1);
	}

	set_device_address(_mux_addr);
	mux_select(0x00);

	// resume the sample loop cleanly from the top of a cycle
	_ch_idx = 0;
	_phase = Phase::MEASURE;
	_pause.store(false);
}

void FlowAngle::do_scan(int stream)
{
	static constexpr uint8_t kAddrs[] = {0x28, 0x36, 0x46, 0x48};   // I / J / K / 0

	if (stream > 1) {
		// pressure-watch mode: stream N frames from each configured channel at ~2 Hz,
		// with full bytes + decoded Pa, so you can watch counts move under pressure.
		PX4_INFO("streaming %d frames/channel -- apply pressure and watch cnt/Pa:", stream);

		for (int n = 0; n < stream; n++) {
			for (int i = 0; i < N_CH; i++) {
				const ChannelCfg &c = _cfg[i];
				uint8_t d[4] {};
				uint8_t st = 0xFF; int counts = -1; float pa = 0.f; bool got = false;

				if (mux_select(c.mux_bit) == PX4_OK) {
					if (send_mr(c.addr) == PX4_OK) {
						px4_usleep(_conv_us);

						if (transfer(nullptr, 0, d, 4) == PX4_OK) {
							st = (d[0] & 0b1100'0000) >> 6;
							counts = ((d[0] & 0b0011'1111) << 8) | d[1];
							pa = transfer_fn((int16_t)counts, c);
							got = true;
						}
					}
				}

				if (got) {
					PX4_INFO("[%d] ch%d(0x%02x) raw=%02x %02x %02x %02x st=%u cnt=%d %.1fPa %.1fC",
						 n, i, (unsigned)c.addr, d[0], d[1], d[2], d[3],
						 (unsigned)st, counts, (double)pa, (double)fa_temp_c(d));

				} else {
					PX4_INFO("[%d] ch%d(0x%02x) no reply", n, i, (unsigned)c.addr);
				}
			}

			px4_usleep(500000);   // ~2 Hz
		}

	} else {
		// presence sweep: every channel x every candidate address, full frame bytes.
		PX4_INFO("PCA9545A channel x MS45x address sweep (full frame):");
		PX4_INFO(" ch  addr  raw bytes      st  counts  temp");

		for (uint8_t ch = 0; ch < 4; ch++) {
			if (mux_select((uint8_t)(1u << ch)) != PX4_OK) {
				PX4_INFO("  %u   ----  mux select failed", ch);
				continue;
			}

			for (uint8_t addr : kAddrs) {
				if (send_mr(addr) != PX4_OK) {
					PX4_INFO("  %u  0x%02x  --", ch, addr);
					continue;
				}

				px4_usleep(_conv_us);
				uint8_t d[4] {};

				if (transfer(nullptr, 0, d, 4) == PX4_OK) {
					const uint8_t s = (d[0] & 0b1100'0000) >> 6;
					const int counts = ((d[0] & 0b0011'1111) << 8) | d[1];
					PX4_INFO("  %u  0x%02x  %02x %02x %02x %02x  %u  %6d  %.1fC",
						 ch, addr, d[0], d[1], d[2], d[3], (unsigned)s, counts, (double)fa_temp_c(d));

				} else {
					PX4_INFO("  %u  0x%02x  ACK (no DF)", ch, addr);
				}
			}
		}
	}

	PX4_INFO("scan done");
}

void FlowAngle::do_null(int n)
{
	if (n < 1) { n = 50; }

	PX4_INFO("nulling %d samples/channel -- probe MUST be capped (no flow):", n);

	for (int i = 0; i < N_CH; i++) {
		const ChannelCfg &c = _cfg[i];
		float sum = 0.f;
		int got = 0;

		for (int k = 0; k < n; k++) {
			if (mux_select(c.mux_bit) != PX4_OK) { continue; }
			if (send_mr(c.addr) != PX4_OK) { continue; }

			px4_usleep(_conv_us);
			uint8_t d[4] {};

			if (transfer(nullptr, 0, d, 4) != PX4_OK) { continue; }

			if (((d[0] & 0b1100'0000) >> 6) != (uint8_t)Status::Normal) { continue; }

			const int16_t bridge = ((d[0] & 0b0011'1111) << 8) | d[1];
			const float pa = transfer_fn(bridge, c);

			if (pa < c.p_min_pa || pa > c.p_max_pa) { continue; }   // reject railed frames

			sum += pa;
			got++;
			px4_usleep(2000);
		}

		if (got < n / 2) {
			PX4_WARN("  ch%d %-8s: only %d/%d good samples -- NOT nulled", i, role_str(c.role), got, n);
			continue;
		}

		const float mean = sum / (float)got;
		const float ceiling = 0.25f * c.p_max_pa;   // sanity: reject obvious flow / fault

		if (fabsf(mean) > ceiling) {
			PX4_WARN("  ch%d %-8s: mean %.1f Pa > %.0f Pa cap -- flow present or fault? NOT nulled",
				 i, role_str(c.role), (double)mean, (double)ceiling);
			continue;
		}

		_off[(int)c.role] = mean;   // live effect immediately

		const char *pname = (c.role == Role::ALPHA) ? "FA_OFF_A"
				    : (c.role == Role::PITOT) ? "FA_OFF_AS" : "FA_OFF_B";
		param_t ph = param_find(pname);

		if (ph != PARAM_INVALID) { param_set(ph, &mean); }

		PX4_INFO("  ch%d %-8s: offset = %.2f Pa (%d samples) -> %s",
			 i, role_str(c.role), (double)mean, got, pname);
	}

	PX4_INFO("null done. Run 'param save' to persist across reboot.");
	enforce_dpres_off();   // remind: the stock offset stacks on this null if nonzero
}

void FlowAngle::print_status()
{
	I2CSPIDriverBase::print_status();

	PX4_INFO("flow_angle v" FLOW_ANGLE_VERSION " | mode: %s, rate: %.1f Hz, q_min: %.1f Pa, mux: 0x%02x, conv: %u us, mr: %d",
		 _sim_en ? "SIM" : "HW", (double)_rate_hz, (double)_q_min, (unsigned)_mux_addr,
		 (unsigned)_conv_us, (int)_mr_mode);

	for (int i = 0; i < N_CH; i++) {
		const ChannelCfg &c = _cfg[i];
		const char *flag = _ch_failed[i] ? " FAILED" : (c.verified ? "" : " UNVERIFIED");
		PX4_INFO("  ch%u %-8s addr=0x%02x P[%.0f..%.0f]Pa last=%.2fPa %s%s (retries=%u)",
			 (unsigned)__builtin_ctz(c.mux_bit), role_str(c.role), (unsigned)c.addr,
			 (double)c.p_min_pa, (double)c.p_max_pa,
			 (double)_samp[i].press_pa, _samp[i].ok ? "ok" : "--",
			 flag, (unsigned)_last_tries[i]);
		PX4_INFO("        raw=%02x %02x %02x %02x st=%u -> %s",
			 _last_raw[i][0], _last_raw[i][1], _last_raw[i][2], _last_raw[i][3],
			 (unsigned)((_last_raw[i][0] >> 6) & 0x3), result_str(_last_result[i]));
	}

	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_reject_perf);
	perf_print_counter(_reread_perf);
	perf_print_counter(_stale_perf);
	perf_print_counter(_fault_perf);
}
