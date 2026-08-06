# 5-Hole Probe → PX4: Alpha/Beta Integration Roadmap

Standalone working document for integrating a custom 5-hole probe (angle of
attack / sideslip) into the PX4 flight stack. Milestone 1 is complete and
verified; milestones 2–3 are the remaining work.

---

## 1. Objective

Measure flow angles (alpha, beta) and airspeed with a custom 5-hole probe and
surface them inside PX4: fed to the airspeed/EKF2 path where appropriate, logged
to the onboard ulog for analysis, and viewable live in QGroundControl. The build
must stay easy to re-validate against new PX4 releases.

## 2. Hardware

- Custom 5-hole probe, calibrated in the lab's 3×5 wind tunnel using the Calspan
  model positioning system.
- Working PCB with **3× MS4525DO** differential pressure sensors:
  - 1× pitot — total − static (airspeed)
  - 2× smaller-range parts — alpha differential and beta differential
- All three sit behind a **TCA9548A I2C mux** on a **dedicated I2C bus**, so they
  can share an I2C address; the mux selects one at a time.
- **Open input for milestone 2:** the mux channel map — which TCA9548A channel is
  pitot vs alpha-pair vs beta-pair — and which flight-controller I2C bus the mux
  hangs off.

## 3. Target & environment

- **PX4 v1.17.0 stable.** Pin the submodule to the tag (see §6, gotcha #1).
- Development on WSL2 / Windows 11.
- Module built **out-of-tree** via `EXTERNAL_MODULES_LOCATION` so PX4-Autopilot
  stays a pristine, tag-pinned checkout. Repo: `px4_flow_angle`.

## 4. Architecture decisions

- **One driver owns the mux.** The stock `ms4525do` driver can't drive three
  same-address sensors behind a mux (no channel-select, and `probe()` can't see
  behind the mux), so a single custom driver selects a channel, reads that
  MS4525DO, and repeats for all three each cycle.
- **Two publications.** `differential_pressure` for the pitot channel feeds the
  stock airspeed selector → EKF2 path unchanged; a custom `sensor_flow_angle`
  uORB topic carries alpha/beta (+ airspeed, dynamic pressure, raw diffs).
- **Reuse the MS4525DO decode.** Lift the 4-byte frame decode and pressure/
  temperature transfer function from `src/drivers/differential_pressure/ms4525do`
  so the pitot reading is bit-identical — matching the part's output-type variant
  (A = 10–90%, B = 5–95%).
- **`FA_SIM_EN` synthetic mode** stays as a hardware-less regression path, so the
  SITL smoke test keeps working on every future PX4 bump.
- **Calibration coefficients live in params** (via `module.yaml`) so a new tunnel
  sweep is a param reload, not a recompile.
- **Logging** via a one-line `add_topic` in `logged_topics.cpp` (applied by the
  patch script). **Live QGC view** via a `debug_array` → `DEBUG_FLOAT_ARRAY`
  mirror — no custom MAVLink dialect.

## 5. Repository layout

```
px4_flow_angle/
├── ROADMAP.md                     # this file
├── README.md                      # build/run/validate details + gotchas
├── msg/
│   ├── CMakeLists.txt             # config_msg_list_external
│   └── SensorFlowAngle.msg        # -> topic sensor_flow_angle
├── src/
│   ├── CMakeLists.txt             # config_module_list_external -> modules/flow_angle
│   └── modules/flow_angle/
│       ├── CMakeLists.txt         # px4_add_module(... EXTERNAL)
│       └── FlowAngle.cpp / .hpp
└── tools/
    └── patch_px4.py               # two idempotent PX4-tree edits (reorder + log topic)
```

## 6. Build setup & hard-won gotchas

1. **Pin PX4 to the v1.17.0 tag.** A submodule left tracking `main`/beta produced
   an `expected template-name before '<'` error on `ModuleBase` (the beta
   refactored the template out of `module.h`). This single misstep generated most
   of the early friction. Verify with `git describe --tags` → must print
   `v1.17.0`.
2. **`config_module_list_external`** goes in `src/CMakeLists.txt` (PX4 does
   `add_subdirectory("${EXTERNAL_MODULES_LOCATION}/src")`), not the repo-root
   `CMakeLists.txt`.
3. **`tools/patch_px4.py <PX4-Autopilot>`** applies two content-matched,
   idempotent edits; re-run after every `git checkout`:
   - Move the external-modules block after the in-tree module loop so
     milestone-2 `DEPENDS px4_work_queue drivers__device` resolve.
   - Add `sensor_flow_angle` to `add_default_topics()` for onboard logging.
4. **Params in external modules.** PX4's force-include for `PARAM_DEFINE_*` isn't
   applied to external-module sources, so a `*_params.c` won't compile there. Use
   `module.yaml` (milestone 2) or `param_find`/`param_get` with fallback defaults.
5. **Headless SITL run:** `make px4_sitl sihsim_quadx EXTERNAL_MODULES_LOCATION=…`
   — SIH provides lockstep time so the module loop actually ticks (`none_iris`
   would hang waiting for a simulator).
6. **WSL2 ↔ QGC (Windows):** add a manual UDP link in QGC to the WSL IP
   (`hostname -I`) on port 18570, or switch WSL to mirrored networking mode.

## 7. Status

### Milestone 1 — COMPLETE (verified in SITL)

- [x] Out-of-tree build wiring (module + custom msg) against pinned v1.17.0.
- [x] Thread-based `ModuleBase` scaffold publishing synthetic `sensor_flow_angle`
      at 50 Hz (`FA_SIM_EN`).
- [x] Custom uORB message builds and advertises; confirmed via `listener`.
- [x] Captured in the onboard ulog (verified with `ulog_info` / `ulog2csv`).
- [x] Live in QGC's MAVLink Inspector via `debug_array` → `DEBUG_FLOAT_ARRAY`
      (name `flow`: alpha°, beta°, TAS).

### Milestone 2 — Hardware driver (NEXT)

- [ ] **Provide the mux channel map + I2C bus** (the one blocking input).
- [ ] Refactor `FlowAngle` from thread-based `ModuleBase` → `I2CSPIDriver`
      (`DEPENDS px4_work_queue drivers__device`).
- [ ] Own the TCA9548A: per cycle, select channel → read MS4525DO → repeat ×3;
      re-select every transaction, handle read failures, optional mux `/RESET`.
- [ ] Lift MS4525DO frame decode + transfer function from the in-tree driver;
      match the part's output-type variant.
- [ ] Publish `differential_pressure` (pitot) + `sensor_flow_angle` (reduction);
      keep angle sensors off the airspeed-selector path.
- [ ] Add params via `module.yaml`: `FA_SIM_EN`, rate, `FA_Q_MIN`, mux channel
      map, and placeholders for the 5-hole calibration coefficients.
- [ ] Bench bring-up: confirm raw pressures per channel before wiring the
      reduction.

### Milestone 3 — Calibration & validation

- [ ] Calspan sweep in the 3×5 tunnel → build the 5-hole reduction map
      (coefficients as functions of the pressure ratios).
- [ ] Load coefficients into params; no recompile for re-cal.
- [ ] Validate alpha/beta against known tunnel angles; check q-normalization
      noise at low q; confirm the smaller-range parts don't saturate at
      max q × max angle.
- [ ] Flight test; analyze `sensor_flow_angle` via PlotJuggler / pyulog.

## 8. Open considerations / backlog

- **Sequential sampling skew.** Muxed reads are ~hundreds of µs apart, not
  simultaneous — negligible at flight rates, worth noting for dynamic tunnel data.
- **Aggregate rate ceiling.** Bounded by 3×(select + read) per cycle; comfortable
  at 50–100 Hz, not kHz.
- **Rev-B sensor choice.** MS5525DSO (finer resolution) or SDP3x (lower offset
  drift) suit the small near-zero alpha/beta differentials better than MS4525DO —
  a fallback if the angle noise floor at low q becomes the limiter.
- **EKF2 fusion.** PX4 has no native alpha/beta fusion; flow angles are currently
  auxiliary (logged/streamed), not fused into the state estimate. Fusing them is
  possible future work.
- **Custom MAVLink message.** If a downstream consumer needs a typed message
  rather than `DEBUG_FLOAT_ARRAY`, add a proper dialect + stream (the fragile,
  in-tree, version-sensitive part — defer until needed).
