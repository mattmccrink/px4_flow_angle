# px4_flow_angle

Out-of-tree PX4 module for five-hole-probe alpha/beta sensing (three MS4525DO
behind a TCA9548A mux on a dedicated I2C bus).

The whole point of this layout is version resilience: **PX4-Autopilot stays a
pristine, unmodified checkout.** All of our code lives here and is pulled in at
build time via `EXTERNAL_MODULES_LOCATION`. To move to a new PX4 release you bump
the submodule tag and rebuild — the only thing you ever have to fix is this small
module against a changed API, never a rebase against upstream.

Target: **PX4 v1.17 (current stable).**

## Layout

```
px4_flow_angle/
├── msg/
│   ├── CMakeLists.txt             # sets config_msg_list_external (SensorFlowAngle.msg)
│   └── SensorFlowAngle.msg        # -> topic sensor_flow_angle, struct sensor_flow_angle_s
└── src/
    ├── CMakeLists.txt             # sets config_module_list_external -> modules/flow_angle
    └── modules/flow_angle/
        ├── CMakeLists.txt         # px4_add_module(... EXTERNAL)
        ├── FlowAngle.hpp / .cpp   # milestone-1 scaffold: publishes synthetic data
        └── flow_angle_params.c    # FA_SIM_EN, FA_RATE, FA_Q_MIN
```

Note on v1.17: the build reads `config_module_list_external` from
`src/CMakeLists.txt` (via `add_subdirectory(".../src")` + `PARENT_SCOPE`) — **not**
from a repo-root `CMakeLists.txt`. Older devguide pages describe a root-level
file; that layout does not work on v1.17.0. The `msg/` side is unchanged: the
in-tree `msg/CMakeLists.txt` picks up `${EXTERNAL_MODULES_LOCATION}/msg`.

## One-time setup (superproject with PX4 pinned as a submodule)

```bash
git init px4-flowangle-ws && cd px4-flowangle-ws
git submodule add https://github.com/PX4/PX4-Autopilot.git
git -C PX4-Autopilot checkout v1.17.0
git -C PX4-Autopilot submodule update --init --recursive
# drop this repo in next to it
git submodule add <your-remote>/px4_flow_angle
```

Recording the PX4 tag as a submodule pointer is what makes "which PX4 have I
validated against" an answer in git rather than a memory.

## Build + run in SITL (no hardware)

```bash
cd PX4-Autopilot
make px4_sitl gazebo-classic EXTERNAL_MODULES_LOCATION=$PWD/../px4_flow_angle
```

Notes:
- The build directory must not already exist the first time you pass
  `EXTERNAL_MODULES_LOCATION`; if it does, delete `build/` or set the CMake
  variable inside the existing build folder.
- Any SITL backend is fine (`gz`, `gazebo-classic`, `jmavsim`); we only need the
  shell.

At the `pxh>` prompt:

```
pxh> flow_angle start
pxh> flow_angle status
pxh> listener sensor_flow_angle
pxh> uorb top sensor_flow_angle
```

You should see alpha/beta sweeping and `valid: True`. That confirms the
out-of-tree build, the custom message, and publication all work.

## Confirm logging + telemetry

- **Logging:** `tools/patch_px4.py` adds `sensor_flow_angle` to the logger's
  default set (a one-line `add_topic(...)` in `add_default_topics()`), so it lands
  in the ulog alongside the standard topics. Flight Review has no plot for a custom
  topic, but the data is in the ulog — use PlotJuggler or pyulog (`ulog2csv`) to
  pull `sensor_flow_angle` for analysis.
- **Live view in QGC:** the driver also publishes a `debug_array` (name `"flow"`,
  `data[0]=alpha°`, `data[1]=beta°`, `data[2]=TAS`), which PX4 streams as
  `DEBUG_FLOAT_ARRAY` by default — visible in QGC's MAVLink Inspector, no custom
  dialect needed. This is live view only; it only reaches the ulog if `SDLOG_PROFILE`
  includes the debug bit, which is why logging uses the real topic (above) instead.
- **Startup on hardware (later):** start the module from the SD card's extras
  startup script rather than editing ROMFS, e.g. add `flow_angle start` to
  `/fs/microsd/etc/extras.txt` — keeps boot config out of the firmware image.

## PX4-tree patches (tools/patch_px4.py)

Two small, content-matched, idempotent edits to your PX4 checkout, applied with:

    python3 tools/patch_px4.py /home/mattmccrink/PX4-Autopilot

1. **External-modules reorder** (root `CMakeLists.txt`). PX4 configures external
   modules before `src/lib` and the in-tree drivers, so an external module can't
   `DEPENDS` on `px4_work_queue` / `drivers__device` — those targets don't exist
   yet at configure time. The patch moves the external-modules block to just after
   the in-tree module loop (still ahead of the events/metadata/parameters libs that
   scan module sources). Milestone 1 (thread-based, no DEPENDS) compiles without it
   on a clean `v1.17.0`; milestone 2's `I2CSPIDriver` needs it.
2. **Log topic** (`src/modules/logger/logged_topics.cpp`). Adds
   `add_topic("sensor_flow_angle", 100)` to `add_default_topics()` so the topic is
   recorded in the onboard ulog alongside the standard set.

Both re-apply cleanly after bumping PX4 to a new tag — keep this in your build
script. Checking out a tag resets `CMakeLists.txt`, so re-run the script after any
`git checkout`.

> A `ModuleBase` "expected template-name" error is NOT a reason to touch these
> patches: that specific error means the PX4 checkout is on `main`/beta instead of
> the `v1.17.0` tag (the beta refactored the `ModuleBase` template out of
> `module.h`). Pin the submodule to the tag. See "One-time setup" above.

Two other choices in this scaffold that keep it robust:

- Milestone 1 is a **thread-based `ModuleBase`** (`px4_task_spawn_cmd` + `run()`),
  no `DEPENDS` needed — mirrors `src/templates/template_module`.
- **Params are deferred to milestone 2.** In-tree `*_params.c` files use
  `PARAM_DEFINE_*` with no includes, relying on a build-provided force-include that
  isn't applied to external-module sources (compiling one there fails with
  `expected ')' before numeric constant`). Milestone 1 runs on compiled-in defaults
  (`FA_SIM_EN=1`, 50 Hz); `load_parameters()` already probes for `FA_*` via
  `param_find` and falls back cleanly, so params drop in later via `module.yaml`
  with no code change to the run loop.

## Milestones

1. **(this)** Out-of-tree scaffold publishing synthetic `sensor_flow_angle` in
   SITL. Proves the version-resilient plumbing before any hardware code.
2. Swap base to `I2CSPIDriver`, own the TCA9548A + three MS4525DO on the
   dedicated bus, publish `differential_pressure` for the pitot sensor (feeds the
   stock airspeed -> EKF2 path) and `sensor_flow_angle` for the reduction.
3. Bench + tunnel validation against the Calspan-calibrated map; calibration
   coefficients loaded from params so recal never needs a recompile.

## Milestone 2 — hardware driver (bench bring-up)

The module is now an `I2CSPIDriver` that owns a **PCA9545A** I2C switch and three
**MS4515DO** sensors behind it. The milestone-1 `flow_angle_params.c` is gone —
parameters live in `module.yaml` — and there is a new `flow_angle_main.cpp`
(command dispatch + bus instantiation). `FlowAngle.{hpp,cpp}` are the driver.

### Confirmed hardware (this board)

- Bus: **I2C4** on a Pixhawk 4; the mux enumerates at **0x70** (A1=A0=GND).
- Three sensors, one per switch channel, all sharing address **0x46**:

  | channel | role     | select byte | I2C addr | sensor |
  |:-------:|----------|:-----------:|:--------:|--------|
  | 0       | alpha    | 0x01        | 0x46     | MS4515DO, +/-4 inH2O differential, type B |
  | 1       | airspeed | 0x02        | 0x46     | MS4515DO, +/-20 inH2O differential, type B |
  | 2       | beta     | 0x04        | 0x46     | MS4515DO, +/-4 inH2O differential, type B |

  All three are marked `3BK` = 3.3V / output type B (5-95%) / interface K (0x46).
  Three identical addresses is exactly what the mux disambiguates. Read this off
  the physical package, not the Digikey ordering string (which drifted).

### How it reads the bus

The driver registers at the mux address (0x70) and retargets to a sensor address
with `set_device_address()` per transaction — one driver owns switch + sensors.
Each cycle, for every channel: write the select byte to the PCA9545A, issue a
Read-MR, wait `CONVERSION_INTERVAL` for the conversion, then Read-DF4 and decode
(14-bit bridge + 11-bit temp, lifted from the in-tree `ms4525do`, type-B constants).
The uniform MR->delay->DF path is what the Low-Power airspeed variant needs and is
harmless to the free-running parts.

PX4 serializes every driver on a bus onto one work queue (`wq:I2C4`), so the module
coexists with other I2C devices with no core changes and no locking.

Publications: `differential_pressure` (pitot only -> stock airspeed selector ->
EKF2), `sensor_flow_angle` (alpha/beta + q + TAS + raw diffs), and a `debug_array`
named `flow` (alpha deg / beta deg / TAS) for live QGC viewing.

> The alpha/beta reduction is a **placeholder** (`alpha = FA_CAL_A * dp_alpha / q`).
> The real Calspan tunnel map is milestone 3 and drops into `publish_cycle()` +
> the `FA_CAL_*` params with no change to the read loop.

### Build

```bash
python3 tools/patch_px4.py /path/to/PX4-Autopilot   # required: reorder + log topic
cd PX4-Autopilot
make px4_fmu-v5_default EXTERNAL_MODULES_LOCATION=$PWD/../px4_flow_angle
```

Milestone 2 depends on the CMake reorder patch (external module `DEPENDS
px4_work_queue drivers__device` must resolve). Re-run `patch_px4.py` after any
`git checkout` of the PX4 tree.

### Bring-up

```
pxh> param set FA_SIM_EN 0
pxh> flow_angle start -b 4 -a 0x70 -f 400
pxh> flow_angle scan
pxh> flow_angle status
pxh> listener sensor_flow_angle
```

`flow_angle scan` sweeps all four channels against {0x28, 0x36, 0x46, 0x48} on the
driver's own work queue (serialized against sampling, no mux race). Expected
signature for this board:

```
 ch  addr  MR   status   counts
  0   0x46  ACK  Normal   ~8192
  1   0x46  ACK  Normal   ~8192
  2   0x46  ACK  Normal   ~8192
  (every other cell: --, ch3 entirely --)
```

Zero-flow counts sit near mid-scale (~8192) because the parts are bidirectional
differential. Sanity check each angle channel with a syringe: suction on one
alpha port should drive its `dp_alpha_pa` **negative through the zero offset** —
a gage part would instead sit near zero and rail. Trust the raw `dp_alpha_pa` /
`dp_beta_pa` / `dynamic_pressure_pa` in `sensor_flow_angle` before believing any
computed angle.

If a channel reads `Stale` where `Normal` is expected, the Low-Power airspeed part
needs more settling — raise `CONVERSION_INTERVAL` in `FlowAngle.hpp`.

### Parameters (module.yaml)

`FA_SIM_EN` (synthetic, no-bus regression), `FA_RATE`, `FA_Q_MIN`, `FA_RHO`,
`FA_OUT_TYP` (0=A / 1=B), `FA_CAL_A` / `FA_CAL_B` (placeholder reduction gains),
and the wiring group `FA_{A,AS,B}_CH` / `FA_{A,AS,B}_ADDR` / `FA_{A,AS,B}_RNG`
(channel index, I2C address in decimal, and full-scale in inH2O per sensor). A
wiring or range change is a param edit, not a recompile.

### FA_SIM_EN vs the SITL smoke test

`init()` is sim-aware: with `FA_SIM_EN=1` it never opens the bus or probes the mux,
it just synthesizes `sensor_flow_angle` at `FA_RATE`. Because M2 is a bus-bound
`I2CSPIDriver`, an instance is only created if `module_start` finds a bindable I2C
bus. On the flight controller that is always true (start with `FA_SIM_EN 1`, no
sensors needed). Under pure SITL it depends on the target exposing an I2C bus — if
`sihsim` doesn't, run the synthetic regression on the FC rather than in SITL. This
is the one behavioural difference from the milestone-1 thread-based module.
