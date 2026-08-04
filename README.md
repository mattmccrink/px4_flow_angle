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
