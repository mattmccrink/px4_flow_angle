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

- **Logging:** `sensor_flow_angle` is a normal advertised topic. Start logging
  (`logger start` or arm) and confirm the topic appears in the ulog. To have it
  logged by default without editing in-tree `logged_topics.cpp`, the
  version-robust route is to also bridge alpha/beta through an existing debug
  message (`debug_array` -> `DEBUG_FLOAT_ARRAY`), which is logged and streamed to
  the GCS automatically. That bridge is a milestone-1b add.
- **Startup on hardware (later):** start the module from the SD card's extras
  startup script rather than editing ROMFS, e.g. add `flow_angle start` to
  `/fs/microsd/etc/extras.txt` — keeps boot config out of the firmware image.

## External-module build-order constraint (important)

PX4 configures external modules (root CMake ~L408-412) **before** `src/lib`
(~L425), so an external module that lists a `src/lib` target in `DEPENDS` (e.g.
`px4_work_queue`) fails at configure with `non-existent target`. Two consequences
baked into this scaffold:

- Milestone 1 is a **thread-based `ModuleBase`** (`px4_task_spawn_cmd` + `run()`),
  which needs **no `DEPENDS`** at all — mirrors `src/templates/template_module`.
- Params are read with raw `param_find` / `param_get` (not `DEFINE_PARAMETERS`),
  so the module compiles and runs even if the external param scan doesn't register
  `FA_*`. Check with `param show FA_RATE` in nsh; if they're missing the module
  still runs on the built-in defaults.

Milestone 2 (I2CSPIDriver owning the mux) *does* need `drivers__device` and
`px4_work_queue`. The cleanest fix is a small, documented one-block reorder of the
PX4 root `CMakeLists.txt` — move the external-modules `add_subdirectory` block to
just after `add_subdirectory(src/lib ...)`. Keep it as a tracked patch applied per
release; it's stable and trivial to re-apply, and it's the price of an external
driver that uses the I2C framework.

## Milestones

1. **(this)** Out-of-tree scaffold publishing synthetic `sensor_flow_angle` in
   SITL. Proves the version-resilient plumbing before any hardware code.
2. Swap base to `I2CSPIDriver`, own the TCA9548A + three MS4525DO on the
   dedicated bus, publish `differential_pressure` for the pitot sensor (feeds the
   stock airspeed -> EKF2 path) and `sensor_flow_angle` for the reduction.
3. Bench + tunnel validation against the Calspan-calibrated map; calibration
   coefficients loaded from params so recal never needs a recompile.
