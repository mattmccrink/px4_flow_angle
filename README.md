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
├── CMakeLists.txt                 # registers modules/flow_angle
├── msg/
│   ├── CMakeLists.txt             # registers SensorFlowAngle.msg (out-of-tree)
│   └── SensorFlowAngle.msg        # -> topic sensor_flow_angle, struct sensor_flow_angle_s
└── src/modules/flow_angle/
    ├── CMakeLists.txt             # px4_add_module(... EXTERNAL)
    ├── FlowAngle.hpp / .cpp       # milestone-1 scaffold: publishes synthetic data
    └── flow_angle_params.c        # FA_SIM_EN, FA_RATE, FA_Q_MIN
```

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

## Likely first-build friction (expected, not alarming)

These are the spots most sensitive to the exact v1.17 API; paste the compiler
output and we fix them:

1. **External param codegen.** If `px4::params::FA_SIM_EN` etc. aren't generated
   for the external module, the `DEFINE_PARAMETERS` block won't compile. Fallback
   is to read them with `param_find("FA_SIM_EN")` / `param_get` directly — no
   dependency on the external param scan.
2. **Work-queue name / ScheduledWorkItem signature.** `wq_configurations::lp_default`
   and the `(name, config)` constructor are stable, but verify against
   `src/templates/` in your checkout if it complains.
3. **`task_id_is_work_queue`** idiom — confirm it's still the work-queue spawn
   pattern in v1.17 (it has been for several releases).

## Milestones

1. **(this)** Out-of-tree scaffold publishing synthetic `sensor_flow_angle` in
   SITL. Proves the version-resilient plumbing before any hardware code.
2. Swap base to `I2CSPIDriver`, own the TCA9548A + three MS4525DO on the
   dedicated bus, publish `differential_pressure` for the pitot sensor (feeds the
   stock airspeed -> EKF2 path) and `sensor_flow_angle` for the reduction.
3. Bench + tunnel validation against the Calspan-calibrated map; calibration
   coefficients loaded from params so recal never needs a recompile.
