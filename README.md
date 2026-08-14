# px4_flow_angle

An out-of-tree PX4 driver for a five-hole probe. It reads three MS45x differential
pressure sensors behind a PCA9545A I2C switch and publishes angle of attack,
sideslip, dynamic pressure, and airspeed.

**Version: 0.5.0. Target: PX4 v1.17.0, board `px4_fmu-v5_default` (Pixhawk 4).**

---

## 1. Overview

PX4 uses publish/subscribe. This driver reads the sensors and publishes three
uORB topics:

- `differential_pressure` — the pitot channel. It feeds the stock airspeed
  selector and EKF2.
- `sensor_flow_angle` — a custom topic with alpha, beta, dynamic pressure, and
  true airspeed.
- `debug_array` (name `flow`) — a live view in QGroundControl (`DEBUG_FLOAT_ARRAY`).

The driver keeps PX4-Autopilot unmodified. Your code stays in this repository. PX4
pulls it in at build time through `EXTERNAL_MODULES_LOCATION`. To move to a new PX4
release, change the tag and rebuild.

---

## 2. Hardware

The mux is a PCA9545A at address 0x70 on an external I2C bus (Pixhawk 4: bus 4).
Each sensor sits on one mux channel:

| Mux channel | Role | Sensor | Address | Range | Output type |
|:-----------:|------|--------|:-------:|-------|:-----------:|
| 0 | alpha | MS4515DO | 0x46 | ±4 inH2O | B |
| 1 | airspeed | MS4525DO | 0x28 | ±1 psi | A |
| 2 | beta | MS4515DO | 0x46 | ±4 inH2O | B |

Read the sensor address and type from the physical package marking, not from the
order string. Confirm the address with `flow_angle scan`.

The two MS4515DO parts share address 0x46, so the mux is required. Sensor
addresses, ranges, and types are set in the SD config file (Section 6).

---

## 3. Prerequisites

- Ubuntu 20.04 or 22.04, native or under WSL2.
- About 15 GB of free disk space.
- A Pixhawk 4 and a USB cable.

You do not need ROS 2 or a simulator.

---

## 4. Build

Do the steps in order.

### Step 1 — Get PX4 and set the version

```bash
mkdir -p ~/flowangle-ws && cd ~/flowangle-ws
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
git checkout v1.17.0
git submodule update --init --recursive
git describe --tags        # must print v1.17.0
```

Use the exact tag. A different version changes internal APIs and fails to build.

### Step 2 — Install the ARM toolchain

PX4 v1.17.0 builds with `arm-none-eabi` GCC 9.3.1.

```bash
cd ~/flowangle-ws/PX4-Autopilot
bash ./Tools/setup/ubuntu.sh --no-sim-tools
```

Open a new shell. Then confirm the version:

```bash
arm-none-eabi-g++ --version    # must report 9.3.1
```

Use this version. A newer distribution GCC compiles most of PX4 but fails on one
vendored library.

### Step 3 — Add this module

Extract this repository next to PX4-Autopilot:

```
~/flowangle-ws/PX4-Autopilot
~/flowangle-ws/px4_flow_angle
```

### Step 4 — Patch the PX4 tree

```bash
cd ~/flowangle-ws
python3 px4_flow_angle/tools/patch_px4.py $PWD/PX4-Autopilot
```

Re-run this after every `git checkout` of PX4-Autopilot. A checkout resets the
files the patch edits.

### Step 5 — Free flash space

The `px4_fmu-v5_default` image is near the flash limit. Disable one unused module:

```bash
cd ~/flowangle-ws/PX4-Autopilot
make px4_fmu-v5_default boardconfig
# Modules -> uxrce_dds_client -> off. Save and exit.
```

### Step 6 — Build

```bash
cd ~/flowangle-ws/PX4-Autopilot
rm -rf build/
make px4_fmu-v5_default EXTERNAL_MODULES_LOCATION=$PWD/../px4_flow_angle
```

Delete `build/` before the build. Several settings apply at configure time only.

### Step 7 — Upload

```bash
make px4_fmu-v5_default upload    # Pixhawk 4 on USB
```

---

## 5. Run

The start command takes no bus arguments. The driver probes the external I2C
buses for the switch.

| Command | Action |
|---------|--------|
| `flow_angle start` | Start the driver. |
| `flow_angle status` | Show version, mode, per-channel state, and counters. |
| `flow_angle scan` | Read all mux channels. Print raw bytes, status, counts, and temperature. |
| `flow_angle scan N` | Stream N frames per channel. Use it to watch counts under applied pressure. |
| `flow_angle null` | Capture the per-channel zero offset. Cap the probe first. |
| `flow_angle null N` | Average N samples for the zero offset. |
| `flow_angle stop` | Stop the driver. |

The driver prints startup messages (config load, channel verification) to the boot
log. Read them with `dmesg`. The MAVLink console does not always show them.

### First bring-up

1. Start the driver: `flow_angle start`.
2. Read the boot log: `dmesg`. Confirm `config: loaded ...` and `verify ... OK` for
   each channel.
3. Confirm the sensors: `flow_angle scan`. Each channel shows a `Normal` status,
   sane counts, and an ambient temperature near 20 °C.
4. Cap the probe. Capture the zero: `flow_angle null`, then `param save`.
5. Check the output: `listener sensor_flow_angle`. At rest, `dynamic_pressure_pa`
   is near zero.

---

## 6. SD-card configuration

Copy the files under `sd_card/` to the flight controller's SD card. Keep the
layout: `sd_card/etc/...` goes to `/fs/microsd/etc/...`.

### Channel config

The driver reads `/fs/microsd/etc/flow_angle/config.txt` at start when
`FA_CFG_SD = 1`. The file sets each channel's role, address, range, units, and
output type. Edit the file to match your sensors. A recompile is not needed.

```
version = 1
ch0.role = alpha      ch0.addr = 0x46   ch0.range = 4.0   ch0.units = inH2O   ch0.type = B
ch1.role = airspeed   ch1.addr = 0x28   ch1.range = 1.0   ch1.units = psi     ch1.type = A
ch2.role = beta       ch2.addr = 0x46   ch2.range = 4.0   ch2.units = inH2O   ch2.type = B
```

Each channel needs `role`, `addr`, `range`, `units`, and `type`. Units are
`inH2O`, `psi`, `kPa`, `hPa`, or `Pa`. Type is `A` (10–90%) or `B` (5–95%). Read
the type from the sensor part number.

If the file is missing, has the wrong version, or has an incomplete channel, the
driver ignores the file, uses compiled defaults, and logs the reason.

### Sensor acceptance gate

At start, the driver reads each channel's temperature. A working sensor reports
ambient temperature. A dead, absent, or unpowered sensor reports −50 °C. A channel
that fails is logged and marked `UNVERIFIED` in `flow_angle status`.

### Zero the sensors

Cap the probe. Run `flow_angle null`, then `param save`. The driver stores the
per-channel offsets in `FA_OFF_A`, `FA_OFF_AS`, and `FA_OFF_B`, and subtracts them
before the reduction. The zero drifts with temperature. Re-null each session.

Use `flow_angle null` for the airspeed zero, not the QGroundControl airspeed
calibration. The driver owns the airspeed zero. With `FA_ZERO_DPRES = 1` (default),
it clears the stock `SENS_DPRES_OFF` to 0 at each start and logs a warning if it
was not zero. This prevents a stale stock offset from stacking on the driver's
zero across reboots. Set `FA_ZERO_DPRES = 0` to only warn.

### In-flight failure alert

The driver monitors each channel during flight. If a channel returns no valid
frame for `FA_FAIL_MS` (default 1000 ms), the driver posts a QGroundControl alert
(STATUSTEXT) and marks the channel `FAILED` in `flow_angle status`. It re-posts the
alert every 30 s while the channel stays failed. The airspeed channel and the
alpha/beta channels use separate messages, because airspeed loss also affects the
airspeed estimate and EKF2.

### Autostart

PX4 runs `/fs/microsd/etc/extras.txt` from the SD card late in boot. The sample
file starts the driver:

```
flow_angle start
```

Validate the driver by hand first (`start`, `scan`, `status`, all channels
verified). Add the autostart line last. An autostarted driver has no operator to
see a bad config or a dead sensor.

---

## 7. Parameters

| Parameter | Default | Meaning |
|-----------|:-------:|---------|
| `FA_MUX_ADDR` | 112 | PCA9545A address (decimal; 112 = 0x70). |
| `FA_SIM_EN` | 0 | 0 = read hardware. 1 = synthetic path, no I2C. |
| `FA_CFG_SD` | 1 | 1 = load the SD config at start. 0 = use compiled defaults. |
| `FA_ZERO_DPRES` | 1 | 1 = clear `SENS_DPRES_OFF` to 0 at start. 0 = warn only. |
| `FA_RATE` | 50 | Sample rate, Hz. |
| `FA_FAIL_MS` | 1000 | Dropout time (ms) before a QGC failure alert. |
| `FA_Q_MIN` | 20 | Minimum dynamic pressure (Pa) for a valid angle. |
| `FA_RHO` | 1.225 | Air density for the airspeed estimate. |
| `FA_CONV_US` | 5000 | Wait after a measurement request, µs. |
| `FA_MR_MODE` | 0 | Measurement request: 0 = data-byte write, 1 = address-only. |
| `FA_DBG_RAW` | 0 | 1 = log raw frame bytes for diagnostics. |
| `FA_OFF_A` / `FA_OFF_AS` / `FA_OFF_B` | 0.0 | Per-channel zero offset (Pa), set by `flow_angle null`. |
| `FA_CAL_A` / `FA_CAL_B` | 1.0 | Placeholder reduction gains (replaced by the tunnel map). |

---

## 8. Troubleshooting

| Symptom | Cause | Action |
|---------|-------|--------|
| No `config:` message at start | `FA_CFG_SD = 0`, or you read the MAVLink console | Set `FA_CFG_SD 1`. Read the boot log with `dmesg`. |
| `config: ... not found` | The file is not at the expected path | Confirm `/fs/microsd/etc/flow_angle/config.txt` exists. The `etc` folder must be at the card root. |
| Channel shows `UNVERIFIED` | The sensor did not report a sane temperature | Check the sensor address, power, and wiring. Run `flow_angle scan`. |
| `i2cdetect` shows only 0x70 | The sensors are behind the mux | Use `flow_angle scan`. It selects each mux channel. |
| Airspeed reads negative in QGroundControl | The published pressure carries a zero offset or a sign error | Run `flow_angle null`. If it stays negative under flow, swap the total and static tubes. |
| Compile stops at `device::I2C` | The external module did not get the platform flags | Rebuild clean (`rm -rf build/`). The fix is in the module `CMakeLists.txt`. |
| `Parameter FA_... not found` | The `module.yaml` was not scanned | Confirm `MODULE_CONFIG module.yaml` is in the module `CMakeLists.txt`. Rebuild clean. |

---

## 9. Repository layout

```
px4_flow_angle/
├── README.md
├── ROADMAP.md
├── msg/                 # SensorFlowAngle.msg -> sensor_flow_angle
├── src/modules/flow_angle/
│   ├── FlowAngle.hpp / .cpp
│   ├── flow_angle_main.cpp
│   ├── module.yaml      # parameters (FA_*)
│   └── CMakeLists.txt
├── sd_card/etc/         # copy to /fs/microsd/etc/
│   ├── extras.txt
│   └── flow_angle/config.txt
└── tools/patch_px4.py
```

The version string is in `FlowAngle.hpp` (`FLOW_ANGLE_VERSION`). It prints at start
and in `flow_angle status`. Change it for each release.
