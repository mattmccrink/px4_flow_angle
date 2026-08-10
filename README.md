# px4_flow_angle

Out-of-tree PX4 driver for a five-hole probe: three **MS4515DO** digital pressure
sensors behind a **PCA9545A** I²C switch, publishing angle-of-attack / sideslip
(alpha/beta), dynamic pressure, and airspeed into the PX4 flight stack.

**Version: 0.2.6** &nbsp;·&nbsp; **Target: PX4 v1.17.0, board `px4_fmu-v5_default` (Pixhawk 4)**

This document takes you from a bare machine to a flashed, running driver. If you
are a student picking this up: read the whole "Build from a clean checkout"
section once before starting — several steps are non-obvious and the order
matters. The "Troubleshooting" table at the end maps every error we hit during
development to its fix.

---

## 1. What this is (and the design bet)

PX4 is a publish/subscribe system. This module is a *driver*: it reads the
sensors and publishes uORB topics. Everything downstream (the airspeed selector,
EKF2, logging, telemetry) subscribes to those topics without knowing a five-hole
probe produced them. The module publishes:

- `differential_pressure` — the pitot channel, into PX4's stock airspeed selector → EKF2.
- `sensor_flow_angle` — custom topic: alpha, beta, dynamic pressure, TAS, raw port differentials.
- `debug_array` (name `"flow"`) — streamed as `DEBUG_FLOAT_ARRAY` for live viewing in QGroundControl.

**The design bet is version resilience.** PX4-Autopilot stays a *pristine,
tag-pinned checkout*. All of our code lives in this repo and is pulled in at build
time via `EXTERNAL_MODULES_LOCATION`. Moving to a new PX4 release is: bump the tag,
re-apply the patches, rebuild, fix this small module against any changed API —
never a rebase against upstream.

---

## 2. Hardware (as built and verified)

| Channel | Role | PCA9545A channel | Mux select byte | Sensor I²C addr | Sensor range |
|:-------:|------|:----------------:|:---------------:|:---------------:|--------------|
| 0 | alpha    | 0 | 0x01 | 0x46 | ±4 inH₂O differential |
| 1 | airspeed | 1 | 0x02 | 0x46 | ±20 inH₂O differential |
| 2 | beta     | 2 | 0x04 | 0x46 | ±4 inH₂O differential |

- **Mux:** PCA9545A at I²C address **0x70** (A1=A0=GND). On the Pixhawk 4 it sits on
  **I²C bus 4**, which the board defines as an *external* bus.
- **Sensors:** all three MS4515DO are marked `3BK` on the package = 3.3 V supply /
  output type **B** (5–95 %) / interface **K** (I²C address **0x46**). All three
  therefore share address 0x46 — which is *why* the mux is required (three
  identical-address parts can only coexist behind separate switch channels).
- **Read the physical package markings, not the Digikey ordering string.** The
  ordering strings drifted between order and delivery during development and gave
  the wrong address and pressure-type. The `3BK … 004D / 020D` markings are ground truth.

---

## 3. Repository layout

```
px4_flow_angle/
├── README.md                      # this file
├── ROADMAP.md                     # milestone plan and design rationale
├── msg/
│   ├── CMakeLists.txt             # sets config_msg_list_external (SensorFlowAngle.msg)
│   └── SensorFlowAngle.msg        # -> topic sensor_flow_angle
├── src/
│   ├── CMakeLists.txt             # sets config_module_list_external -> modules/flow_angle
│   └── modules/flow_angle/
│       ├── CMakeLists.txt         # px4_add_module(... EXTERNAL) + out-of-tree compile fix
│       ├── FlowAngle.hpp / .cpp   # the driver (I2CSPIDriver)
│       ├── flow_angle_main.cpp    # command dispatch (start/stop/status/scan) + bus config
│       └── module.yaml            # parameters (FA_*)
└── tools/
    └── patch_px4.py               # two idempotent PX4-tree edits (re-run after any checkout)
```

---

## 4. Prerequisites

- **OS:** Ubuntu 20.04/22.04, native or under WSL2 on Windows 11. (Development was on WSL2.)
- **Disk/RAM:** ~15 GB free, ≥8 GB RAM for a comfortable build.
- **Board:** Pixhawk 4 (`px4_fmu-v5_default`) and a USB cable.
- **A five-hole-probe PCB** wired per §2 (or run in synthetic mode — see §8).

You do **not** need ROS 2 or a simulator for hardware work.

---

## 5. Build from a clean checkout

Do these in order. Steps 2 and 4 are the ones people skip and then spend a day debugging.

### 5.0 — Workspace layout

Put PX4 and this repo side by side:

```bash
mkdir -p ~/flowangle-ws && cd ~/flowangle-ws
# (extract this repo here so you have ~/flowangle-ws/px4_flow_angle)
```

### 5.1 — Get PX4, pinned to v1.17.0

The tag pin is non-negotiable. A checkout tracking `main`/beta produces a
misleading `expected template-name before '<'` error on `ModuleBase` (the beta
refactored that template out of `module.h`).

```bash
cd ~/flowangle-ws
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
git checkout v1.17.0
git submodule update --init --recursive
git describe --tags        # must print exactly: v1.17.0
```

### 5.2 — Install the pinned ARM toolchain (arm-none-eabi 9.3.1)

**This is the step that causes the most confusion.** PX4 v1.17.0 is built and
validated with GCC `arm-none-eabi` **9.3.1** (`gcc-arm-none-eabi-9-2020-q2-update`).
A distro package (`apt install gcc-arm-none-eabi`) installs a *much* newer GCC
(e.g. 14.x) that compiles most of the tree but hard-fails on a strict vendored
dependency (`uxrce_dds_client` / Micro-XRCE-DDS) with misleading errors. Use the
pinned version.

PX4's setup script fetches the right one and adds it to your PATH:

```bash
cd ~/flowangle-ws/PX4-Autopilot
bash ./Tools/setup/ubuntu.sh --no-sim-tools
# open a NEW shell (or: source ~/.bashrc) so PATH updates take effect
```

Then **verify** — this must show 9.3.1, not a distro GCC:

```bash
which arm-none-eabi-g++          # expect a /opt/gcc-arm-none-eabi-9-2020-q2-update/.../bin path
arm-none-eabi-g++ --version      # expect: ... 9.3.1 ...
```

If `which` still points at `/usr/bin/arm-none-eabi-g++` (a distro version), the
9.3.1 `bin/` is not ahead of `/usr/bin` on your PATH. Prepend it and clear bash's
command cache:

```bash
export PATH="/opt/gcc-arm-none-eabi-9-2020-q2-update/bin:$PATH"   # adjust to your install path
hash -r
which arm-none-eabi-g++ && arm-none-eabi-g++ --version            # re-verify -> 9.3.1
```

Make it permanent by putting that `export` at the **end** of `~/.bashrc` (after
anything that adds `/usr/bin`). Note: the 2020-era toolchain is 32-bit-linked and
on a modern Ubuntu may need `sudo apt install libncurses5 lib32z1` to run at all —
if `--version` errors on a binary that clearly exists, install those.

### 5.3 — Apply the two PX4-tree patches

This module needs two small, idempotent, content-matched edits to the PX4 tree.
**Re-run this after any `git checkout` of PX4-Autopilot** — checking out the tag
resets `CMakeLists.txt` and wipes the edits.

```bash
cd ~/flowangle-ws
python3 px4_flow_angle/tools/patch_px4.py $PWD/PX4-Autopilot
```

Watch the output. You want to see the two edits applied (or "already patched").
If either prints "skipping", the patch's anchor didn't match your tree — stop and
check you're on v1.17.0. The two edits are:

1. **External-modules reorder** in the root `CMakeLists.txt`. PX4 configures
   external modules *before* `src/lib` and the in-tree drivers, so an external
   module can't resolve `DEPENDS px4_work_queue drivers__device` — those targets
   don't exist yet. The patch moves the external block to *after* the in-tree
   module loop (still before the events/metadata/parameters libs). Required for
   the driver to compile.
2. **Log topic** in `src/modules/logger/logged_topics.cpp`: adds
   `add_topic("sensor_flow_angle", 100)` so the custom topic lands in the ulog.

### 5.4 — Make room in flash (disable an unused module)

The `px4_fmu-v5_default` image is very close to the flash ceiling. Adding this
module overflows `FLASH_AXIM` by ~10 KB at link time. Free space by disabling
`uxrce_dds_client` (the ROS 2 / micro-ROS bridge — irrelevant to this driver, and
the one whose vendored lib also fails if your toolchain is wrong):

```bash
cd ~/flowangle-ws/PX4-Autopilot
# edit boards/px4/fmu-v5/default.px4board and set (or remove the =y line):
#   CONFIG_MODULES_UXRCE_DDS_CLIENT=n
# or use the menu:
make px4_fmu-v5_default boardconfig    # Modules -> uxrce_dds_client -> off -> save/exit
```

### 5.5 — Build

```bash
cd ~/flowangle-ws/PX4-Autopilot
rm -rf build/                          # ensure a clean configure (see note)
make px4_fmu-v5_default EXTERNAL_MODULES_LOCATION=$PWD/../px4_flow_angle
```

`rm -rf build/` matters: several fixes above (the reorder, the parameter scan, the
external compile flags) take effect at *configure* time. A warm build directory
won't pick them up — when in doubt, wipe `build/`.

A successful build ends with a `.px4` file under
`build/px4_fmu-v5_default/`. If it fails, jump to the Troubleshooting table (§10).

### 5.6 — Flash

```bash
cd ~/flowangle-ws/PX4-Autopilot
make px4_fmu-v5_default upload         # with the board on USB
```

(Under WSL2 you may need `usbipd` to pass the USB device through to Linux, or flash
from QGroundControl on Windows using the built `.px4`.)

---

## 6. Run on hardware

The board's I²C4 port is an *external* bus, so start the driver against bus 4 at
the mux address, 400 kHz:

```
flow_angle start -b 4 -a 0x70 -f 400
flow_angle status
```

`status` should report `mode: HW` and three channels at addr `0x46` reading small,
changing pressures with `ok`. Because `FA_SIM_EN` defaults to 0, it reads hardware
without any parameter setup.

**Auto-start at boot (optional, once validated):** add the start line to the SD
card's extras script — `/fs/microsd/etc/extras.txt` — rather than editing ROMFS.
It runs late in boot, after core startup. Keep boot config on the SD card so you
can change it without reflashing.

---

## 7. Bench validation

Run these before trusting any computed angle:

```
flow_angle scan            # channel x address sweep, prints full 4-byte frames
flow_angle scan 20         # stream 20 frames/channel at ~2 Hz (watch counts under pressure)
```

`scan` runs on the command thread, so its output reaches the same console as
`start`/`status`; it pauses the sample loop for the duration so it owns the bus
(no need to stop the module). Expected sweep signature for this board: a frame
with status `0` (Normal) and sane counts at `0x46` on channels 0, 1, 2, and `--`
everywhere else. `flow_angle scan N` streams N frames per channel with decoded
counts and Pa — apply pressure and watch the numbers move to confirm a channel's
sense element is alive. `status` also prints each channel's last raw 4 bytes and
reject reason. Note: plain `i2cdetect -b 4` only ever sees `0x70` — it cannot look
behind the mux because it doesn't drive the channel-select register; `scan` does.

```
listener sensor_flow_angle
```

- At rest, `dynamic_pressure_pa` is near zero (a few Pa) and `valid` is false —
  the reduction is gated off below `FA_Q_MIN` (20 Pa) so near-zero q doesn't blow
  up the angle math. This is correct, not a fault.
- Put positive pressure on the pitot / blow across the probe: `dynamic_pressure_pa`
  climbs past 20 Pa, `valid` flips true, and `alpha_rad`/`beta_rad`/`true_airspeed_m_s`
  come alive (and the `debug_array` populates).
- **Differential-vs-gage check:** apply suction to one alpha port. A *differential*
  part swings `dp_alpha_pa` negative through the zero offset. A single-port gage
  part would sit near zero and rail — if you see that, you have the wrong part.

Live view: the `debug_array` named `flow` shows up in QGroundControl's MAVLink
Inspector as `DEBUG_FLOAT_ARRAY` (`data[0]=alpha°`, `data[1]=beta°`, `data[2]=TAS`).

---

## 8. Parameters (FA_*)

All parameters are generated from `module.yaml` and visible via `param show FA_*`.
The driver runs on compiled-in defaults if a param is absent, so it works even
before parameters generate.

| Parameter | Default | Meaning |
|-----------|:-------:|---------|
| `FA_SIM_EN` | 0 | 0 = read hardware. 1 = synthetic alpha/beta sweep, no I²C (SITL / no-hardware regression). |
| `FA_DBG_RAW` | 0 | 1 = dump raw 4-byte sensor frames (hex + status + counts) at ~4 Hz for diagnostics. |
| `FA_CONV_US` | 5000 | Post-measurement-request wait, µs. Raise (10000–20000) if a low-power channel returns stale/reset frames. |
| `FA_MR_MODE` | 0 | Measurement-request style: 0 = 0x00 data-byte write; 1 = address-only write (wakes some low-power parts). |
| `FA_RATE` | 50 | Sample rate, Hz. |
| `FA_Q_MIN` | 20 | Minimum dynamic pressure (Pa) for a valid angle; gates the reduction at low q. |
| `FA_RHO` | 1.225 | Air density for the TAS estimate. |
| `FA_OUT_TYP` | 1 | MS4515DO output type: 0 = A (10–90 %), 1 = B (5–95 %). Our parts are B. |
| `FA_CAL_A` / `FA_CAL_B` | 1.0 | Placeholder alpha/beta reduction gains (replaced by the milestone-3 tunnel map). |
| `FA_{A,AS,B}_CH` | 0/1/2 | Mux channel index per sensor (alpha / airspeed / beta). |
| `FA_{A,AS,B}_ADDR` | 70 | Sensor I²C address, decimal (70 = 0x46). |
| `FA_{A,AS,B}_RNG` | 4/20/4 | Sensor full-scale, inH₂O. |

Synthetic mode for a machine with no probe attached:

```
param set FA_SIM_EN 1
param save
flow_angle start -b 4 -a 0x70 -f 400   # publishes a synthetic sweep; no I2C traffic
```

---

## 9. How it fits into PX4 (brief)

`flow_angle` runs on the bus-specific work queue `wq:I2C4`, so PX4 serializes it
against any other driver on that bus automatically — no locking, no core changes.
Each cycle it selects a mux channel, issues the MS4515DO measurement/fetch
handshake, decodes the 4-byte frame (14-bit pressure, 11-bit temperature, type-B
constants), and after all three channels publishes the topics in §1. The pitot
channel feeds the stock airspeed selector → EKF2 exactly like any airspeed sensor;
alpha/beta stay on the custom topic. See `ROADMAP.md` for the milestone plan and
the (milestone-3) tunnel-calibration reduction that replaces the current linear
placeholder.

To visualize the whole uORB topology, PX4 ships a generator:
`python3 Tools/uorb_graph/create.py -o graph.json`, then open
`Tools/uorb_graph/index.html`. Live: `uorb top`, `uorb status sensor_flow_angle`,
`listener sensor_flow_angle`.

---

## 10. Troubleshooting

Every row here is an error we actually hit. The pattern for build errors: they
tend to appear one at a time, each revealed by fixing the previous — that's normal
for a first hardware bring-up, and it means you're advancing through real layers.

| Symptom | Cause | Fix |
|---------|-------|-----|
| `expected template-name before '<'` on `ModuleBase` | PX4 not on the v1.17.0 tag | `git checkout v1.17.0`; re-run `patch_px4.py` (§5.1, §5.3) |
| `expected class-name before ','` on `device::I2C` | External module didn't inherit `__PX4_NUTTX` / NuttX config; `device::I2C` compiled out | Fixed in this module's `CMakeLists.txt` (re-applies platform flags). Ensure you rebuilt clean (`rm -rf build/`). §5.5 |
| DDS / `uxrce_dds_client` sub-build fails | Wrong ARM toolchain (distro GCC 14 vs pinned 9.3.1) | Install and PATH the pinned toolchain. §5.2 |
| `CMAKE_CXX_COMPILER … not found in PATH` at configure | Toolchain not on PATH in this shell | `export PATH=.../gcc-arm-none-eabi-9.../bin:$PATH; hash -r`. §5.2 |
| `region FLASH_AXIM overflowed by N bytes` | fmu-v5 image at the flash ceiling | Disable `uxrce_dds_client`. §5.4 |
| `Parameter FA_SIM_EN not found` | `module.yaml` not scanned into the param metadata | Ensure `MODULE_CONFIG module.yaml` is in the module `CMakeLists.txt` (it is in v0.2.2); rebuild clean. |
| `Invalid unit in FA_…` (param gen) | A `unit:` value not on PX4's whitelist voids the whole yaml | Use only whitelisted units (`hPa` not `Pa`; grep `allowedUnits` in the param parser). |
| `flow_angle status` shows `mode: SIM` on hardware | `FA_SIM_EN` = 1 | `param set FA_SIM_EN 0; param save`, restart the module. |
| `i2cdetect -b 4` shows only 0x70 | Sensors are behind the mux; i2cdetect can't select channels | Use `flow_angle scan` (drives the channel-select register). §7 |
| Airspeed frozen (e.g. 5529 Pa; raw `3f ff ..`, temp field railed to −50 °C) | Low-power part not converting — returns its reset register (pressure max + temp min) | 0.2.3+ rejects it (shows `--`). To attempt a wake: raise `FA_CONV_US` (e.g. `param set FA_CONV_US 20000`) and/or try `FA_MR_MODE 1` (address-only MR). Use `flow_angle scan 20` under applied pressure to see if counts move. If temp stays railed and nothing moves, the part is dead — replace it. |
| Debug vector empty at rest but `status` shows data | Reduction gated by `q > FA_Q_MIN` at bench-zero q | Apply positive pressure; `valid` flips and the vector fills. Not a bug. §7 |

---

## 11. Version history

- **0.2.6** — low-power wake levers: `FA_CONV_US` (post-MR wait) and `FA_MR_MODE` (0 = data-byte / 1 = address-only measurement request), to diagnose a stuck low-power airspeed part; MR handling centralized in `send_mr()`.
- **0.2.5** — `scan` now runs on the command thread (output reaches the MAVLink console), prints full 4-byte frames, and pauses the sample loop for a clean bus; `flow_angle scan N` streams N frames/channel to watch counts under applied pressure; `status` shows the last raw frame + reject reason per channel.
- **0.2.4** — stale-frame rejection + bounded re-read on the low-power airspeed channel; physical-range backstop; `FA_DBG_RAW` raw-frame dump; per-channel `retries=` and reject/re-read/drop counters in `status`. (Supersedes 0.2.3, which had a member-scope compile error in `result_str`.)
- **0.2.2** — params generate via `MODULE_CONFIG module.yaml`; `FA_Q_MIN` unit fix; HW default (`FA_SIM_EN` default 0).
- **0.2.1** — out-of-tree compile-environment fix (`__PX4_NUTTX` + root includes re-applied in module CMakeLists).
- **0.2.0** — version counter added; `scan` verb; all three sensors confirmed at 0x46; bus 4.
- **Milestone 2** — `I2CSPIDriver` owning the PCA9545A + three MS4515DO; publishes `differential_pressure` + `sensor_flow_angle`.
- **Milestone 1** — out-of-tree scaffold publishing synthetic `sensor_flow_angle` in SITL.

The version string lives in `FlowAngle.hpp` (`FLOW_ANGLE_VERSION`) and prints on
start and in `flow_angle status`. Bump it on every released build so "which build
is on the board" is a one-line check.
