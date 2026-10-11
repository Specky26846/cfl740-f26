# CubesatRef — F´ reference system

(README rewritten by Claude, reviewed by Yukti Vijay and Laura Fernandes)

This repo is an F´ (F Prime) reference deployment for a cubesat device-manager stack: temperature, IMU, GPS, and magnetometer sensors, a LoRa radio, and a system-health manager, all running on **Raspberry Pi (Linux ARM)** over `LinuxI2cDriver` / `LinuxSpiDriver` / `LinuxGpioDriver`.

**F´ (F Prime) is a component-driven framework for spaceflight and other embedded software applications.**
Visit the F´ website: https://fprime.jpl.nasa.gov.

---

## Table of Contents

1. [Repository Structure](#repository-structure)
2. [Components](#components)
3. [State Machines](#state-machines)
4. [Topology & Rate Groups](#topology--rate-groups)
5. [Build Instructions](#build-instructions)
6. [Running & Testing with GDS](#running--testing-with-gds)
7. [Known Issues](#known-issues)
8. [Contributing / Code Review Workflow](#contributing--code-review-workflow)

---

## Repository Structure

```
├── CMakeLists.txt               # Top-level CMake — registers both subdirectories below, project(CubesatRef)
├── CMakePresets.json
├── requirements.txt              # Pins fprime v4.0.0
├── settings.ini                  # fprime-util project config
├── lib/fprime/                   # F Prime framework submodule (v4.0.0)
├── fprimecubesat/                 # Deployment — the executable + topology
│   ├── Main.cpp
│   └── Top/
│       ├── instances.fpp         # Component instances, base IDs, priorities
│       ├── topology.fpp          # Port connections & rate group wiring
│       ├── fprimecubesatTopology.cpp/hpp
│       └── fprimecubesatTopologyDefs.hpp
└── CubesatRef/
    └── Components/                # Project-wide component library (module Managers)
        ├── Types/                 # Shared FPP types (GeometricVector3, ComponentHealth port)
        ├── Led/                   # Status/blink LED over GPIO
        ├── SystemManager/         # Aggregates sensor health, drives system state machine
        ├── RadioManager/          # LoRa radio over SPI
        ├── IMUManager/            # ICM-20649 IMU (accel + gyro) over I2C
        ├── NavigationManager/     # u-blox NEO-M9N GPS over I2C (UBX binary protocol)
        ├── MagnetometerManager/   # PNI RM3100 magnetometer over I2C
        └── TempManager/           # TMP102 temperature sensor over I2C
```

`fprimecubesat` is the deployment (what gets built and flashed); `CubesatRef/Components` is the reusable component library it's built from. Every manager under `Components/` shares `module Managers` in its `.fpp`. (Note: the checkout directory itself may still be named `myproject` on disk — that's just your local clone's folder name and isn't referenced anywhere in the build.)

---

## Components

### IMUManager

| | |
|---|---|
| **Device** | ICM-20649 6-axis IMU |
| **I2C address** | `0x68` |
| **Bus** | `imuI2cDriver` |
| **Rate group** | rateGroup1 — 1Hz |
| **Base ID** | `0x10022000` |

**Telemetry:** `AccelX/Y/Z` (m/s²), `GyroX/Y/Z` (deg/s), `ImuTemp` (°C)
**Commands:** `CALIBRATE_IMU` (logs `ImuCalibrationStarted`, no offset logic yet), `ENABLE_SIM`, `DISABLE_SIM`
**Events:** `StateChange(newState)`, `ImuCalibrationStarted`

### MagnetometerManager

| | |
|---|---|
| **Device** | PNI RM3100 |
| **I2C address** | `0x20` |
| **Bus** | `magI2cDriver` |
| **Rate group** | rateGroup3 — 2.5Hz |
| **Base ID** | `0x10024000` |

**Telemetry:** `MagX/Y/Z` (µT)
**Commands:** `CALIBRATE_MAG` (logs `MagCalibrationStarted`, no offset logic yet), `ENABLE_SIM`, `DISABLE_SIM`
**Events:** `StateChange(newState)`, `MagCalibrationStarted`, `MagReading(mx, my, mz)` (throttled, activity low)

### NavigationManager

| | |
|---|---|
| **Device** | u-blox NEO-M9N GPS |
| **I2C address** | `0x42` |
| **Bus** | `gpsI2cDriver` |
| **Rate group** | rateGroup1 — 1Hz |
| **Base ID** | `0x10023000` |

**Telemetry:** `Latitude`/`Longitude` (°, F64), `Altitude` (m), `GroundSpeed` (m/s), `NumSatellites`
**Commands:** `GPS_RESET`, `ENABLE_SIM` (hardcoded coords), `DISABLE_SIM`
**Events:** `StateChange(newState)`, `GpsFixAcquired(numSats)`, `GpsFixLost`, `GpsReading(lat, lon, alt, sats)` (throttled, activity low)
**Implementation notes:** polls UBX-NAV-PVT (binary protocol, not NMEA); skips a tick rather than reporting zeros when a valid fix can't be parsed.

### TempManager

| | |
|---|---|
| **Device** | TMP102 |
| **I2C address** | `0x4a` (ADD0 tied to ground on this board — not the default `0x48`) |
| **Bus** | `tempI2cDriver` |
| **Rate group** | rateGroup1 — 1Hz |
| **Base ID** | `0x10025000` |

**Telemetry:** `Temperature` (°C)
**Commands:** `READ_TEMP` (one-shot), `ENABLE_SIM`, `DISABLE_SIM`
**Events:** `StateChange(newState)`, `TempReading(temperature)` (throttled, activity low)

### RadioManager

| | |
|---|---|
| **Device** | LoRa radio module |
| **Bus** | SPI via `radioSpiDriver` |
| **Rate group** | rateGroup1 — 1Hz |
| **Base ID** | `0x10021000` |

**Telemetry:** `PacketsReceived`, `PacketsTransmitted`, `LastRssi` (dBm), `RadioStateChannel`, `TxPower` (dBm), `Frequency` (MHz), `SpreadingFactor`, `Bandwidth`, `CodingRate`
**Commands:** `GET_CONFIG`, `SET_TX_POWER(power)`, `SET_FREQUENCY(frequency)`, `SET_LORA_PARAMS(sf, bw, cr)`, `TRANSMIT_PACKET(data)` — max 229 bytes, see below, `GET_TELEMETRY`, `RADIO_RESET`
**Events:** `ConfigUpdated(setting)`, `ConfigReport(freq, txPower, sf)`, `PacketTransmitted(packetSize)`, `PacketReceived(packetSize)`, `RadioError(spiStatus)`, `RadioResetComplete`
**Implementation notes:** every command handler is backed by a real `spiReadWrite` transaction (guarded by `isConnected_spiReadWrite_OutputPort`) — not stubbed.

**Packet format:** `TRANSMIT_PACKET` wraps its payload as a DAP (Data Application Packet) inside a RAP (Radio Application Packet), per the MXL/GRIFEX radio packet spec — see [`docs/MXL_RAP_Format.md`](CubesatRef/Components/RadioManager/docs/MXL_RAP_Format.md) for the full format and [`RadioManager.cpp`](CubesatRef/Components/RadioManager/RadioManager.cpp)'s `TRANSMIT_PACKET_cmdHandler` for the framing code (sync bytes, IDs, Flags, Length, two Fletcher-16 checksums). Known simplifications — see the doc's "Deviations" section for the full list — HMAC is stubbed to zero (no SHA1 in this repo yet), the satellite/application IDs are placeholders, and there's no fragmentation or RX-side parsing yet. **The framing logic has been code-reviewed against the spec but not yet exercised end-to-end against a running deployment** — verify with a real `TRANSMIT_PACKET` call and inspect the SPI bytes (or the `PacketTransmitted` event's byte count, which should be `26 + len(data)`) before relying on it.

### SystemManager

| | |
|---|---|
| **Bus** | None — aggregates health from the other managers over `ComponentHealth` ports |
| **Rate group** | rateGroup1 — 1Hz |
| **Base ID** | `0x10020000` |

**Telemetry:** `SystemState` (0=nominal, 1=degraded, 2=reboot), `CpuUsage` (%), `MemUsage` (%), `SystemUptime` (s), `TotalComponentFaults`, `DegradedTicks`
**Commands:** `REPORT_STATUS`, `INJECT_COMPONENT_FAULT`, `CLEAR_FAULT`, `ESCALATE`, `CLEAR_SENSOR_FAULT`, `CLEAR_TEMP_FAULT`, `CLEAR_GPS_FAULT`, `CLEAR_MAG_FAULT`
**Events:** `HealthCheckComplete`, `EnteredDegraded(faultCount)`, `EscalatedToReboot`, `ComponentFaultDetected(faultCount)`, `RebootComplete`
**Implementation notes:** runs its own `SystemManagerStateMachine` (`NOMINAL` → `DEGRADED` → `REBOOT`) and drives `gpioDriver2` as a physical status LED — HIGH in `NOMINAL`, LOW on `REBOOT`/fault.

### Led

Two instances (`led`, `led2`) — a simple blink-on-interval component, distinct from the sensor managers (no I2C/SPI bus, no fault-retry state machine).
**Commands:** `BLINKING_ON_OFF(onOff)`
**Telemetry:** `BlinkCommandCount`, `LEDTransitions`, `BlinkingState`
**Param:** `BLINK_INTERVAL` (ticks between toggles)

---

## State Machines

Every active manager with real failure modes (the four sensors + `SystemManager`) is driven by an F´ state machine (`.fpp` state machine + `_action_*` handlers in the component `.cpp`), not ad-hoc if/else in the command/run handlers. Two distinct shapes are used, described below.

### Sensor managers: INIT / RUNNING / FAULT / SIM

`IMUManager`, `TempManager`, `NavigationManager`, and `MagnetometerManager` all share the same state machine shape (defined per-component in `<Component>StateMachine.fpp`, e.g. `IMUManagerStateMachine.fpp`):

```
INIT ──success──> RUNNING ──fault──> FAULT ──success──> RUNNING
  │                  │                  │
  └──enableSim──> SIM <──enableSim──────┘
                   │
              disableSim
                   │
                   v
                 INIT (re-probes real hardware)
```

| State | Entered via | Tick action | What it does |
|---|---|---|---|
| `INIT` | boot (initial state) | `doInit` | Attempts one real hardware read to confirm the sensor is on the bus |
| `RUNNING` | `success` from `INIT`/`FAULT` | `doRead` | Reads real hardware every tick, pushes telemetry |
| `FAULT` | `fault` from `INIT`/`RUNNING` | `doFaultRecovery` | Retries the read every tick; `success` signal returns to `RUNNING` |
| `SIM` | `enableSim` from any state | `doSimRead` | Generates fake data every tick, no hardware touched |

**`INIT` always attempts a real hardware read first** — this is the default path, not an opt-in, so behavior on target hardware (the Pi) is what you get without touching any commands. Only a genuine I2C error moves a sensor to `FAULT`, which retries every tick until the bus comes back. `SIM` is reachable only via the `ENABLE_SIM`/`DISABLE_SIM` commands (`disableSim` routes back through `INIT` to re-probe, rather than jumping straight to `RUNNING`).

Every transition logs a `StateChange(newState)` event, so watching the GDS event log tells you exactly which state each sensor is in.

On a Mac dev build (no I2C bus), `Drv.LinuxI2cDriver` is swapped for `LinuxI2cDriverStub` (a framework-provided stub, `FPRIME_USE_STUBBED_DRIVERS`), which always returns `I2C_OK` with an all-zero buffer — so on Mac, sensors land in `RUNNING` showing flat zeros unless you explicitly `ENABLE_SIM`. That's expected, not a bug.

### SystemManager: NOMINAL / DEGRADED / REBOOT

`SystemManagerStateMachine` tracks overall system health by listening on four `ComponentHealth` input ports (`sensorHealth`, `tempHealth`, `gpsHealth`, `magHealth`) — each sensor manager's `healthOut` port calls back `true`/`false` after every read attempt (see `doRead`/`doFaultRecovery` in each sensor's `.cpp`), so this state machine reacts to the sensors' own state changes rather than polling them.

```
NOMINAL ──sensorFault/tempFault/gpsFault/magFault──> DEGRADED
                                                          │
                                        ┌─────────────────┼──────────────────┐
                                        │                                    │
                                  faultCleared                          escalate
                                        │                                    │
                                        v                                    v
                                    NOMINAL                               REBOOT
                                                                              │
                                                                       rebootComplete
                                                                              │
                                                                              v
                                                                          NOMINAL
```

| State | Tick action | Status LED (`gpioDriver2`) | Exit conditions |
|---|---|---|---|
| `NOMINAL` | `runHealthCheck` | solid on | any single sensor fault → `DEGRADED` |
| `DEGRADED` | `monitorDegraded` | blinking | all four sensors healthy again → `NOMINAL`; stuck degraded for `ESCALATION_THRESHOLD` = 500 ticks (~500s at 1Hz) → auto-`escalate` to `REBOOT` |
| `REBOOT` | `performReboot` | off | operator sends `CLEAR_FAULT` command → `rebootComplete` → `NOMINAL` |

`ESCALATE` and `CLEAR_FAULT` are also exposed as commands, so escalation/recovery can be forced manually from GDS instead of waiting on the sensors or the 500-tick timer. `INJECT_COMPONENT_FAULT`/`CLEAR_SENSOR_FAULT`/`CLEAR_TEMP_FAULT`/`CLEAR_GPS_FAULT`/`CLEAR_MAG_FAULT` exist for testing this state machine without physically unplugging hardware.

---

## Topology & Rate Groups

### Rate Groups

| Rate Group | Frequency | Members |
|---|---|---|
| `rateGroup1` | 1Hz | `systemManager`, `radioManager`, `tempManager`, `navigationManager`, `imuManager`, `led` |
| `rateGroup2` | 10Hz | `cmdSeq`, `led2` |
| `rateGroup3` | 2.5Hz | Health, buffer managers, data products, `magnetometerManager` |

`imuManager`/`navigationManager`/`magnetometerManager` were originally on the 10Hz group but were moved off it (commit `35c6a9f`) — blocking `usleep()` calls inside the mag/gps read handlers were stalling everything else scheduled on that rate group. Mag was switched to a two-phase poll (trigger on tick N, read on tick N+1) instead of sleeping, and GPS drops the sleep and skips ticks it can't parse. See the inline notes in `instances.fpp`/`topology.fpp` before moving anything back onto rateGroup2.

### Bus Driver Instances

| Instance | Type | Pin/Bus | Connected To |
|---|---|---|---|
| `imuI2cDriver` | `Drv.LinuxI2cDriver` | `/dev/i2c-1` | `imuManager` |
| `magI2cDriver` | `Drv.LinuxI2cDriver` | — | `magnetometerManager` |
| `gpsI2cDriver` | `Drv.LinuxI2cDriver` | — | `navigationManager` |
| `tempI2cDriver` | `Drv.LinuxI2cDriver` | — | `tempManager` |
| `radioSpiDriver` | `Drv.LinuxSpiDriver` | — | `radioManager` |
| `gpioDriver` | `Drv.LinuxGpioDriver` | BCM 13 | `led` |
| `gpioDriver2` | `Drv.LinuxGpioDriver` | BCM 17 | `systemManager` status LED |
| `gpioDriver3` | `Drv.LinuxGpioDriver` | BCM 27 (**placeholder — not yet confirmed against real wiring**) | `led2` |

### Base ID Convention

All instances follow the pattern `0xDSSCCxxx`:
- `D` = deployment digit (`1` for this deployment)
- `SS` = subtopology (`00` for main topology)
- `CC` = component index
- `xxx` = reserved for internal component items

---

## Build Instructions

### Testing This Repo — Mac vs Pi

Two different things to test, depending on what you're checking:

| | Mac (native) | Raspberry Pi (cross-compiled) |
|---|---|---|
| **Proves** | Code/wiring compiles and runs | Sensors actually work against real hardware |
| **Sensor data** | Flat zeros by default (`LinuxI2cDriverStub` — see [State Machines](#state-machines)); send `ENABLE_SIM` for fake data | Real readings once wired, or `FAULT` with retry if not |
| **Speed** | Fast — no cross-compile, no scp | Slower — Docker build + deploy round-trip |
| **When to use** | Iterating on code/topology changes | Verifying a change actually works end-to-end |

**Mac path** — see [Mac — native build & GDS](#mac--native-build--gds-simulation) below. Always kill the spawned processes when you're done testing a change, before starting the next one:
```bash
ps aux | grep -iE "fprimecubesat|fprime_gds|fprime-gds" | grep -v grep | awk '{print $2}' | xargs -r kill
```

**Pi path** — needs Docker Desktop running first, then see [Docker — cross-compile for Raspberry Pi](#docker--cross-compile-for-raspberry-pi-aarch64) and [Deploy & run on the Pi](#deploy--run-on-the-pi) below.

**NOTE:** If compiling using WSL on Windows, you will also need to use Docker to cross-compile for aarch64-linux on the Pi, either using Docker or another cross-compilation method.

### Prerequisites

- Python 3.8+
- CMake ≥ 3.24.2
- F Prime v4.0.0 (pinned via `requirements.txt`)
- ARM cross-compiler, or Docker, for building for the Pi

### Mac — native build & GDS (simulation)

```bash
cd /Users/yuktivijay/myproject
source fprime-venv/bin/activate
fprime-util purge                        # only if generate fails or after big structural changes
fprime-util generate                     # only when FPP files/components change
fprime-util build
fprime-gds --ip-port 50002 --ip-client   # opens at http://localhost:5000
```

**`--ip-client` is required.** The deployment's `comDriver` is a `Drv.TcpServer` — it listens. GDS's comm bridge must explicitly be told to dial in as the client, or both sides try to bind the same port and the connection fails (`SOCK_FAILED_TO_BIND`). Avoid ports 50000/50001/50050 — those are `fprime-gds`'s own internal defaults and can collide even on an unrelated chosen port.

### Docker — cross-compile for Raspberry Pi (aarch64)

```bash
docker run --rm -it \
  -v /path/to/project:/project \
  nasafprime/fprime-arm \
  bash -c "cd /project && pip install -r requirements.txt && fprime-util generate aarch64-linux -f && fprime-util build aarch64-linux"
```

`aarch64-linux` is a **positional** argument, not `-p` (`-p` means `--path`).

Outputs:
- Binary: `build-artifacts/aarch64-linux/fprimecubesat/bin/fprimecubesat`
- Dictionary: `build-artifacts/aarch64-linux/fprimecubesat/dict/fprimecubesatTopologyDictionary.json`

### Deploy & run on the Pi

```bash
scp build-artifacts/aarch64-linux/fprimecubesat/bin/fprimecubesat <pi-user>@<pi-ip>:~/
ssh <pi-user>@<pi-ip>
chmod +x fprimecubesat
./fprimecubesat -a 0.0.0.0 -p 50000 &
```

Then connect from the Mac:
```bash
source fprime-venv/bin/activate
fprime-gds -n --ip-client \
  --dictionary build-artifacts/aarch64-linux/fprimecubesat/dict/fprimecubesatTopologyDictionary.json \
  --ip-address <pi-ip> --ip-port 50000
```

**Pi one-time setup:**
```bash
sudo raspi-config nonint do_i2c 0
sudo reboot
```

**Verify I2C devices:**
```bash
i2cdetect -y 1
# Expect: 0x20 (RM3100 mag), 0x42 (NEO-M9N GPS), 0x4a (TMP102 temp), 0x68 (ICM-20649 IMU)
```

---

## Running & Testing with GDS

Once connected, in the GDS Channels tab, sensors should show `RUNNING` state (real hardware) via `<manager>.StateChange`. On real hardware with sensors wired, values update every tick per the rate group table above.

Useful smoke-test commands:
```
systemManager.REPORT_STATUS
imuManager.CALIBRATE_IMU
magnetometerManager.CALIBRATE_MAG
navigationManager.GPS_RESET
tempManager.READ_TEMP
radioManager.GET_CONFIG
```

To force fake data without hardware (e.g. dev/demo), send `<manager>.ENABLE_SIM`; `<manager>.DISABLE_SIM` re-probes real hardware.

### Running Unit Tests

```bash
fprime-util check
```

> No unit tests exist yet for the `Managers` components — see [Known Issues](#known-issues).

---

## Known Issues

| Issue | Status |
|---|---|
| `CALIBRATE_IMU` and `CALIBRATE_MAG` log an event and return OK but apply no calibration offsets | Open |
| `gpioDriver3` (drives `led2`) is on placeholder pin BCM 27 — not confirmed against real wiring | Open |
| No unit tests exist for any `Managers` components | Open |
| `NavigationManager` UBX-NAV-PVT parsing has not been validated against real hardware on this branch | Open |
| `RadioManager` RAP/DAP framing (`TRANSMIT_PACKET`) is code-reviewed against spec but not yet exercised end-to-end against a running deployment — see [RadioManager](#radiomanager) | Open |
| RAP header uses placeholder satellite/application IDs and a zeroed HMAC — see `docs/MXL_RAP_Format.md`'s "Deviations" section | Open |

---

## Contributing / Code Review Workflow

### Branching

Work off a new branch for changes to the `Managers` module. Do not merge to `main` until sensor behavior is validated on real Pi hardware.

### Commit Style

Keep commits scoped to one component or one topology change at a time:
```
add <ComponentName> (<brief description>)
wire up <feature> in topology
fix <ComponentName>: <what changed and why>
```

### Review Checklist

- [ ] Port connections in `topology.fpp` are correct — check both directions (output → input)
- [ ] New instances in `instances.fpp` have unique base IDs that don't collide with existing assignments
- [ ] Sensor `doInit` attempts real hardware first — `SIM` should never be the unconditional default (see [State Machines](#state-machines))
- [ ] `run_handler`/output calls guard against unconnected ports before calling `_out()`
- [ ] Events use appropriate severity (`ACTIVITY_LO` for nominal, `WARNING_HI` for bus errors)
- [ ] No hardcoded I2C/SPI addresses that should be configurable via `configure()`
